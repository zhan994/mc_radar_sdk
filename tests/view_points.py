#!/usr/bin/env python3
"""Live 3D display driven by radar_test_main --json (no ROS required)."""

import argparse
import json
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receiver", type=Path,
                        default=Path(__file__).resolve().parents[1] / "build/radar_test_main")
    parser.add_argument("--range", type=float, default=20.0, dest="view_range",
                        help="initial forward viewing distance in meters (default: 20)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="automatically close after this many seconds; 0 runs until closed")
    parser.add_argument("--snapshot", type=Path,
                        help="optional PNG path to save when closing")
    args = parser.parse_args()
    if not 0 < args.view_range < float("inf") or not 0 <= args.duration < float("inf"):
        parser.error("--range must be positive and --duration must be nonnegative")
    if not args.receiver.is_file():
        parser.error("receiver is missing; build it with: cmake --build build --target radar_test_main")

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        import numpy as np
    except ImportError as error:
        parser.error(f"{error}; this viewer requires matplotlib and numpy")

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(11, 7))
    fig.canvas.manager.set_window_title("Radar target points - 3D")
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.02, right=0.86, bottom=0.12, top=0.92)
    ax.set_xlabel("X forward (m)")
    ax.set_ylabel("Y left (m)")
    ax.set_zlabel("Z up (m)")
    ax.set_box_aspect((2, 2, 1))
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False

    def reset_view():
        distance = args.view_range
        ax.set_xlim(0, distance)
        ax.set_ylim(-distance / 2, distance / 2)
        ax.set_zlim(-distance / 4, distance / 4)
        ax.view_init(elev=22, azim=-55)

    reset_view()
    arrow_length = min(args.view_range / 5, 2.0)
    for vector, color in (((1, 0, 0), "#ff6b6b"),
                          ((0, 1, 0), "#57d68d"),
                          ((0, 0, 1), "#58a6ff")):
        ax.quiver(0, 0, 0, *vector, length=arrow_length, color=color, linewidth=2)
    ax.scatter([0], [0], [0], c="white", marker="^", s=65)
    ax.text(0, 0, 0, " Radar", color="white")
    scatter = ax.scatter([], [], [], c=[], cmap="coolwarm", vmin=-2, vmax=2,
                         s=48, depthshade=False)
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.035, pad=0.08, extend="both")
    colorbar.set_label("Radial velocity (m/s)")
    status_text = fig.text(0.04, 0.065, "Waiting for radar frames...", fontsize=10)
    fig.text(0.04, 0.025,
             "Drag: rotate  |  Wheel: zoom  |  R: reset  |  Space: pause  |  Close: stop",
             fontsize=9, color="#aeb8c4")
    ax.set_title("4D radar | live target points")

    latest = queue.Queue(maxsize=1)
    errors = queue.Queue()
    finished = threading.Event()
    try:
        receiver = subprocess.Popen([str(args.receiver.resolve()), "--json"],
                                    stdout=subprocess.PIPE, text=True, bufsize=1)
    except OSError as error:
        plt.close(fig)
        parser.error(str(error))

    def read_frames():
        try:
            for line in receiver.stdout:
                frame = json.loads(line)
                packet = (time.monotonic(), frame)
                try:
                    latest.put_nowait(packet)
                except queue.Full:
                    try:
                        latest.get_nowait()
                    except queue.Empty:
                        pass
                    latest.put_nowait(packet)
        except (ValueError, OSError) as error:
            errors.put(str(error))
        finally:
            finished.set()

    reader = threading.Thread(target=read_frames, name="radar-frame-reader", daemon=True)
    reader.start()
    started = time.monotonic()
    last_received = started
    paused = False
    stream_error = None
    last_label = "Waiting for radar frames..."

    def update(_):
        nonlocal last_received, stream_error, last_label
        now = time.monotonic()
        if args.duration and now - started >= args.duration:
            plt.close(fig)
            return
        try:
            stream_error = errors.get_nowait()
        except queue.Empty:
            pass
        if not paused:
            try:
                last_received, frame = latest.get_nowait()
            except queue.Empty:
                pass
            else:
                points = frame["points"]
                xyz = np.array([[p["x"], p["y"], p["z"]] for p in points], dtype=float).reshape(-1, 3)
                scatter.set_offsets(xyz[:, :2])
                scatter.set_3d_properties(xyz[:, 2], zdir="z")
                scatter.set_array(np.array([p["vel"] for p in points], dtype=float))
                outside = int(np.sum(
                    (xyz[:, 0] < ax.get_xlim()[0]) | (xyz[:, 0] > ax.get_xlim()[1]) |
                    (xyz[:, 1] < ax.get_ylim()[0]) | (xyz[:, 1] > ax.get_ylim()[1]) |
                    (xyz[:, 2] < ax.get_zlim()[0]) | (xyz[:, 2] > ax.get_zlim()[1])))
                last_label = f"Frame {frame['frame']} | Points: {len(points)} | Outside view: {outside}"
        if stream_error:
            status_text.set_text(f"Stream error: {stream_error}")
        elif finished.is_set():
            status_text.set_text(f"Receiver stopped (exit={receiver.poll()}). Close this window to exit.")
        elif paused:
            status_text.set_text(last_label + " | PAUSED")
        elif now - last_received > 1:
            # Clear stale points when frames stop arriving.
            scatter.set_offsets(np.empty((0, 2)))
            scatter.set_3d_properties([], zdir="z")
            scatter.set_array(np.array([]))
            status_text.set_text(f"No new frame for {now - last_received:.1f} s")
        else:
            status_text.set_text(last_label)

    def on_scroll(event):
        if event.inaxes is ax:
            factor = 0.8 if event.button == "up" else 1.25
            for get_limits, set_limits in ((ax.get_xlim, ax.set_xlim),
                                            (ax.get_ylim, ax.set_ylim),
                                            (ax.get_zlim, ax.set_zlim)):
                low, high = get_limits()
                center = (low + high) / 2
                half = (high - low) * factor / 2
                set_limits(center - half, center + half)
            fig.canvas.draw_idle()

    def on_key(event):
        nonlocal paused
        if event.key == "r":
            reset_view()
        elif event.key == " ":
            paused = not paused
        fig.canvas.draw_idle()

    def interrupt(_signal, _frame):
        raise KeyboardInterrupt

    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("key_press_event", on_key)
    signal.signal(signal.SIGTERM, interrupt)
    animation = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        plt.close(fig)
    finally:
        if receiver.poll() is None:
            receiver.terminate()
        try:
            receiver.wait(timeout=3)
        except subprocess.TimeoutExpired:
            receiver.kill()
            receiver.wait()
        reader.join(timeout=2)
        receiver.stdout.close()
        if args.snapshot:
            fig.savefig(args.snapshot, dpi=150, facecolor=fig.get_facecolor())
    # Keep the animation alive throughout the GUI event loop.
    del animation
    return 1 if stream_error or receiver.returncode != 0 else 0


if __name__ == "__main__":
    sys.exit(main())
