import threading
import rospy
import time
from node import CloverAPI
from clover import long_callback
import sensor_msgs
from cv_bridge import CvBridge
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app)
clover = CloverAPI()

H, W = 2000, 2000
MARGIN = 201


def calculate_image_position():
    aruco_map = api.get_map()["markers"]
    left = min(aruco_map, key=lambda x: x["x"] - x["length"] / 2)
    right = max(aruco_map, key=lambda x: x["x"] + x["length"] / 2)
    top = max(aruco_map, key=lambda x: x["y"] + x["length"] / 2)
    bottom = min(aruco_map, key=lambda x: x["y"] - x["length"] / 2)
    dx = right["x"] - left["x"]  # + (right["length"] + left["length"]) / 2
    dy = top["y"] - bottom["y"]  # + (top["length"] + bottom["length"]) / 2
    if dx >= dy:
        k1, k2 = right["length"] / dx, left["length"] / dx
        dx_px = (2 * W - 4 * MARGIN) / (2 + k1 + k2)
        scale = dx / dx_px  # doesn't really work with only one marker present, who cares?
        (dy + (top["length"] + bottom["length"]) / 2) / (dx + (right["length"] + left["length"]) / 2)
        margin_y = (H - (W - 2 * MARGIN) * (dy + (top["length"] + bottom["length"]) / 2) / (
                    dx + (right["length"] + left["length"]) / 2)) / 2
        y = top["y"] + margin_y * scale + top["length"] / 2
        x = left["x"] - MARGIN * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    else:
        k1, k2 = top["length"] / dx, bottom["length"] / dx
        dy_px = (2 * H - 4 * MARGIN) / (2 + k1 + k2)
        scale = dy / dy_px  # doesn't really work with only one marker present, who cares?

        margin_x = (W - (H - 2 * MARGIN) * (dx + (right["length"] + left["length"]) / 2) / (
                    dy + (top["length"] + bottom["length"]) / 2)) / 2
        y = top["y"] + MARGIN * scale + top["length"] / 2
        x = left["x"] - margin_x * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    return x, y, lx, ly


# --- serve frontend ---
@app.route('/')
def index():
    return render_template('index.html')


# Background thread for continuous updates
def map_updater():
    state = clover.get_map()
    while True:
        res = []
        for i in state.markers:
            marker = {
                "id": i.id,
                "length": i.length,
                "x": i.pose.position.x,
                "y": i.pose.position.y,
                "z": i.pose.position.z
            }
            res.append(marker)
        socketio.emit('map', {"markers": res})
        time.sleep(10)  # 5 updates per second


# Start background thread once when server starts
@socketio.on('connect')
def start_updates():
    global updater_thread
    if not hasattr(start_updates, "started"):
        updater_thread = threading.Thread(target=map_updater, daemon=True)
        updater_thread.start()

        battery_thread = threading.Thread(target=battery_updater, daemon=True)
        battery_thread.start()

        coords_thread = threading.Thread(target=get_coords, daemon=True)
        coords_thread.start()

        start_updates.started = True


@socketio.on('get_coords')
def get_coords():
    while True:
        telem = clover.get_telemetry(frame_id='aruco_map')
        emit('coords', {'x': telem.x, 'y': telem.y, 'z': telem.z})
        time.sleep(0.2)


def battery_updater():
    bat = clover.get_battery_state()
    while True:
        socketio.emit('state', {"voltage": bat.voltage, "percentage": bat.percentage})
        time.sleep(1)


if __name__ == '__main__':
    socketio.run(app, host="127.0.0.1", port=4848, allow_unsafe_werkzeug=True)
