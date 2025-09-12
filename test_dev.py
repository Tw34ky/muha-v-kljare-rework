import threading
import time
from node import CloverAPI
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app)
clover = CloverAPI()
# @app.before_request
# def get_clover_instance():
#     global clover
#     clover = CloverAPI()

# --- serve frontend ---
@app.route('/')
def index():
    return render_template('index.html')

# Background thread for continuous updates
def map_updater():
    # class Data:
    #     def __init__(self, id, length, pose):
    #         self.id = id,
    #         self.length = length,
    #         self.pose = pose
    #         self.x = pose[0]
    #         self.y = pose[1]
    #         self.z = pose[2]

    # class Zaglushka:
    #     def __init__(self, markers):
    #         self.markers = markers
    # data_pass = [Data(0, 1, (0, 0, 0)), Data(1, 1, (1, 1, 1)), Data(2, 1, (3, 3, 3))]
    # state = Zaglushka(data_pass)
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
        time.sleep(0.2)  # 5 updates per second

# Start background thread once when server starts
@socketio.on('connect')
def start_updates():
    global updater_thread
    if not hasattr(start_updates, "started"):
        updater_thread = threading.Thread(target=map_updater, daemon=True)
        battery_thread = threading.Thread(target=battery_updater, daemon=True)
        battery_thread.start()
        updater_thread.start()
        start_updates.started = True


def battery_updater():
    class Battery:
        def __init__(self, vol, per):
            self.voltage = vol
            self.percentage = per
 
    bat = clover.get_battery_state()
    # bat = Battery(3, 50)
    # mavros = clover.get_state()
    while True:
        socketio.emit('state', {"voltage": bat.voltage, "percentage": bat.percentage})
        time.sleep(1)


if __name__ == '__main__':
    socketio.run(app, host="127.0.0.1", port=4848, allow_unsafe_werkzeug=True)
