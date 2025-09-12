from flask import Flask
from flask_socketio import SocketIO, emit
import subprocess
from node import CloverAPI
from config import *

app = Flask("remote_panel")
socketio = SocketIO(app, cors_allowed_origins="*")

queue = []
markers_db = []
pipeline_db = dict()

# Initialize CloverAPI instance once
clover = CloverAPI()

# --- Socket Handlers ---

@socketio.on('get_coords')
def get_coords():
    telem = clover.get_telemetry(frame_id='aruco_map')
    emit('coords', {'x': telem.x, 'y': telem.y, 'z': telem.z})


@socketio.on('targets')
def targets(data=None):
    if data:  # Equivalent to POST
        queue.insert(0, data)
        emit('targets_ack', {"status": "added"})
    else:  # Equivalent to GET
        emit('targets', {"targets": queue})


@socketio.on('stop')
def stop():
    global proc
    try:
        proc.terminate()
    except NameError:
        pass
    clover.navigate(x=0, y=0, z=0, frame_id='body')
    emit('stopped', {})


@app.route('disarm')
def disarm():
    clover.arming(False)
    emit('disarmed', {})


@socketio.on('get_map')
def get_map():
    state = clover.get_map()
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
    emit('map', {"markers": res})


@socketio.on('launch_sequence')
def launch_sequence():
    global proc
    proc = subprocess.Popen(
        ["/bin/python3", "/home/pi/square.py"], shell=False
    )
    emit('launched', {})


@socketio.on('markers')
def markers(data=None):
    if data:
        markers_db.append(data)
        emit('markers_ack', {"status": "added"})
    else:
        emit('markers', {"markers": markers_db})


@socketio.on('get_state')
def get_state():
    bat = clover.get_battery_state()
    mavros = clover.get_state()
    emit('state', {"voltage": bat.voltage, "percentage": bat.percentage})


@socketio.on('land')
def land():
    clover.land()
    emit('landed', {})


@socketio.on('home')
def home():
    subprocess.Popen(["/bin/python3", "/home/pi/land.py"], shell=False)
    emit('homed', {})


@socketio.on('pipeline')
def pipeline(data=None):
    global pipeline_db
    if data:
        pipeline_db = data
        emit('pipeline_ack', {"status": "updated"})
    else:
        emit('pipeline', pipeline_db)


if __name__ == '__main__':
    socketio.run(app, host=HOST, port=PORT, debug=True)
