from flask import Flask, jsonify
import flask
import subprocess
from node import CloverAPI
from config import *

app = Flask("remote_panel")

queue = []
markers_db = []
pipeline_db = dict()
@app.before_request
def get_clover_instance():
    global clover
    clover = CloverAPI()

@app.route('/coords', methods=['GET'])
def get_coords():
    telem = clover.get_telemetry(frame_id='aruco_map')
    return jsonify({'x': telem.x, 'y': telem.y, 'z': telem.z})

@app.route('/targets', methods=['POST', 'GET'])
def targets():
    payload = flask.request
    print(queue)
    if payload.method == 'POST':
        queue.insert(0, payload.get_json())
    else:
        return jsonify({"targets": queue})
    # TODO: validation here
    return jsonify({})

@app.route('/stop', methods=['GET'])
def stop():
    try:
        proc.terminate()
    except NameError:
        pass
    clover.navigate(x=0, y=0, z=0, frame_id='body')
    # TODO: disarm
    return jsonify({})

@app.route('/disarm', methods=['GET'])
def disarm():
    clover.arming(False)
    return jsonify({})

@app.route('/map', methods=['GET'])
def get_map():
    print("map")
    state = clover.get_map()
    res = []
    for i in state.markers:
        marker = {"id": i.id, "length": i.length, "x": i.pose.position.x, "y": i.pose.position.y, "z": i.pose.position.z}
        res.append(marker)
    return jsonify({"markers": res})

@app.route('/launch_sequence', methods=['GET'])
def launch_sequence():
    global proc
    proc = subprocess.Popen(["/bin/python3", "/home/pi/square.py"], shell=False)
    print("launched")
    return jsonify({})

@app.route('/markers', methods=['POST', 'GET'])
def markers():
    payload = flask.request
    if payload.method == 'POST':
        markers_db.append(payload.get_json())
    else:
        return jsonify({"markers": markers_db})
    # TODO: validation here
    return jsonify({})

@app.route('/state', methods=['GET'])
def get_state():
    bat = clover.get_battery_state()
    mavros = clover.get_state()
    return jsonify({"voltage": bat.voltage, "percentage": bat.percentage})

@app.route('/land', methods=['GET'])
def land():
    # subprocess.Popen(["/bin/python3", "/home/clover/Desktop/land.py"], shell=False)
    clover.land()
    return jsonify({})

@app.route('/home', methods=['GET'])
def home():
    subprocess.Popen(["/bin/python3", "/home/pi/land.py"], shell=False)
    # clover.land()
    return jsonify({})

@app.route('/pipeline', methods=['POST', 'GET'])
def pipeline():
    payload = flask.request
    if payload.method == 'POST':
        pipeline_db = payload.get_json()
    else:
        return jsonify(pipeline_db)
    # TODO: validation here
    return jsonify({})

if __name__ == '__main__':
    app.run(HOST, PORT, debug=True)
