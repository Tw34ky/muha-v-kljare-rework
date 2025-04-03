import requests as rq
from plotly.validators.scatter.marker import SymbolValidator
from config import *


def add_marker(x: float, y: float, color: str, size: int, type: str) -> dict:
    f'''Possible_types: circle, circle-open, circle-dot, circle-open-dot, square, square-open, square-dot, square-open-dot, diamond, diamond-open, diamond-dot, diamond-open-dot, cross, cross-open, cross-dot, cross-open-dot, x, x-open, x-dot, x-open-dot, triangle-up, triangle-up-open, triangle-up-dot, triangle-up-open-dot, triangle-down, triangle-down-open, triangle-down-dot, triangle-down-open-dot, triangle-left, triangle-left-open, triangle-left-dot, triangle-left-open-dot, triangle-right, triangle-right-open, triangle-right-dot, triangle-right-open-dot, triangle-ne, triangle-ne-open, triangle-ne-dot, triangle-ne-open-dot, triangle-se, triangle-se-open, triangle-se-dot, triangle-se-open-dot, triangle-sw, triangle-sw-open, triangle-sw-dot, triangle-sw-open-dot, triangle-nw, triangle-nw-open, triangle-nw-dot, triangle-nw-open-dot, pentagon, pentagon-open, pentagon-dot, pentagon-open-dot, hexagon, hexagon-open, hexagon-dot, hexagon-open-dot, hexagon2, hexagon2-open, hexagon2-dot, hexagon2-open-dot, octagon, octagon-open, octagon-dot, octagon-open-dot, star, star-open, star-dot, star-open-dot, hexagram, hexagram-open, hexagram-dot, hexagram-open-dot, star-triangle-up, star-triangle-up-open, star-triangle-up-dot, star-triangle-up-open-dot, star-triangle-down, star-triangle-down-open, star-triangle-down-dot, star-triangle-down-open-dot, star-square, star-square-open, star-square-dot, star-square-open-dot, star-diamond, star-diamond-open, star-diamond-dot, star-diamond-open-dot, diamond-tall, diamond-tall-open, diamond-tall-dot, diamond-tall-open-dot, diamond-wide, diamond-wide-open, diamond-wide-dot, diamond-wide-open-dot, hourglass, hourglass-open, bowtie, bowtie-open, circle-cross, circle-cross-open, circle-x, circle-x-open, square-cross, square-cross-open, square-x, square-x-open, diamond-cross, diamond-cross-open, diamond-x, diamond-x-open, cross-thin, cross-thin-open, x-thin, x-thin-open, asterisk, asterisk-open, hash, hash-open, hash-dot, hash-open-dot, y-up, y-up-open, y-down, y-down-open, y-left, y-left-open, y-right, y-right-open, line-ew, line-ew-open, line-ns, line-ns-open, line-ne, line-ne-open, line-nw, line-nw-open, arrow-up, arrow-up-open, arrow-down, arrow-down-open, arrow-left, arrow-left-open, arrow-right, arrow-right-open, arrow-bar-up, arrow-bar-up-open, arrow-bar-down, arrow-bar-down-open, arrow-bar-left, arrow-bar-left-open, arrow-bar-right, arrow-bar-right-open, arrow, arrow-open, arrow-wide, arrow-wide-open'''
    return rq.post(f"http://{HOST}:{PORT}/markers", json={"type": "square", "color": "red", "x": 5, "y": 5, "size":40}).json()

def get_markers() -> dict:
    return rq.get(f"http://{HOST}:{PORT}/markers").json()

def add_target(x: float, y: float, z: float, frame_id: str = "aruco_map", velocity: float = 0.3) -> dict:
    return rq.post(f"http://{HOST}:{PORT}/targets", json={"x": x, "y": y, "z": z, "frame_id": frame_id, "velocity": velocity}).json()

def get_targets():
    return rq.get(f"http://{HOST}:{PORT}/targets").json()

def get_map() -> dict:
    return rq.get(f"http://{HOST}:{PORT}/map").json()

def get_position() -> dict:
    return rq.get(f"http://{HOST}:{PORT}/coords").json()

def draw_pipeline() -> dict:
    # data = rq.get("some_shit").json()
    data = {"nodes": [{"x": 1.0, "y": 2.0}, {"x": 5.0, "y": 4.0}]}
    return data
    