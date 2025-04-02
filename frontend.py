import plotly.express as px
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output, no_update, callback, State
import requests as rq
import json
from config import *
import panel_requests as api


H, W = 2000, 2000
MARGIN = 201


def calculate_image_position():
    aruco_map = api.get_map()["markers"]
    left = min(aruco_map, key=lambda x: x["x"] - x["length"] / 2)
    right = max(aruco_map, key=lambda x: x["x"] + x["length"] / 2)
    top = max(aruco_map, key=lambda x: x["y"] + x["length"] / 2)
    bottom = min(aruco_map, key=lambda x: x["y"] - x["length"] / 2)
    dx = right["x"] - left["x"] # + (right["length"] + left["length"]) / 2
    dy = top["y"] - bottom["y"] # + (top["length"] + bottom["length"]) / 2
    if dx >= dy:
        k1, k2 = right["length"] / dx, left["length"] / dx
        dx_px = (2 * W - 4 * MARGIN) / (2 + k1 + k2)
        scale = dx / dx_px  #  doesn't really work with only one marker present, who cares?
        (dy + (top["length"] + bottom["length"]) / 2) / (dx + (right["length"] + left["length"]) / 2)
        margin_y = (H - (W - 2 * MARGIN) * (dy + (top["length"] + bottom["length"]) / 2) / (dx + (right["length"] + left["length"]) / 2)) / 2
        y = top["y"] + margin_y * scale + top["length"] / 2
        x = left["x"] - MARGIN * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    else:
        k1, k2 = top["length"] / dx, bottom["length"] / dx
        dy_px = (2 * H - 4 * MARGIN) / (2 + k1 + k2)
        scale = dy / dy_px  #  doesn't really work with only one marker present, who cares?
        
        margin_x = (W - (H - 2 * MARGIN) * (dx + (right["length"] + left["length"]) / 2) / (dy + (top["length"] + bottom["length"]) / 2)) / 2
        y = top["y"] + MARGIN * scale + top["length"] / 2
        x = left["x"] - margin_x * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    return x, y, lx, ly

data = calculate_image_position()
fig = go.Figure()  # px.imshow(img)
fig.add_layout_image(go.layout.Image(source=f"http://{CLOVER_HOST}:8080/snapshot?topic=/aruco_map/image", xref="x",
            yref="y",
            x=data[0],
            y=data[1],
            sizex=data[2],
            sizey=data[3],
            sizing="stretch",
            layer="below"))
fig.update_layout(xaxis={
    'range': [data[0], data[0] + data[2]],
    'showgrid': True,
    'zeroline': False,
    'visible': True
}, yaxis={
    'range': [data[1] - data[3], data[1]],
    'showgrid': True,
    'zeroline': False,
    'visible': True,
    'scaleanchor':'x',
    'scaleratio': 1
})
fig.add_scatter(mode="markers", marker_color="green", marker_size=10)
config = {
    "modeBarButtonsToAdd": [
        "drawline",
        "drawopenpath",
        "drawclosedpath",
        "drawcircle",
        "drawrect",
        "eraseshape",

    ],
    'displaylogo': False,
}
fig["layout"]["shapes"] = []
app = Dash()
app.layout = html.Div(
    [
        html.Button('home', id='home', n_clicks=0),
        html.Button('land', id='land', n_clicks=0, disabled=False),
        html.Button('stop', id='stop', n_clicks=0),
        html.Button('start', id='launch_sequence', n_clicks=0),
        html.Button('kill switch', id='kill_switch', n_clicks=0),
        dcc.Graph(id="graph-pic", figure=fig, config=config, style={'width': '80%', 'height': '100vh'}),
        dcc.Markdown("Characteristics of shapes"),
        html.Pre(id="annotations-data-pre"),
        html.Pre(id="state"),
        dcc.Interval(id='interval1', interval=2 * 1000, n_intervals=0),
        html.Div(id="dummy1"), html.Div(id="dummy2"), html.Div(id="dummy3"), html.Div(id="dummy4"), html.Div(id="dummy5")
    ]
)


@app.callback(
    Output("annotations-data-pre", "children", allow_duplicate=True),
    Input("graph-pic", "relayoutData"),
    prevent_initial_call=True,
)
def on_new_annotation(relayout_data):
    if "shapes" in relayout_data:
        return json.dumps(relayout_data["shapes"], indent=2)
    else:
        return no_update
    
@app.callback(
    Output('graph-pic', 'figure'),
    State('graph-pic', 'figure'),
    Input('interval1', 'n_intervals')
)
def update_graph(graph_figure, clickData):
    points = api.get_position()
    markers = api.get_markers()
    
    x = points["x"]
    y = points["y"]
    graph_figure['data'][0].update(x=[x])
    graph_figure['data'][0].update(y=[y])
    for marker in markers["markers"]:
        graph_figure['data'].append({"marker": {"color": marker["color"], "size": marker["size"], "symbol": marker["type"]}, "mode": "markers", "type": "scatter", "x": [marker["x"]], "y": [marker["y"]]})
    # graph_figure["layout"].update(shapes=markers["markers"])
    return graph_figure

@app.callback(
    Output('state', 'children'),
    Input('interval1', 'n_intervals')
)
def update_state(clickData):
    state = rq.get(f"http://{HOST}:{PORT}/state").json()
    return f"Battery voltage: {state['voltage']}\nCharge: {state['percentage'] * 100}%"

@app.callback(
    Output("land", "disabled", allow_duplicate=True),
    Input("stop", "n_clicks"),
    prevent_initial_call=True,
)
def stop(relayout_data):
    # rq.post(f"{HOST}:{PORT}/targets")
    rq.get(f"http://{HOST}:{PORT}/stop")
    return False

@app.callback(
    Output("land", "disabled", allow_duplicate=True),
    Input("launch_sequence", "n_clicks"),
    prevent_initial_call=True,
)
def on_launch_sequence(relayout_data):
    rq.get(f"http://{HOST}:{PORT}/launch_sequence")
    return True

@app.callback(
    Output("dummy2", "children", allow_duplicate=True),
    Input("land", "n_clicks"),
    prevent_initial_call=True,
)
def on_land_sequence(relayout_data):
    rq.get(f"http://{HOST}:{PORT}/land")
    return True

@app.callback(
    Output("dummy4", "children", allow_duplicate=True),
    Input("kill_switch", "n_clicks"),
    prevent_initial_call=True,
)
def on_kill_switch(relayout_data):
    rq.get(f"http://{HOST}:{PORT}/stop")
    rq.get(f"http://{HOST}:{PORT}/disarm")
    # rq.get(f"http://{HOST}:{PORT}/land")
    return True

@app.callback(
    Output("dummy5", "children", allow_duplicate=True),
    Input("home", "n_clicks"),
    prevent_initial_call=True,
)
def go_home(relayout_data):
    rq.get(f"http://{HOST}:{PORT}/home")
    # rq.get(f"http://{HOST}:{PORT}/land")
    return True

@app.callback(
    Output("graph-pic", "relayoutData"),
    # State("graph-pic", "relayoutData"),
    Input("interval1", "n_intervals"),
    # prevent_initial_call=True,
)
def pipelines(relayoutData):
    relayoutData = dict()
    data = api.draw_pipeline()
    path = "M"
    for i in data["nodes"]:
        path = path + f"{i['x']},{i['y']}L"
    path = path[:-1]
    template[0]["path"] = path
    relayoutData.update(shapes=template)
    return relayoutData
template = [
  {
    "editable": True,
    "visible": True,
    "showlegend": False,
    "legend": "legend",
    "legendgroup": "",
    "legendgrouptitle": {
      "text": "",
      "font": {
        "weight": "normal",
        "style": "normal",
        "variant": "normal",
        "lineposition": "none",
        "textcase": "normal",
        "shadow": "none"
      }
    },
    "legendrank": 1000,
    "label": {
      "text": "",
      "texttemplate": ""
    },
    "xref": "x",
    "yref": "y",
    "layer": "above",
    "opacity": 1,
    "line": {
      "color": "#444",
      "width": 4,
      "dash": "solid"
    },
    "type": "path",
    "path": "M0.950640338669658,1.7590923637245515L0.8410918948439721,3.621415908761214L1.060188782495344,5.921933229100621L1.279285670146716,6.250578560577679L2.2104474426650467,6.962643445444639L3.9084483219631796,7.839030996050127L4.127545209614552,7.893805217962971"
  }
]
    

if __name__ == "__main__":
    app.run(port=3456, debug=True)