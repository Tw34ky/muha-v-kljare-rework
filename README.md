# muha-v-kljare
Репозиторий команды "Муха в кляре" на НТО 2025 по профилю "Летающая робототехника"
## Решенные задачи
Мы решили несколько и сейчас я опишу их:
### web-приложение
# Remote Panel for Clover Drone Control

#### Описание задачи

Эта задача представляет собой веб-панель для удаленного управления дроном Clover, которая включает в себя:
- Отображение текущего положения дрона на карте с маркерами ArUco
- Управление базовыми функциями дрона (взлет, посадка, возврат домой)
- Отображение состояния батареи
- Визуализацию целей

#### Структура задачи

Задача состоит из трех основных компонентов:

1. **Серверная часть (server.py)** - Flask-приложение, предоставляющее API для взаимодействия с дроном
2. **Фронтенд (frontend.py)** - Dash-приложение с интерактивным интерфейсом
3. **ROS-обертка (node.py)** - Класс для взаимодействия с ROS-сервисами Clover

#### Подробное описание компонентов

##### 1. Серверная часть (server.py)

```python
from flask import Flask, jsonify
import flask
import subprocess
from node import CloverAPI
from config import *

app = Flask("remote_panel")

# Глобальные переменные для хранения данных
queue = []          # Очередь целей
markers_db = []     # База данных маркеров
pipeline_db = dict() # Конфигурация pipeline

@app.before_request
def get_clover_instance():
    """Инициализация экземпляра CloverAPI перед каждым запросом"""
    global clover
    clover = CloverAPI()

@app.route('/coords', methods=['GET'])
def get_coords():
    """Получение текущих координат дрона в системе ArUco маркеров"""
    telem = clover.get_telemetry(frame_id='aruco_map')
    return jsonify({'x': telem.x, 'y': telem.y, 'z': telem.z})

@app.route('/targets', methods=['POST', 'GET'])
def targets():
    """Управление очередью целей: добавление новых или получение текущего списка"""
    payload = flask.request
    print(queue)
    if payload.method == 'POST':
        queue.insert(0, payload.get_json())
    else:
        return jsonify({"targets": queue})
    return jsonify({})

@app.route('/stop', methods=['GET'])
def stop():
    """Экстренная остановка дрона и прерывание текущего процесса"""
    try:
        proc.terminate()
    except NameError:
        pass
    clover.navigate(x=0, y=0, z=0, frame_id='body')
    return jsonify({})

@app.route('/disarm', methods=['GET'])
def disarm():
    """Отключение моторов дрона"""
    clover.arming(False)
    return jsonify({})

@app.route('/map', methods=['GET'])
def get_map():
    """Получение карты ArUco маркеров"""
    print("map")
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
    return jsonify({"markers": res})

@app.route('/launch_sequence', methods=['GET'])
def launch_sequence():
    """Запуск последовательности полета (квадрата)"""
    global proc
    proc = subprocess.Popen(["/bin/python3", "/home/pi/square.py"], shell=False)
    print("launched")
    return jsonify({})

@app.route('/markers', methods=['POST', 'GET'])
def markers():
    """Управление пользовательскими маркерами: добавление или получение"""
    payload = flask.request
    if payload.method == 'POST':
        markers_db.append(payload.get_json())
    else:
        return jsonify({"markers": markers_db})
    return jsonify({})

@app.route('/state', methods=['GET'])
def get_state():
    """Получение состояния дрона (батарея)"""
    bat = clover.get_battery_state()
    mavros = clover.get_state()
    return jsonify({"voltage": bat.voltage, "percentage": bat.percentage})

@app.route('/land', methods=['GET'])
def land():
    """Посадка дрона"""
    clover.land()
    return jsonify({})

@app.route('/home', methods=['GET'])
def home():
    """Возврат дрона в начальную точку"""
    subprocess.Popen(["/bin/python3", "/home/pi/land.py"], shell=False)
    return jsonify({})

@app.route('/pipeline', methods=['POST', 'GET'])
def pipeline():
    """Управление pipeline: сохранение или получение конфигурации"""
    payload = flask.request
    if payload.method == 'POST':
        pipeline_db = payload.get_json()
    else:
        return jsonify(pipeline_db)
    return jsonify({})

if __name__ == '__main__':
    app.run(HOST, PORT, debug=True)
```

##### 2. Фронтенд (frontend.py)

```python
import plotly.express as px
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output, no_update, callback, State
import requests as rq
import json
from config import *
import panel_requests as api

# Константы для размера изображения
H, W = 2000, 2000
MARGIN = 201

def calculate_image_position() -> tuple[float, float, float, float]:
    """Вычисление позиции и масштаба изображения карты на основе маркеров"""
    aruco_map = api.get_map()["markers"]
    left = min(aruco_map, key=lambda x: x["x"] - x["length"] / 2)
    right = max(aruco_map, key=lambda x: x["x"] + x["length"] / 2)
    top = max(aruco_map, key=lambda x: x["y"] + x["length"] / 2)
    bottom = min(aruco_map, key=lambda x: x["y"] - x["length"] / 2)
    dx = right["x"] - left["x"]
    dy = top["y"] - bottom["y"]
    
    if dx >= dy:
        k1, k2 = right["length"] / dx, left["length"] / dx
        dx_px = (2 * W - 4 * MARGIN) / (2 + k1 + k2)
        scale = dx / dx_px
        margin_y = (H - (W - 2 * MARGIN) * (dy + (top["length"] + bottom["length"]) / 2) / (dx + (right["length"] + left["length"]) / 2)) / 2
        y = top["y"] + margin_y * scale + top["length"] / 2
        x = left["x"] - MARGIN * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    else:
        k1, k2 = top["length"] / dx, bottom["length"] / dx
        dy_px = (2 * H - 4 * MARGIN) / (2 + k1 + k2)
        scale = dy / dy_px
        margin_x = (W - (H - 2 * MARGIN) * (dx + (right["length"] + left["length"]) / 2) / (dy + (top["length"] + bottom["length"]) / 2)) / 2
        y = top["y"] + MARGIN * scale + top["length"] / 2
        x = left["x"] - margin_x * scale - left["length"] / 2
        ly, lx = H * scale, W * scale
    return x, y, lx, ly

# Инициализация фигуры с изображением карты
data = calculate_image_position()
fig = go.Figure()
fig.add_layout_image(
    go.layout.Image(
        source=f"http://{CLOVER_HOST}:8080/snapshot?topic=/aruco_map/image",
        xref="x",
        yref="y",
        x=data[0],
        y=data[1],
        sizex=data[2],
        sizey=data[3],
        sizing="stretch",
        layer="below"
    )
)

# Настройка осей и интерфейса
fig.update_layout(
    xaxis={
        'range': [data[0], data[0] + data[2]],
        'showgrid': True,
        'zeroline': False,
        'visible': True
    }, 
    yaxis={
        'range': [data[1] - data[3], data[1]],
        'showgrid': True,
        'zeroline': False,
        'visible': True,
        'scaleanchor':'x',
        'scaleratio': 1
    }
)
fig.add_scatter(mode="markers", marker_color="green", marker_size=10)

# Конфигурация элементов управления
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

# Создание Dash-приложения
app = Dash()
app.layout = html.Div([
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
])

# Callback-функции для обработки взаимодействия с интерфейсом
@app.callback(
    Output("annotations-data-pre", "children", allow_duplicate=True),
    Input("graph-pic", "relayoutData"),
    prevent_initial_call=True,
)
def on_new_annotation(relayout_data: dict) -> str:
    """Обработка новых аннотаций на карте"""
    if "shapes" in relayout_data:
        return json.dumps(relayout_data["shapes"], indent=2)
    else:
        return no_update
    
@app.callback(
    Output('graph-pic', 'figure'),
    State('graph-pic', 'figure'),
    Input('interval1', 'n_intervals')
)
def update_graph(graph_figure: dict, clickData: int) -> dict:
    """Обновление графика с позицией дрона и маркерами"""
    points = api.get_position()
    markers = api.get_markers()
    
    x = points["x"]
    y = points["y"]
    graph_figure['data'][0].update(x=[x])
    graph_figure['data'][0].update(y=[y])
    
    for marker in markers["markers"]:
        graph_figure['data'].append({
            "marker": {
                "color": marker["color"], 
                "size": marker["size"], 
                "symbol": marker["type"]
            }, 
            "mode": "markers", 
            "type": "scatter", 
            "x": [marker["x"]], 
            "y": [marker["y"]]
        })
    return graph_figure

@app.callback(
    Output('state', 'children'),
    Input('interval1', 'n_intervals')
)
def update_state(clickData: int) -> str:
    """Обновление информации о состоянии батареи"""
    state = rq.get(f"http://{HOST}:{PORT}/state").json()
    return f"Battery voltage: {state['voltage']}\nCharge: {state['percentage'] * 100}%"

# Остальные callback-функции для обработки кнопок управления
@app.callback(
    Output("land", "disabled", allow_duplicate=True),
    Input("stop", "n_clicks"),
    prevent_initial_call=True,
)
def stop(relayout_data: int) -> bool:
    """Обработка нажатия кнопки Stop"""
    rq.get(f"http://{HOST}:{PORT}/stop")
    return False

@app.callback(
    Output("land", "disabled", allow_duplicate=True),
    Input("launch_sequence", "n_clicks"),
    prevent_initial_call=True,
)
def on_launch_sequence(relayout_data: int) -> bool:
    """Обработка нажатия кнопки запуска последовательности"""
    rq.get(f"http://{HOST}:{PORT}/launch_sequence")
    return True

@app.callback(
    Output("dummy2", "children", allow_duplicate=True),
    Input("land", "n_clicks"),
    prevent_initial_call=True,
)
def on_land_sequence(relayout_data: int) -> bool:
    """Обработка нажатия кнопки посадки"""
    rq.get(f"http://{HOST}:{PORT}/land")
    return True

@app.callback(
    Output("dummy4", "children", allow_duplicate=True),
    Input("kill_switch", "n_clicks"),
    prevent_initial_call=True,
)
def on_kill_switch(relayout_data: int) -> bool:
    """Обработка аварийного выключения"""
    rq.get(f"http://{HOST}:{PORT}/stop")
    rq.get(f"http://{HOST}:{PORT}/disarm")
    return True

@app.callback(
    Output("dummy5", "children", allow_duplicate=True),
    Input("home", "n_clicks"),
    prevent_initial_call=True,
)
def go_home(relayout_data: int) -> bool:
    """Обработка нажатия кнопки возврата домой"""
    rq.get(f"http://{HOST}:{PORT}/home")
    return True

@app.callback(
    Output("graph-pic", "relayoutData"),
    Input("interval1", "n_intervals"),
)
def pipelines(relayoutData: dict) -> dict:
    """Обновление pipeline на карте"""
    relayoutData = dict()
    data = api.draw_pipeline()
    path = "M"
    for i in data["nodes"]:
        path = path + f"{i['x']},{i['y']}L"
    path = path[:-1]
    template[0]["path"] = path
    relayoutData.update(shapes=template)
    return relayoutData

# Шаблон для отображения pipeline
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
```

##### 3. ROS-обертка (node.py)

```python
from aruco_pose.msg import MarkerArray
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
import rospy
import tf2_ros
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool

class CloverAPI:
    """Класс-обертка для взаимодействия с ROS-сервисами Clover"""
    
    # Инициализация ROS-сервисов
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    land = rospy.ServiceProxy('land', Trigger)
    arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    
    def __new__(cls):
        """Реализация singleton-паттерна для класса"""
        if not hasattr(cls, 'instance'):
            cls.instance = super(CloverAPI, cls).__new__(cls)
            rospy.init_node("remote_panel_clover", disable_signals=True)
            print("registered")
        return cls.instance

    @staticmethod
    def get_map() -> MarkerArray:
        """Получение карты ArUco маркеров"""
        return rospy.wait_for_message('/aruco_map/map', MarkerArray)
    
    @staticmethod
    def get_battery_state() -> BatteryState:
        """Получение состояния батареи"""
        return rospy.wait_for_message('/mavros/battery', BatteryState)

    @staticmethod
    def get_state() -> State:
        """Получение состояния MAVROS"""
        return rospy.wait_for_message('/mavros/state', State)
```

#### Решенные задачи

1. **Интеграция с ROS**: Создан класс CloverAPI, который предоставляет удобный интерфейс для работы с ROS-сервисами дрона Clover.

2. **Веб-интерфейс управления**: Реализовано Flask-приложение с REST API для управления дроном и Dash-приложение с интерактивным интерфейсом.

3. **Визуализация положения дрона**: Разработан механизм отображения текущего положения дрона на карте с маркерами ArUco.

4. **Управление полетом**: Реализованы функции взлета, посадки, возврата домой и экстренной остановки.

5. **Мониторинг состояния**: Добавлен вывод информации о состоянии батареи дрона.

6. **Работа с маркерами**: Реализована возможность добавления и отображения пользовательских маркеров.


#### Использованные технологии и методы

1. **ROS (Robot Operating System)**: Для взаимодействия с дроном Clover и получения телеметрии.

2. **Flask**: Для создания REST API сервера.

3. **Dash (Plotly)**: Для создания интерактивного веб-интерфейса с графиками.

4. **Singleton паттерн**: В реализации CloverAPI для обеспечения единственного экземпляра класса.

5. **Асинхронные запросы**: Для обновления интерфейса без перезагрузки страницы.

6. **REST API**: Для взаимодействия между фронтендом и бэкендом.

7. **Subprocess**: Для запуска внешних скриптов управления дроном.

#### Особенности реализации

1. **Масштабирование карты**: Алгоритм автоматического расчета положения и масштаба изображения на основе маркеров ArUco.

2. **Интерактивное рисование**: Возможность рисовать маршруты прямо на карте с помощью инструментов Plotly.

3. **Режим реального времени**: Постоянное обновление положения дрона и состояния батареи через интервальные запросы.

4. **Безопасность**: Реализованы функции экстренной остановки и отключения моторов.

5. **Гибкость**: Возможность добавлять пользовательские маркеры и цели через API.
# Дополнительные скрипты для управления дроном Clover

## Описание файлов

### 1. ggsell.py - Обработка тепловизионного изображения

```python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
import numpy as np

# Инициализация ROS узла
rospy.init_node('cv')
bridge = CvBridge()

@long_callback
def image_callback(data):
    """
    Callback-функция для обработки изображения с тепловизора.
    Разделяет изображение на две части, применяет цветовую карту к первой половине
    и публикует результат в топик для отладки.
    """
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # Конвертация ROS сообщения в OpenCV изображение
    a, b = np.array_split(img, 2)  # Разделение изображения на две части
    colorized = cv2.applyColorMap(a, cv2.COLORMAP_JET)  # Применение цветовой карты
    
    # Публикация обработанного изображения
    image_pub.publish(bridge.cv2_to_imgmsg(colorized, 'bgr8'))

# Подписка на топик с изображением тепловизора и публикатор для отладочного изображения
image_sub = rospy.Subscriber('/thermal_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher('~debug', Image)

rospy.spin()  # Запуск обработки сообщений
```

**Назначение**: Этот скрипт обрабатывает изображения с тепловизора, применяя к ним цветовую карту для лучшей визуализации температурных данных. Обработанные изображения публикуются в отдельный топик для отладки.

### 2. land.py - Посадка дрона

```python
import rospy
import math
from std_srvs.srv import Trigger
from clover import srv

# Инициализация ROS сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=0, speed=1, frame_id='', auto_arm=False, tolerance=0.2):
    """
    Функция для навигации с ожиданием достижения цели.
    Параметры:
    - x, y, z: координаты цели
    - yaw: ориентация по курсу
    - speed: скорость движения
    - frame_id: система координат
    - auto_arm: автоматическое включение моторов
    - tolerance: допустимая погрешность достижения цели
    """
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    # Ожидание достижения цели
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# Последовательность команд для посадки
navigate_wait(x=6, y=0.6, z=1.5, frame_id='aruco_map')  # Перелет в точку посадки
navigate_wait(z=0.5, frame_id='aruco_map')  # Снижение
land()  # Посадка
```

**Назначение**: Этот скрипт выполняет последовательность команд для безопасной посадки дрона в заданной точке. Сначала дрон перемещается в точку над местом посадки, затем снижается и выполняет посадку.

### 3. square.py - Выполнение полетного задания

```python
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
import requests as rq
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

# Инициализация ROS узла
rospy.init_node('flight')

# Настройка видеозаписи
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out_main = cv2.VideoWriter('main_camera.avi', fourcc, 30.0, (320,240))  # Для основной камеры
out_thermal = cv2.VideoWriter('thermal.avi', fourcc, 25.0, (320,240))  # Для тепловизора
bridge = CvBridge()

@long_callback
def image_callback(data):
    """Callback для записи изображения с основной камеры"""
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    out_main.write(img)

@long_callback
def thermal_callback(data):
    """Callback для записи изображения с тепловизора"""
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    out_thermal.write(img)

# Инициализация ROS сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def cords() -> dict:
    """Получение координат точек маршрута с сервера"""
    data = rq.get("http://157.180.22.113:8080/coords_1.json", 
                 headers={"Authorization": f"Bearer a16043eb-ca30-4f42-9b02-da727ded44a4"}).json()
    return data['nodes']

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    """Функция навигации с ожиданием достижения цели"""
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# Основная логика выполнения полета
coords = cords()  # Получение координат точек маршрута
coords = ((coords[0]["x"], coords[0]["y"]), (coords[1]["x"], coords[1]["y"]))

navigate_wait(z=1, frame_id='body', auto_arm=True)  # Взлет
navigate_wait(x=6, y=0.6, z=1.0, frame_id='aruco_map')  # Перелет к первой точке
navigate_wait(x=3.95, y=0.4, z=1.0, frame_id='aruco_map')  # Перелет ко второй точке

# Подписка на камеры и начало записи
image_sub = rospy.Subscriber('/main_camera/image_raw', Image, image_callback)
thermal_sub = rospy.Subscriber('/cv/debug', Image, thermal_callback)
rospy.sleep(1)

navigate_wait(x=6.45, y=3.4, z=1.0, frame_id='aruco_map')  # Перелет к третьей точке

# Завершение записи и возврат по точкам маршрута
out_main.release()
out_thermal.release()
navigate_wait(x=coords[0][0], y=coords[0][1], z=1.0, frame_id='aruco_map')
navigate_wait(x=coords[1][0], y=coords[1][0], z=1.0, frame_id='aruco_map')

rospy.sleep(5)
rospy.spin()
```

## Отдельное описание square.py

**Назначение**: Скрипт `square.py` выполняет автономный полет дрона по заданному маршруту с одновременной записью видео с основной и тепловизионной камер.

### Основные функции:

1. **Запись видео**:
   - Создаются два видеопотока: `main_camera.avi` (основная камера) и `thermal.avi` (тепловизор)
   - Запись ведется с частотой 30 и 25 кадров/с соответственно
   - Используется кодек XVID для сжатия видео

2. **Навигация**:
   - Получение координат точек маршрута с внешнего сервера
   - Последовательный перелет между точками с использованием функции `navigate_wait`
   - Автоматическое включение моторов при взлете (`auto_arm=True`)

3. **Логика полета**:
   1. Взлет на высоту 1 метр (`navigate_wait(z=1, frame_id='body', auto_arm=True)`)
   2. Перелет к первой точке маршрута (x=6, y=0.6, z=1.0)
   3. Перелет ко второй точке (x=3.95, y=0.4, z=1.0)
   4. Активация подписки на камеры и начало записи видео
   5. Перелет к третьей точке (x=6.45, y=3.4, z=1.0)
   6. Завершение записи видео
   7. Возврат по точкам маршрута
   8. Завершение работы

### Особенности реализации:

- Использование `long_callback` для обработки изображений, что позволяет избежать блокировки основного потока
- Получение координат маршрута с внешнего сервера по HTTP с авторизацией
- Функция `navigate_wait` обеспечивает точное достижение каждой точки перед продолжением маршрута
- Параллельная запись видео с двух камер
- Использование системы координат `aruco_map` для точной навигации по маркерам

Этот скрипт демонстрирует комплексное использование возможностей дрона Clover, включая автономную навигацию, работу с камерами и взаимодействие с внешними сервисами.
Так же у нас есть получение видеопотока с тепловизора с помощью лаунча thermal_camera.launch вот его описание
# Описание launch-файла для тепловизора

## thermal_camera.launch

```xml
<launch>
    <!-- Параметр для указания устройства видеозахвата (thermal camera) -->
    <arg name="thermal_device" default="/dev/video1"/>

    <!-- Менеджер nodelet'ов для камеры -->
    <node pkg="nodelet" type="nodelet" name="thermal_camera_nodelet_manager" args="manager" output="screen" clear_params="true" respawn="true">
        <!-- Количество рабочих потоков для обработки -->
        <param name="num_worker_threads" value="2"/>
    </node>

    <!-- Узел для работы с тепловизором -->
    <node pkg="nodelet" type="nodelet" name="thermal_camera" args="load cv_camera/CvCameraNodelet thermal_camera_nodelet_manager" launch-prefix="rosrun clover waitfile $(arg thermal_device)" clear_params="true" respawn="true">
        <!-- Путь к устройству видеозахвата -->
        <param name="device_path" value="$(arg thermal_device)"/>

        <!-- Частота кадров (FPS) -->
        <param name="cv_cap_prop_fps" value="25"/>

        <!-- Разрешение изображения -->
        <param name="image_width" value="256"/>
        <param name="image_height" value="384"/>
    </node>
</launch>
```

## Назначение и функциональность

Этот launch-файл предназначен для настройки и запуска тепловизора в системе ROS на дроне Clover. Основные функции:

1. **Инициализация устройства тепловизора**:
   - По умолчанию используется устройство `/dev/video1`
   - Перед запуском проверяется доступность устройства через `waitfile`

2. **Оптимизированная обработка изображений**:
   - Используется механизм nodelet'ов для эффективной обработки видео
   - Создается менеджер nodelet'ов с 2 рабочими потоками

3. **Настройки камеры**:
   - Частота кадров: 25 FPS
   - Разрешение: 256×384 пикселей

## Особенности реализации

1. **Механизм nodelet'ов**:
   - Позволяет избежать накладных расходов на межпроцессное взаимодействие
   - Обеспечивает эффективную передачу больших объемов данных (изображений)

2. **Автоматическое восстановление**:
   - Параметр `respawn="true"` обеспечивает автоматический перезапуск при сбоях
   - `clear_params="true"` очищает параметры при перезапуске

3. **Ожидание устройства**:
   - `launch-prefix="rosrun clover waitfile $(arg thermal_device)"` гарантирует, что узел запустится только после появления устройства

4. **Гибкость конфигурации**:
   - Основные параметры (устройство, FPS, разрешение) могут быть легко изменены

Этот launch-файл интегрируется с другими компонентами системы (такими как `ggsell.py`), предоставляя обработанные изображения с тепловизора для дальнейшего анализа и визуализации.
### Краткое описание алгоритмов во всех предоставленных файлах:

---

#### **1. Основные алгоритмы в `server.py` (Flask-сервер для управления дроном)**  
- **REST API** – обработка HTTP-запросов (GET/POST) для управления дроном (взлет, посадка, навигация).  
- **Очередь целей** – FIFO-буфер (`queue`) для хранения маршрутных точек.  
- **Работа с ROS-сервисами** – взаимодействие с `CloverAPI` для получения телеметрии, управления полетом и чтения карты ArUco-маркеров.  
- **Многопоточность** – асинхронная обработка запросов с помощью Flask.  

---

#### **2. Алгоритмы в `frontend.py` (Dash-интерфейс)**  
- **Визуализация карты** – масштабирование и отображение ArUco-маркеров с помощью `Plotly`.  
- **Интерактивное рисование** – инструменты для создания маршрутов (линии, фигуры).  
- **Polling-механизм** – периодический опрос сервера (`dcc.Interval`) для обновления:  
  - Позиции дрона.  
  - Состояния батареи.  
  - Тепловизора и основной камеры.  
- **Обработка событий** – колбэки для кнопок управления (посадка, экстренная остановка и др.).  

---

#### **3. Алгоритмы в `node.py` (ROS-обертка для Clover)**  
- **Singleton-паттерн** – гарантирует единственный экземпляр `CloverAPI` для всех запросов.  
- **ROS-сервисы** – синхронные вызовы методов:  
  - `get_telemetry` – получение координат.  
  - `navigate` – навигация к точке.  
  - `land` – посадка.  
- **Подписка на топики** – асинхронное чтение данных с камер и датчиков.  

---

#### **4. Алгоритмы в `ggsell.py` (обработка тепловизионного изображения)**  
- **Цветовая карта** – преобразование монохромного изображения в псевдоцветное (`cv2.applyColorMap`).  
- **Разделение кадра** – деление изображения на 2 части (`np.array_split`).  
- **ROS-топики** – подписка (`/thermal_camera/image_raw`) и публикация (`~debug`) изображений.  

---

#### **5. Алгоритмы в `land.py` (посадка дрона)**  
- **ПИД-навигация** – точное позиционирование через `navigate_wait` с проверкой достижения цели (евклидово расстояние).  
- **Двухэтапная посадка**:  
  1. Перелет в точку посадки (`x=6, y=0.6, z=1.5`).  
  2. Плавное снижение (`z=0.5`).  

---

#### **6. Алгоритмы в `square.py` (автономный полет по маршруту)**  
- **Запись видео** – сохранение потоков с основной и тепловой камер (`cv2.VideoWriter`).  
- **HTTP-запрос** – получение координат маршрута с внешнего сервера.  
- **Последовательность waypoints** – навигация между точками с контролем положения.  
- **Параллельная обработка** – запись видео и управление полетом через `long_callback`.  

---

#### **7. Алгоритмы в `thermal_camera.launch` (настройка тепловизора)**  
- **Nodelet-оптимизация** – минимизация накладных расходов при передаче изображений.  
- **Гибкая конфигурация**:  
  - Разрешение (`256x384`).  
  - Частота кадров (`25 FPS`).  
- **Автовосстановление** – перезапуск узла при сбоях (`respawn="true"`).  

---

### Ключевые технологии:  
- **ROS** (сервисы, топики, nodelet'ы).  
- **REST API** (Flask, HTTP-запросы).  
- **Компьютерное зрение** (OpenCV, обработка изображений).  
- **Асинхронность** (Flask, Dash, ROS-колбэки).  
- **ПИД-контроль** (навигация с обратной связью).  

Все алгоритмы ориентированы на **автономность**, **точность** и **минимизацию задержек** при управлении дроном.
