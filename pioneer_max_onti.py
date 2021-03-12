# GitHub Пионер Макс: https://github.com/geoscan/geoscan_pioneer_max
# Пионер Макс это Open Source система, поэтому весь код находится в этом репозитории
# Этот модуль был написан с помощью оберток над ROS, обертки просто упрощяют работу с ROSом, использованы они были для ускорения разработки модуля, при сильном желании это все можно переписать под голый ROS
# Вообще Пионер Макс работает весь на ROS
# В этом репозитории есть рабочая среда со всеми пакетами под Пионер Макс
# Так же есть исходники Web-menu и других Максовских утилит

import rospy
from gs_module import CargoController
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from time import sleep, time
import socket, json
from sensor_msgs.msg import Image
import cv2 # необходимо для работы CvBridge
from cv_bridge import CvBridge
import numpy as np

cloud = [] # облако направлений, формат данных: список из кортежей(можно также из списков). В кортеже первым элементом время, вторым смещение по координате x, третьим смещение по координате z
point = 0 # индекс текущего направления

t = 0.0 # время текущей точки
t_old = 0.0 # время при старте движения по направлению из облака
x = 0 # координата X
y = 0 # координата Y
z = 1.3 # координата Z
# доступ к координатам все-таки был :D

cargo = CargoController()
board = BoardManager()
bridge = CvBridge()

take = False # флаг осуществлен ли взлет
flight = False # флаг полета

left_finish = False # флаг выхода из функции left
right_finish = False # флаг выхода из функции right
forward_finish = False # флаг выхода из функции forward
backward_finish = False # флаг выхода из функции backward
t_p = False # флаг, говорящий, что нужно обновить t_old. Это был хотфикс, если делать по уму, то нужно анализировать point

def __callback_take(data): # колбэк для взлета, ловит сообщения из топика, отвечающего за события автопилота
    global take
    global ap_take
    global x
    global y
    global z
    event = data.data
    if take:
        if event == CallbackEvent.ENGINES_STARTED: 
            ap_take.takeoff() # взлетаем
        elif event == CallbackEvent.TAKEOFF_COMPLETE:
            ap_take.goToLocalPoint(x, y, z) # поднимаемся на нужную высоту
        elif event == CallbackEvent.POINT_REACHED:
            take = False

ap_take = FlightController(__callback_take) # объявляем контроллер взлета

# Рывки и долгий полет на секундах по факту костыль, для решения проблемы оптического потока (поле однотонное, оптический поток не всегда понимает что что-то изменилось)
def __callback_flight(data): # колбэк для полета, ловит сообщения из топика, отвечающего за события автопилота
    global x
    global y
    global z
    global t
    global t_old
    global flight
    global ap_flight
    global point
    global t_p

    event = data.data
    if flight:
        if event == CallbackEvent.POINT_REACHED:
            t = time() # запоминаем время достжения точки
            x += cloud[point][1] # делаем приращение к координате X
            y += cloud[point][2] # делаем приращение к координате Y
            if not t_p:
                t_old = time() # обновляю время точки
                t_p = True
            if t-t_old < cloud[point][0]: # если дельта времен меньле чем указанное время летим дальше
                ap_flight.goToLocalPoint(x+cloud[point][1], y + cloud[point][2], z)
            else:
                point += 1
                t_old = time()
                if point < len(cloud):
                    ap_flight.goToLocalPoint(x+cloud[point][1], y + cloud[point][2], z)
                else:
                    flight = False
                    t_p = False

ap_flight = FlightController(__callback_flight) # объявление контроллера полета

def init():
    global cargo
    cargo.changeAllColor() # отключаем подсветку
    rospy.init_node("mission_onti_node") # инициалицируем ноду

def cargo_on():
    global cargo
    cargo.on() # включаем магнит

def cargo_off():
    global cargo
    cargo.off() # выключаем магнит

def __flight(cl):
    global cloud
    global point
    global ap_flight
    global flight
    global x
    global y
    global z

    point = 0
    cloud = cl
    flight = True

    ap_flight.goToLocalPoint(x + cloud[point][1], y + 0.1, z) # запускаем полет в первую точку
    while not rospy.is_shutdown() and flight: # ждем завершения полета в нужном направлении (эмитация синхронности)
        pass
        
def __take():
    global ap_take
    global take
    take = True
    ap_take.preflight() # делаем предстартовую подготовку
    while not rospy.is_shutdown() and take: # ждем завершения взлета (эмитация синхронности)
        pass

def go_from_2_to_4():
    from_2_to_4 = [
        (9, 0, -0.1),
        (38, 0.1, 0),
        (10, 0, 0.1),
        (12, 0.1, 0)
    ]
    __flight(from_2_to_4)


def go_from_4_to_2():
    from_4_to_2 = [
        (13.5, -0.1, 0),
        (10, 0 ,-0.1),
        (38, -0.1, 0),
        (9, 0, 0.1)
    ]
    __flight(from_4_to_2)

def go_to_2():
    to_2 = [
        (8.5, 0, 0.1),
        (29.5, -0.1, 0),
        (9.5, 0, 0.1)
    ]
    __flight(to_2)

def go_to_4():
    to_4 = [
        (9.5,0.1, 0),
        (21, 0, 0.1),
        (13.5, 0.1, 0)
    ]
    __flight(to_4)

def go_from_4_to_test():
    from_2_to_4_test = [
        (22, 0, 0.1)
    ]
    __flight(from_2_to_4_test)

def __callback_left(data): # колбэк смещения влево
    global left_finish
    event = data.data
    if event == CallbackEvent.POINT_REACHED:
        left_finish = True

def left(distance):
    global x
    global y
    global z
    global left_finish
    left_finish = False

    left_ap = FlightController(__callback_left) # создаем контроллер для смещения
    left_ap.goToLocalPoint(x-distance,y,z) # смещяемся на указанную дистанцию
    while not rospy.is_shutdown() and not left_finish: # ждем завершения полета до точки
        pass
    x -= distance # обновляем координату
    
def __callback_right(data): # колбэк смещения вправо
    global right_finish
    event = data.data
    if event == CallbackEvent.POINT_REACHED:
        right_finish = True

def right(distance):
    global x
    global y
    global z
    global right_finish
    right_finish = False
    right_ap = FlightController(__callback_right) # создаем контроллер для смещения
    right_ap.goToLocalPoint(x+distance, y, z) # смещяемся на указанную дистанцию
    while not rospy.is_shutdown() and not right_finish: # ждем завершения полета до точки
        pass
    x += distance # обновляем координату

def __callback_forward(data): # колбэк смещения вперед
    global forward_finish
    event = data.data
    if event == CallbackEvent.POINT_REACHED:
        forward_finish = True

def forward(distance):
    global x
    global y
    global z
    global forward_finish
    forward_finish = False
    forward_ap = FlightController(__callback_forward) # создаем контроллер для смещения
    forward_ap.goToLocalPoint(x, y+distance, z) # смещяемся на указанную дистанцию
    while not rospy.is_shutdown() and not forward_finish: # ждем завершения полета до точки
        pass
    y += distance # обновляем координату

def __callback_backward(data):
    global backward_finish
    event = data.data
    if event == CallbackEvent.POINT_REACHED:
        backward_finish = True

def backward(distance):
    global x
    global y
    global z
    global backward_finish
    backward_finish = False

    backward_ap = FlightController(__callback_backward) # создаем контроллер для смещения
    backward_ap.goToLocalPoint(x, y-distance, z) # смещяемся на указанную дистанцию
    while not rospy.is_shutdown() and not backward_finish: # ждем завершения полета до точки
        pass
    y -= distance # обновляем координату

def light_rainbow(seconds):
    global cargo
    color = [
        (255, 0, 0), # красный
        (255, 128, 0), # оранжевый
        (255, 255, 0), # желтый
        (0, 255, 0), # зеленый
        (85, 170, 255), # голубой
        (0, 0, 255), # синий
        (128, 0, 255) # фиолетовый
    ]
    cargo.changeAllColor()
    i = 0
    n = 0
    t_s = time()
    while time() - t_s < seconds:
        for j in range(0,4):
            cargo.changeColor(color[i][0], color[i][1], color[i][2], j)
            sleep(0.125)
        if i >= len(color) - 1:
            i = 0
            n+=1
        else:
            i+=1

    cargo.changeAllColor()

def light_green(seconds):
    global cargo
    cargo.changeAllColor(0, 255, 0)
    sleep(seconds)
    cargo.changeAllColor()

def light_red(seconds):
    global cargo
    cargo.changeAllColor(255, 0, 0)
    sleep(seconds)
    cargo.changeAllColor()

def print(string): # по факту это просто передача строки через сокет, на ноутбуке организатора был сервер
    clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clientsocket.connect(('10.5.6.180', 5050))
    clientsocket.send(bytes(json.dumps({'string': string}), encoding='utf-8'))

def get_image():
    data = rospy.wait_for_message("pioneer_max_camera/image_raw", Image) # ждем сообщения из топика картинки
    return bridge.imgmsg_to_cv2(data,"bgr8") # переводим картинку из ROS сообщения в массив, который воспринимает cv2

def takeoff():
    __take()
    
def landing():
    global ap_take
    ap_take.landing() # садимся