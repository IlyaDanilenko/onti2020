# Очень простой сокет-сервер, принимающий строку json
# У многих может возникнуть вопрос:"Почему не на ROS?"
# Ответ очень прост:"На операторском ноутбуке стоит Винда, а ROS ставить на винду, то еще удовольствие))"

import socket
import json
from datetime import datetime

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('10.5.6.180', 5050))

server.listen(1)

while True:
    connection, addres = server.accept()
    data = json.loads(connection.recv(1024))
    print(f"time: {datetime.now()}; address: {addres}; string: {data['string']}")
    connection.close()