import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)  # REQ 表示请求模式
socket.connect("tcp://127.0.0.1:5555")

while True:  
    # 发送消息并等待响应  
    socket.send_string("Hello, Server!")  
    message = socket.recv_string()  
    print(f"收到响应: {message}")    