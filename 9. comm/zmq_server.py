# import zmq

# context = zmq.Context()
# socket = context.socket(zmq.REP)  # REP 表示响应模式
# socket.bind("tcp://127.0.0.1:5555")

# while True:
#     message = socket.recv_string()
#     print(f"收到消息: {message}")
#     socket.send_string("消息已收到")  


# server.py
import zmq

# 创建一个 ZeroMQ 上下文
context = zmq.Context()

# 创建 REP（响应）套接字
socket = context.socket(zmq.REP)

# 绑定到任意 IP 的 5555 端口（更换为实际服务器 IP 地址）
socket.bind("tcp://10.154.28.205:7778")  

print("服务器已启动，等待客户端连接...")

while True:
    # 接收消息
    # message = socket.recv_string()
    # print(f"收到客户端消息: {message}")  
    
    data = socket.recv_string()    
    all_list = [item.strip() for item in data.split(",")]   
    print("data :", all_list[0])  
    # 处理并响应
    response = f"已收到: {data}"
    socket.send_string(response)