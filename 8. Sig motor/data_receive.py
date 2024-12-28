# udp_server.py
import socket

# 定义IP地址和端口号
server_ip = '10.154.28.205'  # 本地环回地址，或者使用 '0.0.0.0' 监听所有可用的接口
server_port = 12345 

# 创建UDP服务器Socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((server_ip, server_port))

print(f"UDP server is listening on {server_ip}:{server_port}...")

# 不断接收消息
while True:  
    data, addr = server_socket.recvfrom(1024)  # 最大接收1024字节
    print(f"Received message from {addr}: {data.decode()}")  
    
    all_list = [item.strip() for item in data.split(",")]  
    
    print(all_list)    
    if data.decode().lower() == 'exit':
        print("Server is shutting down...") 
        break

server_socket.close()   