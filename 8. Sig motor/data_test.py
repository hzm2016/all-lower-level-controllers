import struct


# 四字节数据
bytes_data = [0x6A, 0x67, 0xE1, 0xC0]

# 将四字节数组转换为二进制数据
packed_data = bytes(bytes_data)

# 使用 'f' 格式将4字节转换为 float32 (单精度浮点数)
float_value = struct.unpack('f', packed_data)[0]

# 输出转换后的浮点数
print(f'Converted float: {float_value}')