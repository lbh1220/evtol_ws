import time
import csv
from pymavlink import mavutil

# 创建连接到MAVLink的实例，指定连接方式（UDP，串口等）
# 这里以UDP连接为例，通常地址是本地主机的14550端口
mav = mavutil.mavlink_connection('udp:127.0.0.1:14445')

# 等待心跳，确认连接
mav.wait_heartbeat()
print("MAVLink connection established!")

# 打开CSV文件进行数据记录
with open('rpm_data.csv', 'w', newline='') as csvfile:
    # fieldnames = ['timestamp', 'roll', 'pitch', 'yaw', 'rpm']
    fieldnames = ['timestamp', 'rpm']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    # 循环接收MAVLink消息
    try:
        while True:
            # 获取ATTITUDE消息
            msg = mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
            if msg:
                # 将消息数据记录到CSV文件
                # writer.writerow({
                #     'timestamp': time.time(),
                #     # 'roll': msg.roll,
                #     # 'pitch': msg.pitch,
                #     # 'yaw': msg.yaw,
                #     'rpm': msg.rpm
                # })
                # print(f"Recorded: Roll={msg.roll}, Pitch={msg.pitch}, Yaw={msg.yaw}, Rpm={msg.rpm}")
                print(f"Recorded: Rpm={msg.servo1_raw}")

    except KeyboardInterrupt:
        print("Stopped by User")
