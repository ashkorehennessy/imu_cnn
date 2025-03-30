import serial
import numpy as np
import os
import re
from datetime import datetime

# ====== 配置参数 ======
PORT = "/dev/ttyUSB0"  # 串口设备（Windows为COMx，Linux/Mac为/dev/tty*）
BAUDRATE = 115200               # 波特率（需与ESP32一致）
WINDOW_SIZE = 200               # 目标数据长度（根据模型调整）
SAVE_DIR = "dataset/9"          # 保存目录（按标签分类，如数字5）
MAX_SAMPLES = 50                # 最大采集样本数
# =====================

def ensure_dir(dir_path):
    """创建目录（如果不存在）"""
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)


def main():
    # 初始化串口
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"连接串口: {ser.name}")

    # 初始化保存目录
    ensure_dir(SAVE_DIR)
    sample_count = 0

    # 数据采集状态机
    recording = False
    raw_data = []

    try:
        while sample_count < MAX_SAMPLES:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if not line:
                continue

            if "jitter" in line:
                print("等待抖动延时...")
                continue

            if "write" in line:
                print("开始采集数据，请书写相应数字...")
                continue

            # 检测数据块开始
            if "SAMPLE_START" in line and not recording:
                recording = True
                raw_data = []
                print("开始接收数据...")
                continue

            # 检测数据块结束
            if "SAMPLE_END" in line and recording:
                recording = False
                sample_count += 1

                # 处理数据
                processed = raw_data
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{SAVE_DIR}/{timestamp}_{sample_count}.npy"
                np.save(filename, processed)
                print(f"保存样本 {sample_count}/{MAX_SAMPLES}: {filename}")
                continue

            # 记录数据（跳过表头和非数值行）
            if recording and re.match(r"^-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*$", line):
                row = list(map(float, line.split(',')))
                raw_data.append(row)

    except KeyboardInterrupt:
        print("用户中断采集")
    finally:
        ser.close()

if __name__ == "__main__":
    main()