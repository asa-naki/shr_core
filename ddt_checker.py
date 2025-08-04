import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time

PORT = "/dev/ttyACM0"  # 使用しているRS485ポートに変更
BAUDRATE = 115200

def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def check_port_availability():
    """利用可能なシリアルポートをチェックする"""
    available_ports = []
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        try:
            with serial.Serial(port.device, BAUDRATE, timeout=0.1) as ser:
                available_ports.append(port.device)
        except (serial.SerialException, OSError):
            continue
    
    return available_ports

def is_port_available(port_name):
    """指定されたポートが利用可能かチェックする"""
    try:
        with serial.Serial(port_name, BAUDRATE, timeout=0.1) as ser:
            return True
    except (serial.SerialException, OSError):
        return False

def refresh_port_status():
    """ポート状態を更新"""
    if is_port_available(PORT):
        labels["Port Status"].config(text="利用可能", foreground="green")
    else:
        labels["Port Status"].config(text="利用不可", foreground="red")
    
    # 利用可能なポート一覧を表示
    available_ports = check_port_availability()
    if available_ports:
        port_list.set("利用可能ポート: " + ", ".join(available_ports))
    else:
        port_list.set("利用可能なポートがありません")

def get_motor_info():
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=0.1) as ser:
            cmd = bytes([0x74] + [0x00]*8)
            cmd += bytes([crc8_maxim(cmd)])  # CRC8追加
            ser.write(cmd)
            time.sleep(0.05)
            response = ser.read(10)

            if len(response) != 10:
                raise Exception("レスポンス不足")

            data = list(response)
            id_ = data[0]
            mode = data[1]
            current = (data[2] << 8 | data[3]) / 4096.0  # 例: A単位想定
            velocity = (data[4] << 8 | data[5])
            temp = data[6]
            angle = data[7] * 360 / 256
            fault = data[8]

            # 表示更新
            labels["ID"].config(text=str(id_))
            labels["Mode"].config(text=str(mode))
            labels["Current"].config(text=f"{current:.2f} A")
            labels["Velocity"].config(text=f"{velocity} rpm")
            labels["Temperature"].config(text=f"{temp} ℃")
            labels["Angle"].config(text=f"{angle:.1f} °")
            labels["Fault"].config(text=f"0x{fault:02X}")

    except Exception as e:
        messagebox.showerror("通信エラー", str(e))

def set_motor_id():
    new_id = entry_id.get()
    try:
        new_id_int = int(new_id)
        if not 0 <= new_id_int <= 255:
            raise ValueError("0〜255の範囲で入力してください")
    except ValueError as e:
        messagebox.showerror("IDエラー", str(e))
        return

    cmd = bytes([0xAA, 0x55, 0x53, new_id_int] + [0x00]*6)
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=0.1) as ser:
            for _ in range(5):
                ser.write(cmd)
                time.sleep(0.05)
        messagebox.showinfo("完了", f"IDを {new_id_int} に変更しました。\n再接続時に有効になります。")
    except Exception as e:
        messagebox.showerror("通信エラー", str(e))


# GUI
root = tk.Tk()
root.title("モーター状態表示 & ID設定")
root.geometry("400x360")

frame_info = ttk.LabelFrame(root, text="モーター情報", padding=10)
frame_info.pack(padx=10, pady=10, fill="x")

labels = {}
for i, key in enumerate(["ID", "Mode", "Current", "Velocity", "Temperature", "Angle", "Fault", "Port Status"]):
    ttk.Label(frame_info, text=f"{key}:").grid(row=i, column=0, sticky="e")
    labels[key] = ttk.Label(frame_info, text="---")
    labels[key].grid(row=i, column=1, sticky="w")

ttk.Button(frame_info, text="情報更新", command=get_motor_info).grid(row=8, columnspan=2, pady=5)
ttk.Button(frame_info, text="ポート状態更新", command=refresh_port_status).grid(row=9, columnspan=2, pady=5)

frame_set = ttk.LabelFrame(root, text="ID設定", padding=10)
frame_set.pack(padx=10, pady=10, fill="x")

ttk.Label(frame_set, text="新しいID (0〜255):").pack(side="left")
entry_id = ttk.Entry(frame_set, width=10)
entry_id.pack(side="left", padx=5)

ttk.Button(frame_set, text="ID変更", command=set_motor_id).pack(side="left", padx=5)

# ポート状態表示用ラベル
port_list = tk.StringVar()
ttk.Label(root, textvariable=port_list, foreground="blue").pack(pady=5)

refresh_port_status()  # 初期状態のポートチェック
root.mainloop()
