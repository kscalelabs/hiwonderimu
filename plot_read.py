#!/usr/bin/env python3
import serial
import struct
import time

# adjust if needed
PORT = "/dev/ttyUSB0"
BAUD = 9600

# conversion factors from the datasheet:
#   Accel: ±16g → raw/32768*16 g      :contentReference[oaicite:0]{index=0}&#8203;:contentReference[oaicite:1]{index=1}
#   Gyro: ±2000°/s → raw/32768*2000 °/s&#8203;:contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}
#   Angle: ±180° → raw/32768*180°     :contentReference[oaicite:4]{index=4}&#8203;:contentReference[oaicite:5]{index=5}
SCALE_ACC   = 16.0   / 32768.0
SCALE_GYRO  = 2000.0 / 32768.0
SCALE_ANGLE = 180.0  / 32768.0

def open_imu(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser

def read_stream(ser):
    print("Listening for IMU packets…")
    while True:
        # find packet header
        if ser.read(1) != b'\x55':
            continue
        pkt_type = ser.read(1)
        if not pkt_type:
            continue
        pkt_type = pkt_type[0]

        # read payload (8 bytes) + checksum
        pkt = ser.read(9)
        if len(pkt) < 9:
            continue
        payload, chk = pkt[:8], pkt[8]

        # checksum: low‑8 bits of sum(0x55 + TYPE + payload)
        if ((0x55 + pkt_type + sum(payload)) & 0xFF) != chk:
            continue

        # dispatch by packet type:
        if pkt_type == 0x50:
            # Time: YY MM DD HH MN SS MSL MSH
            YY, MM, DD, HH, MN, SS, MSL, MSH = payload
            ms   = (MSH << 8) | MSL
            year = 2000 + YY
            print(f"Time: {year:04d}-{MM:02d}-{DD:02d} "
                  f"{HH:02d}:{MN:02d}:{SS:02d}.{ms:03d}")
        elif pkt_type == 0x51:
            # Accel: Ax Ay Az + temp
            Ax, Ay, Az, TL, TH = struct.unpack("<hhhBB", payload)
            temp = ((TH << 8) | TL) / 100.0
            print(f"Accel: x={Ax*SCALE_ACC:.3f}g  "
                  f"y={Ay*SCALE_ACC:.3f}g  "
                  f"z={Az*SCALE_ACC:.3f}g  "
                  f"temp={temp:.2f}°C")
        elif pkt_type == 0x52:
            # Gyro: Wx Wy Wz + voltage
            Wx, Wy, Wz, VoL, VoH = struct.unpack("<hhhBB", payload)
            voltage = ((VoH << 8) | VoL) / 100.0
            print(f"Gyro: x={Wx*SCALE_GYRO:.2f}°/s  "
                  f"y={Wy*SCALE_GYRO:.2f}°/s  "
                  f"z={Wz*SCALE_GYRO:.2f}°/s  "
                  f"V={voltage:.2f}V")
        elif pkt_type == 0x53:
            # Euler Angles: Roll Pitch Yaw + version
            RL, RH, PL, PH, YL, YH, VL, VH = payload
            roll  = struct.unpack("<h", bytes([RL, RH]))[0] * SCALE_ANGLE
            pitch = struct.unpack("<h", bytes([PL, PH]))[0] * SCALE_ANGLE
            yaw   = struct.unpack("<h", bytes([YL, YH]))[0] * SCALE_ANGLE
            version = (VH << 8) | VL
            print(f"Angles: Roll={roll:6.2f}°  "
                  f"Pitch={pitch:6.2f}°  "
                  f"Yaw={yaw:6.2f}°  "
                  f"Ver={version}")
        elif pkt_type == 0x59:
            # Quaternion: q0,q1,q2,q3
            Q0, Q1, Q2, Q3 = struct.unpack("<hhhh", payload)
            q0 = Q0 / 32768.0; q1 = Q1 / 32768.0
            q2 = Q2 / 32768.0; q3 = Q3 / 32768.0
            print(f"Quat: q0={q0:.4f}  q1={q1:.4f}  "
                  f"q2={q2:.4f}  q3={q3:.4f}")
        # otherwise ignore other types

if __name__ == "__main__":
    try:
        ser = open_imu(PORT, BAUD)
        read_stream(ser)
    except serial.SerialException as e:
        print("Could not open serial port:", e)
