#!/usr/bin/env python3
import serial, struct, time, math
import dearpygui.dearpygui as dpg
import numpy as np

# adjust this to your port/baud
PORT = "/dev/ttyUSB0"
BAUD = 9600

# conversion factor from raw to degrees (±180° → raw/32768*180°)
SCALE_ANGLE = 180.0 / 32768.0

def open_imu(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    # unlock & enable angle (0x53) stream if you haven’t already
    ser.write(b'\xFF\xAA\x69\x88\xB5')       # unlock
    ser.write(b'\xFF\xAA\x02\x08\x02')       # RSW = bit3 for 0x53 only
    time.sleep(0.05)
    return ser

def read_euler(ser):
    """Blocks until a valid 0x53 packet is read, then returns (roll,pitch,yaw) in radians."""
    while True:
        if ser.read(1) != b'\x55':
            continue
        pid = ser.read(1)
        if not pid or pid[0] != 0x53:
            ser.read(9)
            continue
        pkt = ser.read(9)
        if len(pkt) < 9:
            continue
        payload, chk = pkt[:8], pkt[8]
        if ((0x55 + 0x53 + sum(payload)) & 0xFF) != chk:
            continue
        RL, RH, PL, PH, YL, YH, _, _ = payload
        r = struct.unpack("<h", bytes([RL, RH]))[0] * SCALE_ANGLE
        p = struct.unpack("<h", bytes([PL, PH]))[0] * SCALE_ANGLE
        y = struct.unpack("<h", bytes([YL, YH]))[0] * SCALE_ANGLE
        return math.radians(r), math.radians(p), math.radians(y)

# --- DearPyGui setup (from First Steps) :contentReference[oaicite:0]{index=0} ---
dpg.create_context()
dpg.create_viewport(title="IMU Orientation", width=600, height=600)

with dpg.window(label="Orientation", width=600, height=600):
    with dpg.drawlist(width=600, height=600):
        with dpg.draw_layer(tag="layer",
                            depth_clipping=True,
                            perspective_divide=True,
                            cull_mode=dpg.mvCullMode_None):
            # three axis nodes
            with dpg.draw_node(tag="axis_x"):
                dpg.draw_line([0,0,0], [1,0,0], color=[255,0,0,255], thickness=4)
            with dpg.draw_node(tag="axis_y"):
                dpg.draw_line([0,0,0], [0,1,0], color=[0,255,0,255], thickness=4)
            with dpg.draw_node(tag="axis_z"):
                dpg.draw_line([0,0,0], [0,0,1], color=[0,0,255,255], thickness=4)

dpg.setup_dearpygui()
dpg.show_viewport()

# build camera & projection once :contentReference[oaicite:1]{index=1}
width, height = 600, 600
view = dpg.create_lookat_matrix([0,0,5], [0,0,0], [0,1,0])
proj = dpg.create_perspective_matrix(math.radians(45), width/height, 0.1, 100)
dpg.set_clip_space("layer", 0, 0, width, height, -1.0, 1.0)

# open IMU
ser = open_imu(PORT, BAUD)

# main loop
while dpg.is_dearpygui_running():
    # get new orientation
    roll, pitch, yaw = read_euler(ser)

    # build model matrix (Rz * Ry * Rx)
    Rz = dpg.create_rotation_matrix(yaw,   [0,0,1])
    Ry = dpg.create_rotation_matrix(pitch, [0,1,0])
    Rx = dpg.create_rotation_matrix(roll,  [1,0,0])
    model = Rz * Ry * Rx

    # composite transform
    transform = proj * view * model

    # apply to each axis node
    dpg.apply_transform("axis_x", transform)
    dpg.apply_transform("axis_y", transform)
    dpg.apply_transform("axis_z", transform)

    dpg.render_dearpygui_frame()

# cleanup
ser.close()
dpg.destroy_context()
