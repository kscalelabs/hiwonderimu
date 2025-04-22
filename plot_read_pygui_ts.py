#!/usr/bin/env python3
import serial, struct, time, math
import dearpygui.dearpygui as dpg

# --- IMU Serial Setup ------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUD = 9600
# conversion factor: raw->deg (±180°)
SCALE_ANGLE = 180.0 / 32768.0
# time-series buffer length
MAX_POINTS = 200

# buffers for time and RPY and time stamps
t_buf = []
roll_buf = []
pitch_buf = []
yaw_buf = []

# open and configure IMU
def open_imu(port, baud):
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    # unlock registers
    ser.write(b'\xFF\xAA\x69\x88\xB5')
    # enable Euler stream (TYPE=0x53, bit3)
    ser.write(b'\xFF\xAA\x02\x08\x02')
    return ser

# read a single Euler packet
def read_euler(ser):
    while True:
        if ser.read(1) != b'\x55': continue
        pid = ser.read(1)
        if not pid or pid[0] != 0x53:
            ser.read(9)
            continue
        pkt = ser.read(9)
        if len(pkt) < 9: continue
        payload, chk = pkt[:8], pkt[8]
        if ((0x55+0x53+sum(payload)) & 0xFF) != chk:
            continue
        RL, RH, PL, PH, YL, YH, _, _ = payload
        # raw to degrees
        roll_deg  = struct.unpack("<h", bytes([RL, RH]))[0] * SCALE_ANGLE
        pitch_deg = struct.unpack("<h", bytes([PL, PH]))[0] * SCALE_ANGLE
        yaw_deg   = struct.unpack("<h", bytes([YL, YH]))[0] * SCALE_ANGLE
        return roll_deg, pitch_deg, yaw_deg

# --- DearPyGui Setup --------------------------------------------------
dpg.create_context()

# 3D Orientation Window
with dpg.window(label="3D Orientation", width=600, height=600):
    with dpg.drawlist(width=600, height=600):
        with dpg.draw_layer(tag="layer", depth_clipping=True, perspective_divide=True, cull_mode=dpg.mvCullMode_None):
            with dpg.draw_node(tag="axis_x"): dpg.draw_line([0,0,0],[1,0,0], color=[255,0,0,255], thickness=4)
            with dpg.draw_node(tag="axis_y"): dpg.draw_line([0,0,0],[0,1,0], color=[0,255,0,255], thickness=4)
            with dpg.draw_node(tag="axis_z"): dpg.draw_line([0,0,0],[0,0,1], color=[0,0,255,255], thickness=4)

# RPY Time Series Window
with dpg.window(label="IMU Roll/Pitch/Yaw", width=600, height=600, pos=(610, 0)):
    # Roll
    with dpg.plot(label="Roll (°)", height=150):
        dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="roll_xaxis")
        dpg.add_plot_axis(dpg.mvYAxis, label="°", tag="roll_yaxis")
        dpg.add_line_series([], [], parent="roll_yaxis", tag="roll_series")
    # Pitch
    with dpg.plot(label="Pitch (°)", height=150):
        dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="pitch_xaxis")
        dpg.add_plot_axis(dpg.mvYAxis, label="°", tag="pitch_yaxis")
        dpg.add_line_series([], [], parent="pitch_yaxis", tag="pitch_series")
    # Yaw
    with dpg.plot(label="Yaw (°)", height=150):
        dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="yaw_xaxis")
        dpg.add_plot_axis(dpg.mvYAxis, label="°", tag="yaw_yaxis")
        dpg.add_line_series([], [], parent="yaw_yaxis", tag="yaw_series")

# viewport
dpg.create_viewport(title="IMU Visualization", width=1220, height=620)
dpg.setup_dearpygui()
dpg.show_viewport()

# initialize 3D camera & projection
w, h = 600, 600
view = dpg.create_lookat_matrix([0,0,5],[0,0,0],[0,1,0])
proj = dpg.create_perspective_matrix(math.radians(45), w/h, 0.1, 100)
dpg.set_clip_space("layer", 0, 0, w, h, -1.0, 1.0)

# open IMU
er = open_imu(PORT, BAUD)
start = time.time()

# Main Loop
while dpg.is_dearpygui_running():
    # read degrees
    roll_deg, pitch_deg, yaw_deg = read_euler(ser:=er)
    # convert to radians for 3D transform
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    # timestamp
    t = time.time() - start
    # update buffers
    t_buf.append(t); roll_buf.append(roll_deg); pitch_buf.append(pitch_deg); yaw_buf.append(yaw_deg)
    if len(t_buf) > MAX_POINTS:
        t_buf.pop(0); roll_buf.pop(0); pitch_buf.pop(0); yaw_buf.pop(0)
    # update timeseries
    dpg.set_value("roll_series", [t_buf, roll_buf])
    dpg.set_value("pitch_series", [t_buf, pitch_buf])
    dpg.set_value("yaw_series", [t_buf, yaw_buf])
    # adjust axes
    if t_buf:
        dpg.set_axis_limits("roll_xaxis", t_buf[0], t_buf[-1])
        dpg.set_axis_limits("pitch_xaxis", t_buf[0], t_buf[-1])
        dpg.set_axis_limits("yaw_xaxis", t_buf[0], t_buf[-1])
        dpg.set_axis_limits("roll_yaxis", -180.0, 180.0)
        dpg.set_axis_limits("pitch_yaxis", -180.0, 180.0)
        dpg.set_axis_limits("yaw_yaxis", -180.0, 180.0)
    # update 3D axes with proper radians
    Rz = dpg.create_rotation_matrix(yaw, [0,0,1])
    Ry = dpg.create_rotation_matrix(pitch, [0,1,0])
    Rx = dpg.create_rotation_matrix(roll, [1,0,0])
    model = Rz * Ry * Rx
    transform = proj * view * model
    dpg.apply_transform("axis_x", transform)
    dpg.apply_transform("axis_y", transform)
    dpg.apply_transform("axis_z", transform)

    dpg.render_dearpygui_frame()

# cleanup
er.close()
dpg.destroy_context()