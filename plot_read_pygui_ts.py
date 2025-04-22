#!/usr/bin/env python3
import serial, struct, time, math
import dearpygui.dearpygui as dpg

# --- IMU Serial & Buffers ------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUD = 9600
SCALE_ANGLE = 180.0 / 32768.0
SCALE_ACC   = 16.0   / 32768.0
SCALE_GYRO  = 2000.0 / 32768.0
MAX_POINTS = 200

# data buffers
t_buf = []
roll_buf = []
pitch_buf = []
yaw_buf = []
acc_t_buf = []
acc_x_buf = []
acc_y_buf = []
acc_z_buf = []
gyro_t_buf = []
gyro_x_buf = []
gyro_y_buf = []
gyro_z_buf = []

# open & configure IMU (ACC, GYRO, ANGLE)
def open_imu(port, baud):
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    # unlock registers
    ser.write(b'\xFF\xAA\x69\x88\xB5')
    # enable ACC(0x51), GYRO(0x52), ANGLE(0x53)
    ser.write(b'\xFF\xAA\x02\x0E\x02')
    return ser

# parse all pending packets into buffers
def update_buffers(ser, start_time):
    while ser.in_waiting >= 11:
        if ser.read(1) != b'\x55': continue
        pid = ser.read(1)[0]
        pkt = ser.read(9)
        if len(pkt) < 9: break
        payload, chk = pkt[:8], pkt[8]
        if ((0x55 + pid + sum(payload)) & 0xFF) != chk: continue
        t = time.time() - start_time
        if pid == 0x51:  # Accel
            Ax, Ay, Az, TL, TH = struct.unpack("<hhhBB", payload)
            acc_t_buf.append(t)
            acc_x_buf.append(Ax * SCALE_ACC)
            acc_y_buf.append(Ay * SCALE_ACC)
            acc_z_buf.append(Az * SCALE_ACC)
        elif pid == 0x52:  # Gyro
            Wx, Wy, Wz, VL, VH = struct.unpack("<hhhBB", payload)
            gyro_t_buf.append(t)
            gyro_x_buf.append(Wx * SCALE_GYRO)
            gyro_y_buf.append(Wy * SCALE_GYRO)
            gyro_z_buf.append(Wz * SCALE_GYRO)
        elif pid == 0x53:  # Euler Angles
            RL, RH, PL, PH, YL, YH, _, _ = payload
            roll  = struct.unpack("<h", bytes([RL, RH]))[0] * SCALE_ANGLE
            pitch = struct.unpack("<h", bytes([PL, PH]))[0] * SCALE_ANGLE
            yaw   = struct.unpack("<h", bytes([YL, YH]))[0] * SCALE_ANGLE
            t_buf.append(t)
            roll_buf.append(roll)
            pitch_buf.append(pitch)
            yaw_buf.append(yaw)
    # trim buffers
    def trim(buf):
        if len(buf) > MAX_POINTS:
            del buf[0:len(buf)-MAX_POINTS]
    for B in [t_buf, roll_buf, pitch_buf, yaw_buf,
              acc_t_buf, acc_x_buf, acc_y_buf, acc_z_buf,
              gyro_t_buf, gyro_x_buf, gyro_y_buf, gyro_z_buf]:
        trim(B)

# --- UI Setup ---------------------------------------------------------
dpg.create_context()
with dpg.window(label="IMU Visualization", width=2400, height=620):
    # Use a table layout for 4 columns
    with dpg.table(header_row=False, resizable=True, policy=dpg.mvTable_SizingStretchProp) as table:
        # define 4 columns
        for _ in range(4):
            dpg.add_table_column()
        # one row: 3D view + RPY + Accel + Gyro
        with dpg.table_row():
            # Column 1: 3D orientation
            with dpg.drawlist(width=600, height=600):
                with dpg.draw_layer(tag="layer", depth_clipping=True, perspective_divide=True, cull_mode=dpg.mvCullMode_None):
                    with dpg.draw_node(tag="axis_x"): dpg.draw_line([0,0,0],[1,0,0], color=[255,0,0,255], thickness=4)
                    with dpg.draw_node(tag="axis_y"): dpg.draw_line([0,0,0],[0,1,0], color=[0,255,0,255], thickness=4)
                    with dpg.draw_node(tag="axis_z"): dpg.draw_line([0,0,0],[0,0,1], color=[0,0,255,255], thickness=4)
            # Column 2: RPY timeseries
            with dpg.plot(label="RPY (°)", height=600, width=600):
                dpg.add_plot_axis(dpg.mvXAxis, tag="rpy_x", label="Time")
                ay = dpg.add_plot_axis(dpg.mvYAxis, tag="rpy_y", label="°")
                dpg.add_line_series([], [], label="Roll", parent=ay, tag="roll_s")
                dpg.add_line_series([], [], label="Pitch", parent=ay, tag="pitch_s")
                dpg.add_line_series([], [], label="Yaw", parent=ay, tag="yaw_s")
            # Column 3: Acceleration
            with dpg.plot(label="Accel (g)", height=600, width=600):
                dpg.add_plot_axis(dpg.mvXAxis, tag="acc_x", label="Time")
                ay2 = dpg.add_plot_axis(dpg.mvYAxis, tag="acc_y", label="g")
                dpg.add_line_series([], [], label="Ax", parent=ay2, tag="ax_s")
                dpg.add_line_series([], [], label="Ay", parent=ay2, tag="ay_s")
                dpg.add_line_series([], [], label="Az", parent=ay2, tag="az_s")
            # Column 4: Gyro
            with dpg.plot(label="Gyro (°/s)", height=600, width=600):
                dpg.add_plot_axis(dpg.mvXAxis, tag="gyr_x", label="Time")
                ay3 = dpg.add_plot_axis(dpg.mvYAxis, tag="gyr_y", label="°/s")
                dpg.add_line_series([], [], label="Wx", parent=ay3, tag="wx_s")
                dpg.add_line_series([], [], label="Wy", parent=ay3, tag="wy_s")
                dpg.add_line_series([], [], label="Wz", parent=ay3, tag="wz_s")

# Show viewport
dpg.create_viewport(title="IMU", width=2400, height=620)
dpg.setup_dearpygui()
dpg.show_viewport()

# Start serial and render loop
ser = open_imu(PORT, BAUD)
start_time = time.time()
# initialize 3D matrices
iw, ih = 600, 600
view_mat = dpg.create_lookat_matrix([0,0,5],[0,0,0],[0,1,0])
proj_mat = dpg.create_perspective_matrix(math.radians(45), iw/ih, 0.1, 100)
dpg.set_clip_space("layer", 0, 0, iw, ih, -1.0, 1.0)

while dpg.is_dearpygui_running():
    update_buffers(ser, start_time)
    # update series values
    dpg.set_value("roll_s",  [t_buf, roll_buf])
    dpg.set_value("pitch_s", [t_buf, pitch_buf])
    dpg.set_value("yaw_s",   [t_buf, yaw_buf])
    dpg.set_value("ax_s",    [acc_t_buf, acc_x_buf])
    dpg.set_value("ay_s",    [acc_t_buf, acc_y_buf])
    dpg.set_value("az_s",    [acc_t_buf, acc_z_buf])
    dpg.set_value("wx_s",    [gyro_t_buf, gyro_x_buf])
    dpg.set_value("wy_s",    [gyro_t_buf, gyro_y_buf])
    dpg.set_value("wz_s",    [gyro_t_buf, gyro_z_buf])
    # axis limits
    if t_buf:
        for tag in ["rpy_x","acc_x","gyr_x"]:
            dpg.set_axis_limits(tag, t_buf[0], t_buf[-1])
    dpg.set_axis_limits("rpy_y", -180, 180)
    dpg.set_axis_limits("acc_y", -2, 2)
    dpg.set_axis_limits("gyr_y", -500, 500)
    # update 3D orientation
    if roll_buf:
        rd, pd, yd = roll_buf[-1], pitch_buf[-1], yaw_buf[-1]
        rz = dpg.create_rotation_matrix(math.radians(yd), [0,0,1])
        ry = dpg.create_rotation_matrix(math.radians(pd), [0,1,0])
        rx = dpg.create_rotation_matrix(math.radians(rd), [1,0,0])
        model = rz * ry * rx
        xf = proj_mat * view_mat * model
        for axis in ["axis_x","axis_y","axis_z"]:
            dpg.apply_transform(axis, xf)
    dpg.render_dearpygui_frame()

# cleanup
ser.close()
dpg.destroy_context()