#!/usr/bin/env python3
import serial, struct, time, math
import numpy as np
from scipy.spatial.transform import Rotation
import dearpygui.dearpygui as dpg

# --- Constants ---------------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUD = 9600
SCALE_ANGLE = 180.0 / 32768.0
SCALE_ACC   = 16.0   / 32768.0
SCALE_GYRO  = 2000.0 / 32768.0
MAX_POINTS = 200
PLOT_WIDTH = 600
PLOT_HEIGHT = 600

# --- Data buffers ------------------------------------------------------
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
quat_t_buf = []
quat_q0_buf = []
quat_q1_buf = []
quat_q2_buf = []
quat_q3_buf = []
grav_t_buf = []
grav_x_buf = []
grav_y_buf = []
grav_z_buf = []

# --- Trimming helpers --------------------------------------------------
def trim(buf):
    if len(buf) > MAX_POINTS:
        del buf[:-MAX_POINTS]

def trim_all():
    for B in [t_buf, roll_buf, pitch_buf, yaw_buf,
              acc_t_buf, acc_x_buf, acc_y_buf, acc_z_buf,
              gyro_t_buf, gyro_x_buf, gyro_y_buf, gyro_z_buf,
              quat_t_buf, quat_q0_buf, quat_q1_buf, quat_q2_buf, quat_q3_buf,
              grav_t_buf, grav_x_buf, grav_y_buf, grav_z_buf]:
        trim(B)

# --- IMU interface -----------------------------------------------------
def open_imu(port, baud):
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.write(b'\xFF\xAA\x69\x88\xB5')  # unlock registers
    ser.write(b'\xFF\xAA\x02\x4E\x02')  # enable ACC, GYRO, ANGLE, QUAT
    return ser

# --- Packet parsing ----------------------------------------------------
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
            Ax, Ay, Az, _, _ = struct.unpack("<hhhBB", payload)
            acc_t_buf.append(t)
            acc_x_buf.append(Ax * SCALE_ACC)
            acc_y_buf.append(Ay * SCALE_ACC)
            acc_z_buf.append(Az * SCALE_ACC)
        elif pid == 0x52:  # Gyro
            Wx, Wy, Wz, _, _ = struct.unpack("<hhhBB", payload)
            gyro_t_buf.append(t)
            gyro_x_buf.append(Wx * SCALE_GYRO)
            gyro_y_buf.append(Wy * SCALE_GYRO)
            gyro_z_buf.append(Wz * SCALE_GYRO)
        elif pid == 0x53:  # Euler angles
            RL, RH, PL, PH, YL, YH, _, _ = payload
            roll  = struct.unpack("<h", bytes([RL, RH]))[0] * SCALE_ANGLE
            pitch = struct.unpack("<h", bytes([PL, PH]))[0] * SCALE_ANGLE
            yaw   = struct.unpack("<h", bytes([YL, YH]))[0] * SCALE_ANGLE
            t_buf.append(t)
            roll_buf.append(roll)
            pitch_buf.append(pitch)
            yaw_buf.append(yaw)
        elif pid == 0x59:  # Quaternion + gravity
            Q0, Q1, Q2, Q3 = struct.unpack("<hhhh", payload)
            x, y, z, w = Q1/32768.0, Q2/32768.0, Q3/32768.0, Q0/32768.0
            quat_t_buf.append(t)
            quat_q0_buf.append(w)
            quat_q1_buf.append(x)
            quat_q2_buf.append(y)
            quat_q3_buf.append(z)
            # project gravity in world frame
            r = Rotation.from_quat([x, y, z, w])
            gx, gy, gz = r.apply([0, 0, -1])
            grav_t_buf.append(t)
            grav_x_buf.append(gx)
            grav_y_buf.append(gy)
            grav_z_buf.append(gz)
    trim_all()

# --- DearPyGui UI setup -----------------------------------------------
dpg.create_context()
with dpg.window(label="IMU Visualization", width=2500, height=1250):
    with dpg.table(header_row=False, resizable=True, policy=dpg.mvTable_SizingStretchProp):
        for _ in range(4): dpg.add_table_column()
        # first row: Orientation 3D, Gravity 3D, RPY, Accel
        with dpg.table_row():
            # orientation axes
            with dpg.drawlist(width=PLOT_WIDTH, height=PLOT_HEIGHT):
                with dpg.draw_layer(tag="ori_layer", depth_clipping=True, perspective_divide=True, cull_mode=dpg.mvCullMode_None):
                    with dpg.draw_node(tag="axis_x"): dpg.draw_line([0,0,0],[1,0,0], color=[255,0,0,255], thickness=4)
                    with dpg.draw_node(tag="axis_y"): dpg.draw_line([0,0,0],[0,1,0], color=[0,255,0,255], thickness=4)
                    with dpg.draw_node(tag="axis_z"): dpg.draw_line([0,0,0],[0,0,1], color=[0,0,255,255], thickness=4)
            # gravity vector
            with dpg.drawlist(width=PLOT_WIDTH, height=PLOT_HEIGHT):
                with dpg.draw_layer(tag="grav_layer", depth_clipping=True, perspective_divide=True, cull_mode=dpg.mvCullMode_None):
                    with dpg.draw_node(tag="gvec"): dpg.draw_line([0,0,0],[0,0,-1], color=[255,255,0,255], thickness=4)
            # RPY timeseries
            with dpg.plot(label="RPY (°)", width=PLOT_WIDTH, height=PLOT_HEIGHT):
                dpg.add_plot_axis(dpg.mvXAxis, tag="rpy_x", label="Time")
                ay = dpg.add_plot_axis(dpg.mvYAxis, tag="rpy_y", label="°")
                dpg.add_line_series([], [], label="Roll", parent=ay, tag="roll_s")
                dpg.add_line_series([], [], label="Pitch", parent=ay, tag="pitch_s")
                dpg.add_line_series([], [], label="Yaw", parent=ay, tag="yaw_s")
            # Accel timeseries
            with dpg.plot(label="Accel (g)", width=PLOT_WIDTH, height=PLOT_HEIGHT):
                dpg.add_plot_axis(dpg.mvXAxis, tag="acc_x", label="Time")
                ay2 = dpg.add_plot_axis(dpg.mvYAxis, tag="acc_y", label="g")
                dpg.add_line_series([], [], label="Ax", parent=ay2, tag="ax_s")
                dpg.add_line_series([], [], label="Ay", parent=ay2, tag="ay_s")
                dpg.add_line_series([], [], label="Az", parent=ay2, tag="az_s")
        # second row: Gyro, Quaternion, Gravity TS, spacer
        with dpg.table_row():
            with dpg.plot(label="Gyro (°/s)", width=PLOT_WIDTH, height=PLOT_HEIGHT):
                dpg.add_plot_axis(dpg.mvXAxis, tag="gyr_x", label="Time")
                ay3 = dpg.add_plot_axis(dpg.mvYAxis, tag="gyr_y", label="°/s")
                dpg.add_line_series([], [], label="Wx", parent=ay3, tag="wx_s")
                dpg.add_line_series([], [], label="Wy", parent=ay3, tag="wy_s")
                dpg.add_line_series([], [], label="Wz", parent=ay3, tag="wz_s")
            with dpg.plot(label="Quaternion", width=PLOT_WIDTH, height=PLOT_HEIGHT):
                dpg.add_plot_axis(dpg.mvXAxis, tag="quat_x", label="Time")
                ay4 = dpg.add_plot_axis(dpg.mvYAxis, tag="quat_y", label="Q")
                dpg.add_line_series([], [], label="q0", parent=ay4, tag="q0_s")
                dpg.add_line_series([], [], label="q1", parent=ay4, tag="q1_s")
                dpg.add_line_series([], [], label="q2", parent=ay4, tag="q2_s")
                dpg.add_line_series([], [], label="q3", parent=ay4, tag="q3_s")
            with dpg.plot(label="Gravity XYZ", width=PLOT_WIDTH, height=PLOT_HEIGHT):
                dpg.add_plot_axis(dpg.mvXAxis, tag="grav_x", label="Time")
                ay5 = dpg.add_plot_axis(dpg.mvYAxis, tag="grav_y", label="g")
                dpg.add_line_series([], [], label="g_x", parent=ay5, tag="gx_s")
                dpg.add_line_series([], [], label="g_y", parent=ay5, tag="gy_s")
                dpg.add_line_series([], [], label="g_z", parent=ay5, tag="gz_s")
            dpg.add_spacer(width=PLOT_WIDTH, height=PLOT_HEIGHT)

# show viewport
dpg.create_viewport(title="IMU", width=2500, height=1250)
dpg.setup_dearpygui()
dpg.show_viewport()

# --- Main loop --------------------------------------------------------
ser = open_imu(PORT, BAUD)
start_time = time.time()
iw, ih = PLOT_WIDTH, PLOT_HEIGHT
view_mat = dpg.create_lookat_matrix([0,0,5],[0,0,0],[0,1,0])
proj_mat = dpg.create_perspective_matrix(math.radians(45), iw/ih, 0.1, 100)
dpg.set_clip_space("ori_layer", 0, 0, iw, ih, -1.0, 1.0)
dpg.set_clip_space("grav_layer", 0, 0, iw, ih, -1.0, 1.0)
while dpg.is_dearpygui_running():
    update_buffers(ser, start_time)
    # update time-series
    dpg.set_value("roll_s",  [t_buf, roll_buf])
    dpg.set_value("pitch_s", [t_buf, pitch_buf])
    dpg.set_value("yaw_s",   [t_buf, yaw_buf])
    dpg.set_value("ax_s",    [acc_t_buf, acc_x_buf])
    dpg.set_value("ay_s",    [acc_t_buf, acc_y_buf])
    dpg.set_value("az_s",    [acc_t_buf, acc_z_buf])
    dpg.set_value("wx_s",    [gyro_t_buf, gyro_x_buf])
    dpg.set_value("wy_s",    [gyro_t_buf, gyro_y_buf])
    dpg.set_value("wz_s",    [gyro_t_buf, gyro_z_buf])
    dpg.set_value("q0_s",    [quat_t_buf, quat_q0_buf])
    dpg.set_value("q1_s",    [quat_t_buf, quat_q1_buf])
    dpg.set_value("q2_s",    [quat_t_buf, quat_q2_buf])
    dpg.set_value("q3_s",    [quat_t_buf, quat_q3_buf])
    dpg.set_value("gx_s",    [grav_t_buf, grav_x_buf])
    dpg.set_value("gy_s",    [grav_t_buf, grav_y_buf])
    dpg.set_value("gz_s",    [grav_t_buf, grav_z_buf])

    # axis limits
    if t_buf:
        for tag in ["rpy_x","acc_x","quat_x","gyr_x","grav_x"]:
            dpg.set_axis_limits(tag, t_buf[0], t_buf[-1])
    dpg.set_axis_limits("rpy_y", -180, 180)
    dpg.set_axis_limits("acc_y", -2, 2)
    dpg.set_axis_limits("gyr_y", -500, 500)
    dpg.set_axis_limits("quat_y", -1, 1)
    dpg.set_axis_limits("grav_y", -1, 1)

    # update 3D orientation axes
    if roll_buf:
        rd, pd, yd = roll_buf[-1], pitch_buf[-1], yaw_buf[-1]
        rz = dpg.create_rotation_matrix(math.radians(yd), [0,0,1])
        ry = dpg.create_rotation_matrix(math.radians(pd), [0,1,0])
        rx = dpg.create_rotation_matrix(math.radians(rd), [1,0,0])
        xf_ori = proj_mat * view_mat * (rz * ry * rx)
        for axis in ["axis_x","axis_y","axis_z"]:
            dpg.apply_transform(axis, xf_ori)
    
    # update gravity vector 3D
    if grav_x_buf:
        gx, gy, gz = grav_x_buf[-1], grav_y_buf[-1], grav_z_buf[-1]
        # compute rotation from Z-axis to gravity vector
        v = np.array([gx, gy, gz])
        v = v / np.linalg.norm(v)
        # axis-angle
        axis = np.cross([0,0,1], v)
        if np.linalg.norm(axis) < 1e-6:
            axis = [1,0,0]
        axis = axis / np.linalg.norm(axis)
        angle = math.acos(np.clip(np.dot([0,0,1], v), -1, 1))
        Rg = dpg.create_rotation_matrix(angle, axis.tolist())
        xf_grav = proj_mat * view_mat * Rg
        dpg.apply_transform("gvec", xf_grav)

    dpg.render_dearpygui_frame()

# cleanup
ser.close()
dpg.destroy_context()
