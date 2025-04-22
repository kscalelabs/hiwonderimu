#!/usr/bin/env python3
import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import math

# adjust if needed
PORT = "/dev/ttyUSB0"
BAUD = 9600

# conversion factors from the datasheet:
#   Accel: ±16g → raw/32768*16 g      :contentReference[oaicite:0]{index=0}&#8203;:contentReference[oaicite:1]{index=1}
#   Gyro: ±2000°/s → raw/32768*2000 °/s&#8203;:contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}
#   Angle: ±180° → raw/32768*180°     :contentReference[oaicite:4]{index=4}&#8203;:contentReference[oaicite:5]{index=5}
SCALE_ACC   = 16.0   / 32768.0
SCALE_GYRO  = 2000.0 / 32768.0
SCALE_ANGLE = 180.0  / 32768.0

# Global variable to store the latest angles (roll, pitch, yaw) in degrees
latest_angles = [0.0, 0.0, 0.0]

def open_imu(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser

# --- Rotation Matrix Helper ---
def euler_to_rotation_matrix(roll, pitch, yaw):
    """ Convert Euler angles (in radians) to a 3x3 rotation matrix (ZYX convention). """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    # ZYX Convention: R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return R

# --- Matplotlib Plotting ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('IMU Orientation')

# Initial axis vectors
origin = np.array([0, 0, 0])
axis_x = np.array([1, 0, 0])
axis_y = np.array([0, 1, 0])
axis_z = np.array([0, 0, 1])

# Create quiver objects (these will be updated)
# Storing them is tricky for updates; we'll redraw instead.

def update_plot(frame, ser):
    """ Reads data from serial and updates the plot. """
    global latest_angles
    new_data = False

    # Try to read a packet
    if ser.in_waiting >= 11: # Check if enough bytes for a packet might be present
        if ser.read(1) == b'U':
            pkt_type = ser.read(1)
            if pkt_type and pkt_type[0] == 0x53: # Check for Euler angles packet
                pkt = ser.read(9)
                if len(pkt) == 9:
                    payload, chk = pkt[:8], pkt[8]
                    # checksum: low‑8 bits of sum(0x55 + TYPE + payload)
                    if ((0x55 + 0x53 + sum(payload)) & 0xFF) == chk:
                        # Euler Angles: Roll Pitch Yaw + version
                        RL, RH, PL, PH, YL, YH, VL, VH = payload
                        roll_raw = struct.unpack("<h", bytes([RL, RH]))[0]
                        pitch_raw = struct.unpack("<h", bytes([PL, PH]))[0]
                        yaw_raw = struct.unpack("<h", bytes([YL, YH]))[0]

                        latest_angles[0] = roll_raw * SCALE_ANGLE
                        latest_angles[1] = pitch_raw * SCALE_ANGLE
                        # Yaw might need adjustment depending on IMU coordinate system
                        # and desired visualization mapping.
                        latest_angles[2] = yaw_raw * SCALE_ANGLE
                        new_data = True
            else:
                # Discard the rest of a potential packet if type is not 0x53
                ser.read(9)
        # If header is wrong, the byte is consumed, loop continues

    # If new data was received, update the plot
    if new_data:
        ax.cla() # Clear previous frame

        roll_rad = math.radians(latest_angles[0])
        pitch_rad = math.radians(latest_angles[1])
        yaw_rad = math.radians(latest_angles[2])

        R = euler_to_rotation_matrix(roll_rad, pitch_rad, yaw_rad)

        # Rotate axes
        rotated_x = R @ axis_x
        rotated_y = R @ axis_y
        rotated_z = R @ axis_z

        # Draw rotated axes
        ax.quiver(*origin, *rotated_x, color='r', length=0.8, normalize=True)
        ax.quiver(*origin, *rotated_y, color='g', length=0.8, normalize=True)
        ax.quiver(*origin, *rotated_z, color='b', length=0.8, normalize=True)

        # Reset limits and labels after clearing
        ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title(f'IMU Orientation Roll: {latest_angles[0]:.1f}°, Pitch: {latest_angles[1]:.1f}°, Yaw: {latest_angles[2]:.1f}°')

    # Required for FuncAnimation: return the artists that were updated (none here as we use cla())
    return []


# Remove the old read_stream function as its logic is now inside update_plot
# def read_stream(ser): ... (delete this)


if __name__ == "__main__":
    try:
        print(f"Opening serial port {PORT}...")
        ser = open_imu(PORT, BAUD)
        print("Serial port opened. Starting visualization...")

        # Set up animation
        # interval is the delay between frames in milliseconds
        ani = FuncAnimation(fig, update_plot, fargs=(ser,), interval=20, blit=False, cache_frame_data=False)

        plt.show() # Display the plot

    except serial.SerialException as e:
        print(f"Could not open serial port {PORT}: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

# Remove old packet parsing logic if it remained from previous read_stream
# ... (ensure old print statements for accel, gyro etc are removed)
