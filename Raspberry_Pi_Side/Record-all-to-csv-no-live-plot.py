# Offline Telemetry Logger
#
# * This script runs on the on-board Raspberry Pi and records binary telemetry frames streamed from the STM32 via
#   ST-Link UART (send_telem_frame_over_STLink()) into a local CSV file for offline analysis. Stop recording by
#   hitting Ctrl+C. This CSV can be fetched from the pi and plotted using "fetch_pi_logs.sh" and "display-offlne-telemetry.py"
#   on the host pc or any machine.
#
# * Running this script conflicts with passing live telemetry using Communicatoy.py telem_thread. So the creation of the latter should be commented
#   out if offline logging using this script is desired.
#
# * A single aggregated telemetry frame (OFFLINE_TELEMETRY_HEADER) is parsed and written to one CSV file containing all
#   relevant data for post-processing and analysis.
# 
# * Old code lines for parsing other telemetry frames (IMU, encoders, Kalman filters, PID data, etc.) are intentionally
#   commented out rather than deleted. This allows users to quickly enable logging of multiple separate CSV files for specific
#   subsets of telemetry data if desired, instead of writing all data to a single CSV.

import serial
import struct
import time
import csv

# ----------------------- CONFIG -----------------------
PORT = '/dev/ttyACM0'  # The STM telemetry output port
BAUD = 115200

# CSV_FILENAME = 'imu_data_binary.csv' # IM data
# ENC_CSV_FILENAME = 'encoder_data.csv' # wheel encoders data
# PITCH_KALMAN_CSV_FILENAME = 'pitch_kalman_data.csv'  # pitch for Kalman data
# YAW_KALMAN_CSV_FILENAME = 'yaw_kalman_data.csv'  # yaw Kalman data
# COMP_FILTER_CSV_FILENAME = 'comp_filter_data.csv'  # complementary filter data
# PITCH_KAL1x1_CSV_FILENAME = 'pitch_kal1x1_data.csv'  # 1x1 Kalman pitch data
# YAW_KAL1x1_CSV_FILENAME = 'yaw_kal1x1_data.csv'  # 1x1 Kalman yaw data
# PITCH_PID_INFO_CSV_FILENAME = 'pitch_pid_info_data.csv'  # pitch pid data
OFFLINE_TELEMETRY_FILENAME = 'Offline_telemetry.csv'

# HEADER = b'\x55\xAA'      # IMU frame sync
# ENC_HEADER = b'\x44\xBB'  # Encoder frame sync
# PITCH_KALMAN_HEADER = b'\x33\xCC'  # Pitch Kalman frame sync
# YAW_KALMAN_HEADER = b'\x22\xDD'  # Yaw Kalman frame sync
# COMP_FILTER_HEADER = b'\x11\xEE'  # Complementary filter frame sync
# PITCH_KAL1x1_HEADER = b'\x66\xAB'  # Pitch Kalman 1x1 frame sync
# YAW_KAL1x1_HEADER = b'\x77\xCD'  # Yaw Kalman 1x1 frame sync
# PITCH_PID_INFO_HEADER = b'\x88\xEF'  # Yaw Kalman 1x1 frame sync
OFFLINE_TELEMETRY_HEADER = b'\x88\xEF'  

ACC_MAX = 16.0  
GYRO_MAX = 2000.0

# Setup serial port
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Wait for MCU reset

# Prepare CSV files
# csv_file = open(CSV_FILENAME, 'w', newline='')
# csv_writer = csv.writer(csv_file)
# csv_writer.writerow(['time', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])

# enc_csv_file = open(ENC_CSV_FILENAME, 'w', newline='')
# enc_csv_writer = csv.writer(enc_csv_file)
# enc_csv_writer.writerow(['time', 'left_enc', 'right_enc', 'xdot', 'thetadot', 'integrated_yaw'])

# pitch_kalman_csv_file = open(PITCH_KALMAN_CSV_FILENAME, 'w', newline='')
# pitch_kalman_csv_writer = csv.writer(pitch_kalman_csv_file)
# pitch_kalman_csv_writer.writerow(['time', 'dt', 'angle', 'bias', 'K0', 'K1', 'P00', 'P11', 'measurement_residual'])

# yaw_kalman_csv_file = open(YAW_KALMAN_CSV_FILENAME, 'w', newline='')
# yaw_kalman_csv_writer = csv.writer(yaw_kalman_csv_file)
# yaw_kalman_csv_writer.writerow(['time', 'dt', 'angle', 'bias', 'K0', 'K1', 'P00', 'P11', 'measurement_residual'])

# comp_filter_csv_file = open(COMP_FILTER_CSV_FILENAME, 'w', newline='')
# comp_filter_csv_writer = csv.writer(comp_filter_csv_file)
# comp_filter_csv_writer.writerow(['time', 'angle'])

# pitch_kal1x1_csv_file = open(PITCH_KAL1x1_CSV_FILENAME, 'w', newline='')
# pitch_kal1x1_csv_writer = csv.writer(pitch_kal1x1_csv_file)
# pitch_kal1x1_csv_writer.writerow(['time', 'dt', 'angle', 'K0', 'P00', 'measurement_residual'])

# yaw_kal1x1_csv_file = open(YAW_KAL1x1_CSV_FILENAME, 'w', newline='')
# yaw_kal1x1_csv_writer = csv.writer(yaw_kal1x1_csv_file)
# yaw_kal1x1_csv_writer.writerow(['time', 'dt', 'angle', 'K0', 'P00', 'measurement_residual'])

# pitch_pid_info_csv_file = open(PITCH_PID_INFO_CSV_FILENAME, 'w', newline='')
# pitch_pid_info_csv_writer = csv.writer(pitch_pid_info_csv_file)
# pitch_pid_info_csv_writer.writerow(['time',
#                                     'pitch_kalman1x1_estimate',
#                                     'imu_gx',
#                                     'u_pitch_angle',
#                                     'chassisData_xdot',
#                                     'chassisData_left_wheel_speed_rad_sec',
#                                     'chassisData_right_wheel_speed_rad_sec',
#                                     'u_speed_hold',
#                                     'pwm_left',
#                                     'pwm_right',
#                                     'u_forward',
#                                     'chassis_psidot',
#                                     'chassis_yaw_angle',
#                                     'yaw_kalman1x1_estimate',
#                                     'imu_gz',
#                                     'yaw_angle_kalman_rate_integration',
#                                     'imu_gy',
#                                     'yaw_rate_world',
#                                     'yaw_angle_IMU_rate_integ_no_kalman',
#                                     'speed_pid_integral',
#                                     'u_turn',
#                                     'pid_update_available',
#                                     'kp_angle',
#                                     'ki_angle',
#                                     'kd_angle',
#                                     'kp_speed',
#                                     'ki_speed',
#                                     'kd_speed',
#                                     #'kp_yaw',    
#                                     #'ki_yaw',    
#                                     'Min_PWM_LeftRight_Forward_Full',
#                                     'Min_PWM_LeftRight_Backward_Full',
#                                     'yaw_pid_integral'])

offline_tele_csv_file = open(OFFLINE_TELEMETRY_FILENAME, 'w', newline='')
offline_tele_csv_writer = csv.writer(offline_tele_csv_file)
offline_tele_csv_writer.writerow(['time',
                                    'pitch_kalman1x1_estimate',
                                    'imu_gx',
                                    'u_pitch_angle',
                                    'chassisData_xdot',
                                    'chassisData_left_wheel_speed_rad_sec',
                                    'chassisData_right_wheel_speed_rad_sec',
                                    'u_speed_hold',
                                    'pwm_left',
                                    'pwm_right',
                                    'u_forward',
                                    'chassis_psidot',
                                    'chassis_yaw_angle',
                                    'yaw_kalman1x1_estimate',
                                    'imu_gz',
                                    'yaw_angle_kalman_rate_integration',
                                    'imu_gy',
                                    'yaw_rate_world',
                                    'yaw_angle_IMU_rate_integ_no_kalman',
                                    'speed_pid_integral',
                                    'u_turn',
                                    'pid_update_available',
                                    'kp_angle',
                                    'ki_angle',
                                    'kd_angle',
                                    'kp_speed',
                                    'ki_speed',
                                    'kd_speed',
                                    #'kp_yaw',    
                                    #'ki_yaw',    
                                    'Min_PWM_LeftRight_Forward_Full',
                                    'Min_PWM_LeftRight_Backward_Full',
                                    'yaw_pid_integral'])

t0 = time.time()

# ----------------------- FUNCTIONS -----------------------
def read_frame():
    """Read one frame (IMU, encoder, or Kalman) from UART and unpack accordingly."""
    while True:
        b = ser.read(1)
        if not b:
            return None, None

        # # IMU frame
        # if b == HEADER[0:1] and ser.read(1) == HEADER[1:2]:
        #     data = ser.read(24)  # 6 floats x 4 bytes
        #     if len(data) == 24:
        #         return 'imu', struct.unpack('<6f', data)
        #     else:
        #         return None, None

        # # Encoder frame
        # elif b == ENC_HEADER[0:1] and ser.read(1) == ENC_HEADER[1:2]:
        #     data = ser.read(20)  # 5 floats x 4 bytes
        #     if len(data) == 20:
        #         return 'enc', struct.unpack('<5f', data)
        #     else:
        #         return None, None

        # # Pitch Kalman frame
        # elif b == PITCH_KALMAN_HEADER[0:1] and ser.read(1) == PITCH_KALMAN_HEADER[1:2]:
        #     data = ser.read(32)  # 8 floats x 4 bytes
        #     if len(data) == 32:
        #         return 'pitch_kalman', struct.unpack('<8f', data)
        #     else:
        #         return None, None
            
        # # Yaw Kalman frame
        # elif b == YAW_KALMAN_HEADER[0:1] and ser.read(1) == YAW_KALMAN_HEADER[1:2]:
        #     data = ser.read(32)  # 8 floats x 4 bytes
        #     if len(data) == 32:
        #         return 'yaw_kalman', struct.unpack('<8f', data)
        #     else:
        #         return None, None
        
        # # Complementary filter frame
        # elif b == COMP_FILTER_HEADER[0:1] and ser.read(1) == COMP_FILTER_HEADER[1:2]:
        #     data = ser.read(4)  # 1 floats x 4 bytes
        #     if len(data) == 4:
        #         return 'comp_filter', struct.unpack('<1f', data)
        #     else:
        #         return None, None
        
        # # Pitch Kalman 1x1 frame
        # elif b == PITCH_KAL1x1_HEADER[0:1] and ser.read(1) == PITCH_KAL1x1_HEADER[1:2]:
        #     data = ser.read(20)  # 5 floats x 4 bytes
        #     if len(data) == 20:
        #         return 'pitch_kal1x1', struct.unpack('<5f', data)
        #     else:
        #         return None, None
            
        # # Yaw Kalman 1x1 frame
        # elif b == YAW_KAL1x1_HEADER[0:1] and ser.read(1) == YAW_KAL1x1_HEADER[1:2]:
        #     data = ser.read(20)  # 5 floats x 4 bytes
        #     if len(data) == 20:
        #         return 'yaw_kal1x1', struct.unpack('<5f', data)
        #     else:
        #         return None, None
            
        # # Pitch PID
        # elif b == PITCH_PID_INFO_HEADER[0:1] and ser.read(1) == PITCH_PID_INFO_HEADER[1:2]:
        #     data = ser.read(120)  # 30 floats x 4 bytes
        #     if len(data) == 120:
        #         return 'pitch_pid_info', struct.unpack('<30f', data)
        #     else:
        #         return None, None
            
        # Pitch PID
        elif b == OFFLINE_TELEMETRY_HEADER[0:1] and ser.read(1) == OFFLINE_TELEMETRY_HEADER[1:2]:
            data = ser.read(120)  # 30 floats x 4 bytes
            if len(data) == 120:
                return 'offline_tele', struct.unpack('<30f', data)
            else:
                return None, None
            
            

# ----------------------- MAIN LOOP -----------------------
print("Logging started. Press Ctrl+C to stop.")
try:
    while True:
        ftype, result = read_frame()
        if not result:
            continue

        t = time.time() - t0

        # if ftype == 'imu':
        #     csv_writer.writerow([t] + list(result))
        #     csv_file.flush()
        # elif ftype == 'enc':
        #     enc_csv_writer.writerow([t] + list(result))
        #     enc_csv_file.flush()
        # elif ftype == 'pitch_kalman':
        #     pitch_kalman_csv_writer.writerow([t] + list(result))
        #     pitch_kalman_csv_file.flush()
        # elif ftype == 'yaw_kalman':
        #     yaw_kalman_csv_writer.writerow([t] + list(result))
        #     yaw_kalman_csv_file.flush()
        # elif ftype == 'comp_filter':
        #     comp_filter_csv_writer.writerow([t] + list(result))
        #     comp_filter_csv_file.flush()
        # elif ftype == 'pitch_kal1x1':
        #     pitch_kal1x1_csv_writer.writerow([t] + list(result))
        #     pitch_kal1x1_csv_file.flush()
        # elif ftype == 'yaw_kal1x1':
        #     yaw_kal1x1_csv_writer.writerow([t] + list(result))
        #     yaw_kal1x1_csv_file.flush()
        # elif ftype == 'pitch_pid_info':
        #     pitch_pid_info_csv_writer.writerow([t] + list(result))
        #     pitch_pid_info_csv_file.flush()
        if ftype == 'offline_tele':
            offline_tele_csv_writer.writerow([t] + list(result))
            offline_tele_csv_file.flush()

except KeyboardInterrupt:
    print("\nLogging stopped by user.")

finally:
    ser.close()
    # csv_file.close()
    # enc_csv_file.close()
    # pitch_kalman_csv_file.close()
    # yaw_kalman_csv_file.close()
    # comp_filter_csv_file.close()
    # pitch_kal1x1_csv_file.close()
    # yaw_kal1x1_csv_file.close()
    # pitch_pid_info_csv_file.close()
    offline_tele_csv_file.close()
    print("Serial port and files closed.")
