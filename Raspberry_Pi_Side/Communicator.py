# =============================================================================
# Raspberry Pi ↔ STM32 Communicator
# =============================================================================
#
# This script runs on the on-board raspberry pi. Configure it to start automatically on power up using communicatorService.service
# This is a multithreading script, whose purpose is to communicate essential data with the on-board STM MCU.
# The MCU is offloaded the task of running the balance control loop. The intention of this is to free computation
# resources on the Pi, and have it function as main computer that can perform heavier AI related tasks, without 
# having compromise between fast control of fast processing of other tasks.
# 
# The main purpose of this program, and the source code that it was initially was based on, is SCUTTLE's L2_kinematics.py.
# Here's its description from SCUTTLE's GitHub:
# "The program takes the encoder values from encoders, computes wheel movement
# and computes the movement of the wheelbase center based on SCUTTLE kinematics."
# It uses SCUTTLE's L1_encoder.py that interfaces with the wheel encoder hardware.
# In simpler words: Read wheel encoders data in [rad/sec], use transformation matrices to get chassis speed and psi_dot (yaw angle change rate).
# This script modifies the original program to send these kinematic parameters through serial to the on-board STM32 MCU.
# 
# In addition, this program creates 3 additional threads, which run in parallel and in synchronization with the main loop and each other. 
# These threads handle tasks that are independent of the main control loop:
#
# 1. PID tuning listener thread: Receives updated Tuning data, mainly used for PID gains but not exclusively, over UDP from a remote PC.
#    These values are stored safely in a shared variable protected by a lock, and are passed over UART1 (via jumper wires) to
#    the STM32 MCU in constant rate PID_PERIOD defined by the user. This rate should not be too fast, and should not intervene with the
#    continuous smooth transmission of essential kinematics payloads to the MCU.
#  
# 2. Live telemetry forwarder thread: Reads telemetry frames from the STM MCU over UART2 (USB cable), extracts relevant data 
#    (e.g., PWM outputs, pitch angle, wheel speed, etc), and forwards it via UDP to a PC for live plotting using "live_plot_pwm-pyqtgraph.py"
#    * Creating this thread currently interferes with recording the telemetry to a CSV file using "record-all-to-csv-no-live-plot". Choose
#      between which one to use by placing the creation of this thread in/out of comment.
#
# 3. Operator command listener_thread: Continuously monitors a named FIFO for commands sent from other processes or scripts. 
#    * When a command is received, it updates a global variable for the main loop to read and act upon.
#    * Operator commands may be for example setting temporary speed / yaw setpoints ("routes"),
#      commanding wheel rotations for tests, zeroing PID accumulated integrals, or any other functionality required.
#    * Typical usage: Host PC --> SSH --> On-board main computer (Raspberry Pi) command line --> echo a command
#      on the locam cmd_fifo named FIFO --> Send to STM --> STM recognizes and takes action
#    * Requires the creation of a named FIFO and assigning its path to variable COMMAND_FIFO
#    
# All 3 threads are optional:
# The PID tuning listener thread is used for testing and adjustment. It can be disabled once balance is achieved and gains are fixed.
# The live telemetry forwarder thread can be disabled without affecting robot operation.
# The operator command listener thread can be disabled without affecting robot operation.
#
# The main loop of the program is synchronized with the STM32 MCU. it waits for a READY_BYTE from
# it, to ensure the previous frame was processed.  


try:
    import numpy as np                          
    import time                                 
    import serial
    import struct
    import threading
    import select
    import os
    import socket
    import L1_encoder as enc                    
    import sys


    
    # ---------------- UART setup ----------------
    PORT = '/dev/serial0'
    BAUD = 115200
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(4)  # allow UART to initialize
    SYNC_HEADER = b'\x44\xBB'  # Header for chassis kinematics data payload
    PID_HEADER = b'\xA9\xC9'   # Header for PID tuning payload
    READY_BYTE = b'\xD4'       # STM sends when ready to receive
    PID_PERIOD = 0.3           # Period at which PID payload is sent.
                               # Slow enough to not intervene with essential continuous kinematic payloads to the MCU

    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)

    # ---------------- FIFO setup ----------------
    COMMAND_FIFO = "/home/barak/cmd_fifo"
    last_command = None  # Updated by the 'command listener thread' with the command identifer sent by an operator

    def crc8(data: bytes) -> int:
        crc = 0
        for b in data:
            crc ^= b
        return crc & 0xFF


    def command_listener():
        """Thread: listens for commands from FIFO without blocking."""
        global last_command

        with open(COMMAND_FIFO, "r") as fifo:
            while True:
                r, _, _ = select.select([fifo], [], [], 0.1)
                if fifo in r:
                    line = fifo.readline().strip()
                    if line:
                        last_command = line
                        print(f"Received command: {last_command}")

    # Start listener thread
    listener_thread = threading.Thread(target=command_listener, daemon=True)
    listener_thread.start()


    # ---------------- UDP PID LISTENER ----------------
    UDP_IP = "10.100.102.53"  # Raspberry pi
    UDP_PORT = 5005              

    latest_pid_values = None    # Updated by the 'PID listener thread'
    pid_lock = threading.Lock()
    

    def pid_listener():
        """Thread that receives 8 PID floats via UDP."""
        global latest_pid_values

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        print(f"[PID] Listening on UDP {UDP_IP}:{UDP_PORT}")

        sock.settimeout(1.0) 
        last_heartbeat = time.monotonic()  # time.time() jumps

        while True:

            now = time.monotonic()
            if now - last_heartbeat >= 5.0:  # Optional (debugging): heartbeat every 5 seconds
                #print("[PID] Listener still running...")
                last_heartbeat = now

            try:
                data, addr = sock.recvfrom(1024)
                #print(f"[PID] Raw UDP data received from {addr}: {data}")
                msg = data.decode("utf-8").strip()
                values = list(map(float, msg.split(",")))

                if len(values) != 8:
                    print(f"[PID] Invalid payload length {len(values)}: {msg}")
                    continue
                
                with pid_lock:
                    latest_pid_values = values

                #print(f"[PID] Received from {addr}: {values}")

            except socket.timeout:
                print("socket timeout")
                pass  # no packet received this loop

    pid_thread = threading.Thread(target=pid_listener, daemon=True)
    pid_thread.start()


    # ---------------- DEFINE KINEMATICS (SCUTTLE code) ----------------
    R = 0.041                                   # wheel radius (meters)
    L = 0.213                                   # half of wheelbase (meters)
    res = (360/2**14)                           # resolution of the encoders (degrees per LSB)
    pulleyRatio = 0.5                           # wheel movement per shaft movement
    A = np.array([[R/2, R/2], [-R/(2*L), R/(2*L)]])     # This matrix relates [PDL, PDR] to [XD,TD]
    wait = 0.02                                 # wait time between encoder measurements (s)


    # Note:  this function takes at least 5.1ms plus "wait" to run.  It also populates a global
    # variable so programs can access the previous measurement instantaneously.
    def getPdCurrent():
        global pdCurrents                       # make a global var for easy retrieval
        encoders_t1 = enc.readShaftPositions()  # grabs the current encoder readings in degrees
        t1 = time.monotonic()                   # time.monotonic() reports in seconds
        time.sleep(wait)                        # delay for the specified amount
        
        encoders_t2 = enc.readShaftPositions()  # grabs the current encoder readings in degrees
        t2 = time.monotonic()                   # usually takes about .003 seconds gap
        global deltaT
        deltaT = round((t2 - t1), 3)            # compute delta-time (t.ttt scalar)

        # calculate travel of both wheels simultaneously
        travel = encoders_t2 - encoders_t1      # compute change in both shaft encoders (degrees)
        travel = encoders_t2 - encoders_t1      # array, 2x1 to indicate travel
        trav_b = travel + 360                   # array variant b
        trav_c = travel - 360                   # array variant c
        mx = np.stack((travel, trav_b, trav_c)) # combine array variants
        mx_abs = np.absolute(mx)                # convert to absolute val
        mins = np.argmin(mx_abs,0)              # find the indices of minimum values (left and right hand)
        left = mx[mins[0],0]                    # pull corresponding indices from original array
        right = mx[mins[1],1]                   # pull corresponding index for RH
        shaftTravel = np.array([left,right])    # combine left and right sides to describe travel (degrees)
        
        # build an array of wheel speeds in rad/s
        wheelTravel = shaftTravel * pulleyRatio     # compute wheel turns from motor turns
        wheelSpeeds_deg = wheelTravel / deltaT      # compute wheel speeds (degrees/s)
        pdCurrents = wheelSpeeds_deg * np.pi / 180  # compute wheel speeds (rad/s) & store to global variable
        return(pdCurrents)                          # returns [pdl, pdr] in radians/second


    # ---------------- TELEMETRY → UDP FORWARDER ----------------
    TELEM_PORT = '/dev/ttyACM0'     # The STM telemetry output port
    TELEM_BAUD = 115200
    TELEM_HEADER = b'\x88\xEF'
    TELEM_PAYLOAD_SIZE = 30 * 4     # 120 bytes

    PC_IP = "10.100.102.55"         # Destination machine for live plotting
    PC_PORT = 6000                  

    def telemetry_listener():
        """Reads 30-float telemetry frames from STM32 and forwards selected values."""
        import serial
        telem_ser = serial.Serial(TELEM_PORT, TELEM_BAUD, timeout=1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print("[TELEM] Listening on", TELEM_PORT)

        target_speed = 0.0
        target_pitch_angle = 0.0

        while True:
            b = telem_ser.read(1)
            if not b:
                continue

            # detect header byte 1
            if b != TELEM_HEADER[0:1]:
                continue
            # detect header byte 2
            b2 = telem_ser.read(1)
            if b2 != TELEM_HEADER[1:2]:
                continue

            # read full payload (30 floats)
            data = telem_ser.read(TELEM_PAYLOAD_SIZE)
            if len(data) != TELEM_PAYLOAD_SIZE:
                continue

            values = struct.unpack("<30f", data)

            # --- Extract selected fields ---
            pwm_left = values[7]
            pwm_right = values[8]
            speed_pid_integral = values[18]            
            pitch_angle_kalman = values[0]
            gx = values[1]
            u_pitch_angle = values[2]
            xdot = values[3]
            u_speed_hold = values[6]
                     
            with pid_lock:
                if latest_pid_values is not None:
                    KP_ANGLE = latest_pid_values[0]
                    KD_ANGLE = latest_pid_values[2]
                    KP_SPEED = latest_pid_values[3]
                    KI_SPEED = latest_pid_values[4]
                else:
                    KP_ANGLE = 0.0                    # fallback value
                    KD_ANGLE = 0.0
                    KP_SPEED = 0.0
                    KI_SPEED = 0.0

                        
            u_speed_hold_kp_term = KP_SPEED * (target_speed - xdot)
            u_speed_hold_ki_term = KI_SPEED * speed_pid_integral
            #u_speed_hold_kd_term = u_speed_hold - u_speed_hold_ki_term - u_speed_hold_kp_term

            # Pitch control components
            targetPitch_uSpeedHold = target_pitch_angle + u_speed_hold
            #targetPitch_uSpeedHold = target_pitch_angle - u_speed_hold

            u_pitch_angle_kp_term_cascade = KP_ANGLE * (pitch_angle_kalman - targetPitch_uSpeedHold)
            #u_pitch_angle_kp_term_superposition = KP_ANGLE * (pitch_angle_kalman - target_pitch_angle)

            u_pitch_angle_kp_term_kalman_contribution = KP_ANGLE * target_pitch_angle
            u_pitch_angle_kp_term_cascade_contribution = -KP_ANGLE * targetPitch_uSpeedHold
            u_pitch_angle_kd_term = KD_ANGLE * gx

            # Pack and send
            msg = struct.pack("<16f", pwm_left, pwm_right, speed_pid_integral, KP_ANGLE, KD_ANGLE, KP_SPEED, KI_SPEED, u_pitch_angle, u_pitch_angle_kd_term, u_pitch_angle_kp_term_cascade, u_pitch_angle_kp_term_cascade_contribution, u_pitch_angle_kp_term_kalman_contribution, u_speed_hold_kp_term, u_speed_hold_ki_term, pitch_angle_kalman, gx)
            sock.sendto(msg, (PC_IP, PC_PORT))

    #Interferes with recording the telemetry to a CSV file using "record-all-to-csv-no-live-plot". Choose between which one to use by placing the creation of this thread in/out of comment.
    telem_thread = threading.Thread(target=telemetry_listener, daemon=True)
    telem_thread.start()

    def wait_for_ready():
        """Block until STM32 sends READY_BYTE."""
        while True:
            byte = ser.read(1)     # blocks until byte arrives
            if byte == READY_BYTE:
                return
            # ignore everything else



    if __name__ == "__main__":
    
        print("Start message")
        last_pid_send = 0
        last_pid_values_to_send = None
        first_loop = True
        time.sleep(2)

        while True:
            
            # --- SYNC WITH STM32 ---
            if not first_loop:
                # Wait for STM to signal it's ready
                while True:
                    if ser.in_waiting > 0:
                        byte = ser.read(1)
                        if byte == READY_BYTE:
                            break
            else:
                first_loop = False  # skip waiting on the first iteration
            
            now = time.time()
            send_pid = False

            # Fetch PID values safely
            with pid_lock:
                if latest_pid_values is not None:
                    last_pid_values_to_send = latest_pid_values.copy()

            # Determine if it's time to send PID
            if last_pid_values_to_send is not None and (now - last_pid_send) >= PID_PERIOD:
                send_pid = True

            if send_pid:
                # Send PID tuning data
                p_rounded = [round(v, 3) for v in last_pid_values_to_send]
                pid_payload = struct.pack("<8f", *p_rounded)  # Largest payload to be packed. Determines size of all transmitted packets

                frame_wo_crc = PID_HEADER + pid_payload
                crc = crc8(frame_wo_crc)

                print("Sending PID frame")
                ser.write(frame_wo_crc + bytes([crc]))
                ser.flush()
                last_pid_send = now

            else:
                # Send chassis data
                B = getPdCurrent()                      # store phidots to array B (here still in rad/s)
                C = np.matmul(A, B)                     # perform matrix multiplication
                C = np.round(C, decimals=3)             # round the matrix. C should now contain the chassis speeds - [xDot, thetaDot]
                
                left_wheel_speed_rad_sec, right_wheel_speed_rad_sec = B
                xdot, thetadot = C            # in m/s and rad/s

                # Pack 4 floats + 4 dummy float. Padding to meet the maximal message length of 8 floats
                payload = struct.pack('<8f', left_wheel_speed_rad_sec, right_wheel_speed_rad_sec, xdot, thetadot, 0.0, 0.0, 0.0, 0.0)
                frame_wo_crc = SYNC_HEADER + payload
                crc = crc8(frame_wo_crc)
                #print("Sending payload (CHASSIS DATA)")
                ser.write(frame_wo_crc + bytes([crc]))
                ser.flush()
                #print("[CHASSIS] Sent to STM32:", payload)


            # ----- check FIFO command -----
            if last_command is not None:
                try:
                    cmd_value = int(last_command)

                    # ---------------------------
                    # Command → header map
                    # ---------------------------
                    if cmd_value == 6:
                        special_header = b'\xA6\xC6'
                    elif cmd_value == 8:
                        special_header = b'\xB6\xD6'
                    else:
                        print(f"Unknown command {cmd_value}, ignored.")
                        special_header = None

                    if special_header:
                        # full packet: header + 5 zero floats
                        zero_payload = struct.pack('<5f', 0.0, 0.0, 0.0, 0.0, 0.0).ljust(32, b'\x00') # Padding to meet the maximal message length of 8 floats
                        
                        frame_wo_crc = special_header + zero_payload
                        crc = crc8(frame_wo_crc)

                        print("Sending special header")
                        ser.write(frame_wo_crc + bytes([crc]))
                        ser.flush()

                        print(f"Sent special command packet for {cmd_value}: "
                              f"{special_header.hex()} + 5 float zeros")

                except ValueError:
                    print(f"Ignoring invalid command: {last_command}")

                last_command = None  # reset after handling



except Exception as e:
    with open("/home/barak/barak.err", "a") as f:
        import traceback
        f.write(traceback.format_exc() + "\n")
    raise