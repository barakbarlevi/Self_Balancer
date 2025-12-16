import socket
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
from pyqtgraph.Qt import QtGui   # <-- ONLY for QFont

import time

UDP_IP = "0.0.0.0"
UDP_PORT = 6000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.01) 

# Data lists
pwm_l = []
pwm_r = []
pid_i = []
KP_ANGLE = []
KD_ANGLE = []
KP_SPEED = []
KI_SPEED = []
u_pitch_angle = []
u_pitch_angle_kd_term = []
u_pitch_angle_kp_term_cascade = []
u_pitch_angle_kp_term_cascade_contribution = []
u_pitch_angle_kp_term_kalman_contribution = []
u_speed_hold_kp_term = []
u_speed_hold_ki_term = []
pitch_angle_kalman = []
gx = []

tvals = []

# PyQtGraph setup
app = QtWidgets.QApplication([])

win1 = pg.GraphicsLayoutWidget(show=True, title="PWM + Integral")
plot1 = win1.addPlot(title="PWM Signals")
plot1.addLegend()
plot1.getAxis('left').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot1.getAxis('bottom').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot1.setLabel('left', 'PWM', **{'font-size': '18pt'})
plot1.setLabel('bottom', 'Time (s)', **{'font-size': '18pt'})
plot1.setTitle("PWM Left, Right, PID Integral", size="20pt")
plot1.showGrid(x=True, y=True, alpha=0.3)
ticks_1100 = [(i, str(i)) for i in range(-1100, 1101, 100)]
plot1.getAxis('left').setTicks([ticks_1100])

curve_l = plot1.plot(pen='r', name="PWM Left")
curve_r = plot1.plot(pen='magenta', name="PWM Right")
curve_i = plot1.plot(pen='g', name="Speed PID Integral")

plot1.setYRange(-1100, 1100)

win2 = pg.GraphicsLayoutWidget(show=True, title="PID Gains")
plot2 = win2.addPlot(title="PID Gain Values")
plot2.addLegend()
plot2.getAxis('left').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot2.getAxis('bottom').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot2.setLabel('left', 'Gains', **{'font-size': '18pt'})
plot2.setLabel('bottom', 'Time (s)', **{'font-size': '18pt'})
plot2.showGrid(x=True, y=True, alpha=0.3)
plot2.setTitle("Angle + Speed Gains", size="20pt")

curve_KP_ANGLE = plot2.plot(pen=pg.mkPen('y', width=2), name="KP_ANGLE")
curve_KD_ANGLE = plot2.plot(pen=pg.mkPen('c', width=2), name="KD_ANGLE")
curve_KP_SPEED = plot2.plot(pen=pg.mkPen('m', width=2), name="KP_SPEED")
curve_KI_SPEED = plot2.plot(pen=pg.mkPen('w', width=2), name="KI_SPEED")

win3 = pg.GraphicsLayoutWidget(show=True, title="Control Terms part 1")
plot3 = win3.addPlot(title="Control Contributions")
plot3.addLegend()
plot3.getAxis('left').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot3.getAxis('bottom').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot3.setLabel('left', 'Control Terms', **{'font-size': '18pt'})
plot3.setLabel('bottom', 'Time (s)', **{'font-size': '18pt'})
plot3.setYRange(-1100, 1100)
plot3.setTitle("Pitch + Speed Control Terms", size="20pt")
plot3.showGrid(x=True, y=True, alpha=0.3)
ticks_1100 = [(i, str(i)) for i in range(-1100, 1101, 100)]
plot3.getAxis('left').setTicks([ticks_1100])

curve_u_pitch_angle = plot3.plot(pen=pg.mkPen('#0000ff', width=2), name="u_pitch_angle")
curve_u_pitch_angle_kd_term = plot3.plot(pen=pg.mkPen('orange', width=2), name="u_pitch_angle_kd_term")
curve_u_pitch_angle_kp_term_cascade_plot3 = plot3.plot(pen=pg.mkPen('#ff77aa', width=2), name="u_pitch_angle_kp_term_cascade")
#curve_u_pitch_angle_kp_term_cascade_contribution = plot3.plot(pen=pg.mkPen('#33bbbb', width=2), name="u_pitch_angle_kp_term_cascade_contribution")

#curve_u_pitch_angle_kp_term_kalman_contribution = plot3.plot(pen=pg.mkPen('#aa55ff', width=2), name="u_pitch_angle_kp_term_kalman_contribution")


win4 = pg.GraphicsLayoutWidget(show=True, title="Control Terms")
plot4 = win4.addPlot(title="Control Contributions part 2")
plot4.addLegend()
plot4.getAxis('left').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot4.getAxis('bottom').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot4.setLabel('left', 'Control Terms', **{'font-size': '18pt'})
plot4.setLabel('bottom', 'Time (s)', **{'font-size': '18pt'})
plot4.setYRange(-1100, 1100)
plot4.setTitle("Speed Control Terms", size="20pt")
plot4.showGrid(x=True, y=True, alpha=0.3)
ticks_1100 = [(i, str(i)) for i in range(-1100, 1101, 100)]
plot4.getAxis('left').setTicks([ticks_1100])
curve_u_pitch_angle_kp_term_cascade_plot4 = plot4.plot(pen=pg.mkPen('#ff77aa', width=2), name="u_pitch_angle_kp_term_cascade")
curve_u_pitch_angle_kp_term_cascade_contribution = plot4.plot(pen=pg.mkPen('#33bbbb', width=2), name="u_pitch_angle_kp_term_cascade_contribution")


win5 = pg.GraphicsLayoutWidget(show=True, title="Control Terms")
plot5 = win5.addPlot(title="pitch angle and rate")
plot5.addLegend()
plot5.getAxis('left').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot5.getAxis('bottom').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot5.setLabel('left', 'pitch angle and rate', **{'font-size': '18pt'})
plot5.setLabel('bottom', 'Time (s)', **{'font-size': '18pt'})
plot5.setYRange(-6, 6)
plot5.setTitle("Pitch angle and rate", size="20pt")
plot5.showGrid(x=True, y=True, alpha=0.3)
curve_pitch_angle_kalman = plot5.plot(pen=pg.mkPen('cyan', width=2), name="pitch_angle_kalman")
curve_gx = plot5.plot(pen=pg.mkPen('orange', width=2), name="gx")

win6 = pg.GraphicsLayoutWidget(show=True, title="Two speed terms")
plot6 = win6.addPlot(title="Two speed terms")
plot6.addLegend()
plot6.getAxis('left').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot6.getAxis('bottom').setStyle(tickFont=QtGui.QFont("Arial", 16))
plot6.setLabel('left', 'pitch angle and rate', **{'font-size': '18pt'})
plot6.setLabel('bottom', 'Time (s)', **{'font-size': '18pt'})
plot6.setYRange(-1.5, 1.5)
plot6.setTitle("Two speed terms", size="20pt")
plot6.showGrid(x=True, y=True, alpha=0.3)
curve_u_speed_hold_kp_term = plot6.plot(pen=pg.mkPen('#55ff55', width=2), name="u_speed_hold_kp_term")
curve_u_speed_hold_ki_term = plot6.plot(pen=pg.mkPen('#ff5533', width=2), name="u_speed_hold_ki_term")

t0 = time.time()

def update():

    # Read all available UDP packets
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            #left, right = struct.unpack("<2f", data) # Old. Use if want to pass less telemetry data
            left, right, integral, KP_ANGLE_to_append, KD_ANGLE_to_append, KP_SPEED_to_append, KI_SPEED_to_append, u_pitch_angle_to_append, u_pitch_angle_kd_term_to_append, u_pitch_angle_kp_term_cascade_to_append, u_pitch_angle_kp_term_cascade_contribution_to_append, u_pitch_angle_kp_term_kalman_contribution_to_append, u_speed_hold_kp_term_to_append, u_speed_hold_ki_term_to_append, pitch_angle_kalman_to_append, gx_to_append= struct.unpack("<16f", data)
            integral = integral * 100

            t = time.time() - t0
            tvals.append(t)
            pwm_l.append(left)
            pwm_r.append(right)
            pid_i.append(integral)
            KP_ANGLE.append(KP_ANGLE_to_append)
            KD_ANGLE.append(KD_ANGLE_to_append)
            KP_SPEED.append(KP_SPEED_to_append)
            KI_SPEED.append(KI_SPEED_to_append)
            u_pitch_angle.append(u_pitch_angle_to_append)
            u_pitch_angle_kd_term.append(u_pitch_angle_kd_term_to_append)
            u_pitch_angle_kp_term_cascade.append(u_pitch_angle_kp_term_cascade_to_append)
            u_pitch_angle_kp_term_cascade_contribution.append(u_pitch_angle_kp_term_cascade_contribution_to_append)
            u_pitch_angle_kp_term_kalman_contribution.append(u_pitch_angle_kp_term_kalman_contribution_to_append)
            u_speed_hold_kp_term.append(u_speed_hold_kp_term_to_append)
            u_speed_hold_ki_term.append(u_speed_hold_ki_term_to_append)
            pitch_angle_kalman.append(pitch_angle_kalman_to_append)
            gx.append(gx_to_append)

        except socket.timeout:
            break  # no more packets

    # Update the curves
    curve_l.setData(tvals, pwm_l)
    curve_r.setData(tvals, pwm_r)
    curve_i.setData(tvals, pid_i)

    curve_KP_ANGLE.setData(tvals, KP_ANGLE)
    curve_KD_ANGLE.setData(tvals, KD_ANGLE)
    curve_KP_SPEED.setData(tvals, KP_SPEED)
    curve_KI_SPEED.setData(tvals, KI_SPEED)

    curve_u_pitch_angle.setData(tvals, u_pitch_angle)
    curve_u_pitch_angle_kd_term.setData(tvals, u_pitch_angle_kd_term)
    curve_u_pitch_angle_kp_term_cascade_plot3.setData(tvals, u_pitch_angle_kp_term_cascade)
    curve_u_pitch_angle_kp_term_cascade_plot4.setData(tvals, u_pitch_angle_kp_term_cascade)
    curve_u_pitch_angle_kp_term_cascade_contribution.setData(tvals, u_pitch_angle_kp_term_cascade_contribution)

    # curve_u_pitch_angle_kp_term_kalman_contribution.setData(tvals, u_pitch_angle_kp_term_kalman_contribution)

    curve_pitch_angle_kalman.setData(tvals, pitch_angle_kalman)
    curve_gx.setData(tvals, gx)

    curve_u_speed_hold_kp_term.setData(tvals, u_speed_hold_kp_term)
    curve_u_speed_hold_ki_term.setData(tvals, u_speed_hold_ki_term)
    

# Use a Qt timer to update the plot repeatedly
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # update every 50 ms

# Start Qt event loop
QtWidgets.QApplication.instance().exec()
