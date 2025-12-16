# This script produces two plots:
#
# 1. The Controller Terms Graph. Intended for close up looks on the terms that make up the PID controllers outputs, including in the cascaded case.
# It requires FIXED PID gains, so that the operator can have a fine details look on the terms that make up the control.
# Therefore, this script asks for gain values or other constant values. 
# If no such fine detail analysis is needed, one can input random values which will have no importance, and plot:
#
# 2. The combined plot, which shows all telemetry values as sent by the STM. This includes varying PID gains if the device is under
# adjustments and is receiving them from the tuning station.

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import mplcursors
import sys

# ---------------- CONFIG ----------------
CSV_FILENAME = 'Offline_telemetry.csv'
TIME_COLUMN = 'time'
THRESHOLD = 1001                       # Ignore rows with |value| > this
SAVE_DIR = '.'                         # Directory to save figures
# ---------------------------------------

if len(sys.argv) != 9:
    print("Usage: python3 display-pitch_pid_info-data.py kp_angle ki_angle kd_angle kp_speed ki_speed kd_speed factor_angle factor_speed")
    sys.exit(1)


# Create save directory if needed
os.makedirs(SAVE_DIR, exist_ok=True)

# Load data
data = pd.read_csv(CSV_FILENAME)

# data contains the 'time' + fields that the STM sends in send_telem_frame_over_STLink():
#['time',
# 'pitch_kalman1x1_estimate',
# 'imu_gx',
# 'u_pitch_angle',
# 'chassisData_xdot',
# 'chassisData_left_wheel_speed_rad_sec',
# 'chassisData_right_wheel_speed_rad_sec',
# 'u_speed_hold',
# 'pwm_left',
# 'pwm_right',
# 'u_forward',
# 'chassis_psidot',
# 'chassis_yaw_angle',
# 'yaw_kalman1x1_estimate',
# 'imu_gz',
# 'yaw_angle_kalman_rate_integration',
# 'imu_gy',
# 'yaw_rate_world',
# 'yaw_angle_IMU_rate_integ_no_kalman',
# 'speed_pid_integral',
# 'u_turn',
# 'pid_update_available',
# 'kp_angle',
# 'ki_angle',
# 'kd_angle',
# 'kp_speed',
# 'ki_speed',
# 'kd_speed',
# 'kp_yaw',
# 'ki_yaw',
# 'yaw_pid_integral']


# Ensure time column exists
if TIME_COLUMN not in data.columns:
    raise ValueError(f"Time column '{TIME_COLUMN}' not found in {CSV_FILENAME}")

variables = [col for col in data.columns if col != TIME_COLUMN]

# ---------------- Filtering ----------------
mask = (data[variables].abs() <= THRESHOLD).all(axis=1)
filtered_data = data[mask]
num_removed = len(data) - len(filtered_data)

if num_removed > 0:
    print(f"⚠️  Filtered out {num_removed} rows exceeding ±{THRESHOLD}")

time = filtered_data[TIME_COLUMN]

# # ---------------- Combined Figure ----------------

# Set depending on PWM frequency. Draws the MIN/MAX horizontal lines
Min_PWM_Left_Forward_Tuning_Safe = 200
Min_PWM_Left_Backward_Tuning_Safe = -200
Min_PWM_Right_Forward_Tuning_Safe = 200
Min_PWM_Right_Backward_Tuning_Safe = -200	

# Adjust scales for plotting if needed
scale = {
    "u_pitch_angle": 1.0,
    "pwm_left": 1.0,
    "pwm_right": 1.0,
    #"chassisData_xdot": 1000.0,
    "pitch_kalman1x1_estimate": 1.0,
    "chassisData_xdot": 1.0,
    "u_speed_hold": 1.0,
    "imu_gx": 1.0,
    "imu_gy": 1.0,
    "imu_gz": 1.0,
    # "imu_gy": 180/3.1415,
    # "imu_gz": 180/3.1415,
    "yaw_angle_IMU_rate_integ_no_kalman": 1.0,
    "speed_pid_integral": 1.0,
    "u_speed_hold_kp_term": 1.0,
    "u_speed_hold_ki_term": 1.0,
    "u_speed_hold_kd_term": 1.0,
    "u_pitch_angle_kp_term_kalman_contribution": 1.0,
    "u_pitch_angle_kp_term_cascade_contribution": 1.0,
    "u_pitch_angle_kp_term_superposition": 1.0,
    "pid_update_available": 1.0,
    "chassis_psidot": 180/3.1415
}

fig_all, ax_all = plt.subplots(figsize=(10, 6))

# Set background color
fig_all.patch.set_facecolor('black')  # figure background
ax_all.set_facecolor('black')         # axes background

line_colors = {
    # "pitch_kalman1x1_angle": "#ff9f05ff",  # orange
    # "pwm_left": "#0055ffff",                # blue
    # "pwm_right": "#008300ff",               # green
}

scatter_vars = {}  # <-- variables drawn as scatter

lines = []
for var in variables:
    factor = scale.get(var, 1.0)
    y = filtered_data[var] * factor

    label = var + (f" (×{factor})" if factor != 1 else "")

    if var in scatter_vars:
        # --- scatter instead of line ---
        line = ax_all.scatter(time, y, label=label, s=10)  # s = marker size
    else:
        # --- normal line ---
        line, = ax_all.plot(time, y, label=label)

    lines.append(line)

ax_all.set_xlabel('Time [s]', color='white')
ax_all.set_ylabel('Value', color='white')
ax_all.set_title(f"All Signals from {CSV_FILENAME}", color='white')
ax_all.tick_params(colors='white')
ax_all.grid(True, color='gray', linestyle='--', alpha=0.5)

h1 = ax_all.axhline(Min_PWM_Left_Forward_Tuning_Safe * scale["pwm_left"], color='red', linestyle='--', linewidth=1.5, label='Min Left Forward')
h2 = ax_all.axhline(Min_PWM_Right_Forward_Tuning_Safe * scale["pwm_right"], color='magenta', linestyle='--', linewidth=1.5, label='Min Right Forward')
h3 = ax_all.axhline(Min_PWM_Left_Backward_Tuning_Safe * scale["pwm_left"], color='red', linestyle='--', linewidth=1.5, label='Min Left Backward')
h4 = ax_all.axhline(Min_PWM_Right_Backward_Tuning_Safe * scale["pwm_right"], color='magenta', linestyle='--', linewidth=1.5, label='Min Right Backward')

leg = ax_all.legend()

for text in leg.get_texts():
    text.set_color('white')  # legend text

# legend
leg.get_frame().set_facecolor('black')  
leg.get_frame().set_edgecolor('white')   
leg.get_frame().set_alpha(0.8)           
leg.set_draggable(True)

# --- Click legend to toggle visibility ---
legend_line_map = {}

all_plot_lines = lines + [h1, h2, h3, h4]

for legend_handle, origline in zip(leg.legend_handles, all_plot_lines):
    legend_handle.set_picker(5)
    legend_line_map[legend_handle] = origline

def on_pick(event):
    legline = event.artist
    origline = legend_line_map[legline]
    visible = not origline.get_visible()
    origline.set_visible(visible)
    legline.set_alpha(1.0 if visible else 0.2)
    fig_all.canvas.draw()

fig_all.canvas.mpl_connect('pick_event', on_pick)

# --- Hover tooltips (snap to actual data point) ---
import mplcursors

def on_add(sel):
    artist = sel.artist

    # Line label (remove scale suffix if present)
    label = artist.get_label().split(" ")[0]

    # Get line data
    xdata = artist.get_xdata()
    ydata = artist.get_ydata()

    # Mouse hover position
    hover_x, _ = sel.target

    # Snap to nearest X point
    idx = np.argmin(np.abs(xdata - hover_x))

    real_x = xdata[idx]
    real_y = ydata[idx]

    # Force integer formatting if the data were originally integer
    if label in filtered_data.columns:
        orig_dtype = filtered_data[label].dtype
        if np.issubdtype(orig_dtype, np.integer):
            real_y = int(real_y)

    sel.annotation.set_text(
        f"{artist.get_label()}\n"
        f"time = {real_x}\n"
        f"value = {real_y}"
    )

# Show value on mouser hover
mplcursors.cursor(all_plot_lines, hover=True).connect("add", on_add)


# Save combined figure
combined_path = os.path.join(SAVE_DIR, 'all_signals.png')
fig_all.savefig(combined_path, dpi=300)
print(f"💾 Saved combined plot: {combined_path}")


# ---------------- Controller Terms Graph ----------------

# ====== MANUALLY SET THESE VALUES ======
kp_angle = float(sys.argv[1])
ki_angle = float(sys.argv[2])
kd_angle = float(sys.argv[3])
kp_speed = float(sys.argv[4])
ki_speed = float(sys.argv[5])
kd_speed = float(sys.argv[6])

target_speed = 0.0
target_pitch_angle = 0.0
# =======================================

# Extract signals
pitch = filtered_data["pitch_kalman1x1_estimate"]
gx = filtered_data["imu_gx"]
xdot = filtered_data["chassisData_xdot"]
u_speed_hold = filtered_data["u_speed_hold"]
u_pitch_angle = filtered_data["u_pitch_angle"]
speed_int = filtered_data["speed_pid_integral"]
t = filtered_data["time"]

# ---- Compute controller terms ----

# Speed control components
u_speed_hold_kp_term = kp_speed * (target_speed - xdot)
u_speed_hold_ki_term = ki_speed * speed_int
u_speed_hold_kd_term = u_speed_hold - u_speed_hold_ki_term - u_speed_hold_kp_term

# Pitch control components
targetPitch_uSpeedHold = target_pitch_angle + u_speed_hold
#targetPitch_uSpeedHold = target_pitch_angle - u_speed_hold

u_pitch_angle_kp_term_cascade = kp_angle * (pitch - targetPitch_uSpeedHold)
u_pitch_angle_kp_term_superposition = kp_angle * (pitch - target_pitch_angle)

u_pitch_angle_kp_term_kalman_contribution = kp_angle * pitch
u_pitch_angle_kp_term_cascade_contribution = -kp_angle * targetPitch_uSpeedHold
u_pitch_angle_kd_term = kd_angle * gx

# pitch kp term with u_speed_hold = 0
u_pitch_angle_kp_term_no_speed_hold = kp_angle * (pitch - target_pitch_angle)

# total pitch command without speed hold (pure pitch PD)
u_pitch_angle_no_speed_hold = (
    u_pitch_angle_kp_term_no_speed_hold +
    u_pitch_angle_kd_term
)

# ---- Plot everything on the same figure ----
fig_ctrl, ax_ctrl = plt.subplots(figsize=(10, 6))

lines_ctrl = []

factor = scale.get("u_speed_hold_kp_term", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, u_speed_hold_kp_term* scale["u_speed_hold_kp_term"], label="u_speed_hold_kp_term" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.4)[0])

factor = scale.get("u_speed_hold_ki_term", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, u_speed_hold_ki_term* scale["u_speed_hold_ki_term"], label="u_speed_hold_ki_term" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.4)[0])

factor = scale.get("u_speed_hold", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, u_speed_hold* scale["u_speed_hold"], label="u_speed_hold" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.8)[0])

factor = scale.get("u_pitch_angle_kp_term_kalman_contribution", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, u_pitch_angle_kp_term_kalman_contribution* scale["u_pitch_angle_kp_term_kalman_contribution"], label="u_pitch_angle_kp_term_kalman_contribution" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.8)[0])

factor = scale.get("u_pitch_angle_kp_term_cascade_contribution", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, u_pitch_angle_kp_term_cascade_contribution* scale["u_pitch_angle_kp_term_cascade_contribution"], label="u_pitch_angle_kp_term_cascade_contribution" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.8)[0])

factor = scale.get("u_pitch_angle_kp_term_superposition", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, u_pitch_angle_kp_term_superposition* scale["u_pitch_angle_kp_term_superposition"], label="u_pitch_angle_kp_term_superposition" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.8)[0])

factor = scale.get("imu_gx", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, gx* scale["imu_gx"], label="imu_gx" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.8)[0])

factor = scale.get("u_speed_hold_kd_term", 1.0)
lines_ctrl.append(ax_ctrl.plot(t, u_speed_hold_kd_term* scale["u_speed_hold_kd_term"], label="u_speed_hold_kd_term (total - other two)" + (f" (×{factor})" if factor != 1 else ""), linewidth=1.8)[0])

lines_ctrl.append(ax_ctrl.plot(t, u_pitch_angle_kp_term_cascade, label="u_pitch_angle_kp_term_cascade", linewidth=1.4)[0])
lines_ctrl.append(ax_ctrl.plot(t, u_pitch_angle_kd_term, label="u_pitch_angle_kd_term", linewidth=1.4)[0])
lines_ctrl.append(ax_ctrl.plot(t, u_pitch_angle, label="u_pitch_angle (recorded)", linewidth=1.8)[0])

# --- Add signals ---
factor = scale.get("chassisData_xdot", 1.0)
lines_ctrl.append(
    ax_ctrl.plot(
        t,
        filtered_data["chassisData_xdot"]* scale["chassisData_xdot"],
        label="chassisData_xdot" + (f" (×{factor})" if factor != 1 else ""),
        linewidth=1.8,
    )[0]
)

factor = scale.get("pitch_kalman1x1_estimate", 1.0)
lines_ctrl.append(
    ax_ctrl.plot(
        t,
        filtered_data["pitch_kalman1x1_estimate"]* scale["pitch_kalman1x1_estimate"],
        label="pitch_kalman1x1_estimate" + (f" (×{factor})" if factor != 1 else ""),
        linewidth=1.8,
    )[0]
)

# --- Add speed_pid_integral ---
factor = scale.get("speed_pid_integral", 1.0)
lines_ctrl.append(
    ax_ctrl.plot(
        t,
        filtered_data["speed_pid_integral"]* scale["speed_pid_integral"],
        label="speed_pid_integral" + (f" (×{factor})" if factor != 1 else ""),
        linewidth=1.8,
    )[0]
)

# Kp without speed hold
lines_ctrl.append(
    ax_ctrl.plot(
        t,
        u_pitch_angle_kp_term_no_speed_hold,
        label="u_pitch_angle_kp_term_no_speed_hold",
        linewidth=1.4,
    )[0]
)

# Full pitch command without speed hold
lines_ctrl.append(
    ax_ctrl.plot(
        t,
        u_pitch_angle_no_speed_hold,
        label="u_pitch_angle_no_speed_hold",
        linewidth=1.4,
    )[0]
)

ax_ctrl.set_xlabel("Time [s]")
ax_ctrl.set_ylabel("Controller Output Value")
ax_ctrl.set_title("Controller Terms and Outputs")
ax_ctrl.grid(True)

# ----- Legend -----
leg_ctrl = ax_ctrl.legend()
legend_line_map_ctrl = {}
for legline, origline in zip(leg_ctrl.get_lines(), lines_ctrl):
    legline.set_picker(5)
    legend_line_map_ctrl[legline] = origline

def on_pick_ctrl(event):
    legline = event.artist
    origline = legend_line_map_ctrl[legline]

    visible = not origline.get_visible()
    origline.set_visible(visible)
    legline.set_alpha(1.0 if visible else 0.2)

    fig_ctrl.canvas.draw_idle()

fig_ctrl.canvas.mpl_connect("pick_event", on_pick_ctrl)
mplcursors.cursor(lines_ctrl, hover=True).connect("add", on_add)

# Save figure
controller_plot_path = os.path.join(SAVE_DIR, "controller_terms.png")
fig_ctrl.savefig(controller_plot_path, dpi=300)
print(f"💾 Saved controller terms plot: {controller_plot_path}")


# ---------------- Show All ----------------
plt.show()