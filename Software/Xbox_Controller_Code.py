v# controller.py

import pygame
import paho.mqtt.client as mqtt
import time
import sys

# === CONFIG ===
BROKER_IP        = "192.168.4.1"
BROKER_PORT      = 1883
MQTT_TOPIC       = "arm/joints/master"
SEND_RATE        = 0.1      # seconds per loop
DEADZONE         = 0.1
JOY_DZ           = 0.2      # ARM‐mode deadzone isolation
JOINT_SCALE      = 1.0
GRIP_RATE        = 5.0      # degrees per loop
GRIP_MIN,GRIP_MAX= 0.0,100.0

# ANSI colors for console feedback
ARM_COLOR   = "\x1b[35m"    # magenta
DRIVE_COLOR = "\x1b[36m"    # cyan
RESET_COLOR = "\x1b[0m"

# === STATE ===
mode        = "ARM"          # or "DRIVE"
joint_vel   = [0.0]*7        # 0,1=drive; 2–5=J3–J6; 6=gripper
grip_pos    = 50.0           # start at midpoint

# DRIVE-mode joint selection
sel_joints  = [3,4,5]        # J4,J5,J6
sel_idx     = 0
last_lb, last_rb = False, False

# Menu (3-line) button edge detect (index 7)
last_menu   = False

# === INIT ===
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count()==0:
    print("No joystick found.", file=sys.stderr)
    sys.exit(1)
joy = pygame.joystick.Joystick(0)
joy.init()

client = mqtt.Client(protocol=mqtt.MQTTv311)
try:
    client.connect(BROKER_IP, BROKER_PORT)
    client.loop_start()
    print("✅ MQTT connected")
except:
    print("⚠️ MQTT connect failed")
    client = None

def print_mode():
    if mode=="ARM":
        print(f"{ARM_COLOR}=== ARM MODE ==={RESET_COLOR}")
    else:
        print(f"{DRIVE_COLOR}=== DRIVE MODE ==={RESET_COLOR}")

print_mode()
last_time = time.time()

# === MAIN LOOP ===
while True:
    now = time.time()
    dt  = now - last_time
    last_time = now

    pygame.event.pump()
    values = [0.0]*7

    # — Toggle ARM/DRIVE on Menu button (index 7)
    menu = joy.get_button(7)
    if menu and not last_menu:
        mode = "DRIVE" if mode=="ARM" else "ARM"
        print_mode()
        joint_vel = [0.0]*7
    last_menu = menu

    if mode=="ARM":
        # → Right stick: J3 = X (inverted), J4 = Y (inverted)
        rx = joy.get_axis(3)
        ry = -joy.get_axis(2)
        j3 = rx if abs(rx)>DEADZONE else 0.0
        j4 = ry if abs(ry)>DEADZONE else 0.0
        if abs(j3)>JOY_DZ and abs(j4)<=JOY_DZ: j4 = 0.0
        if abs(j4)>JOY_DZ and abs(j3)<=JOY_DZ: j3 = 0.0
        joint_vel[2] = round(-j3 * JOINT_SCALE, 2)
        joint_vel[3] = round(-j4 * JOINT_SCALE, 2)

        # → Left stick: J5 = Y, J6 = X
        lx = joy.get_axis(0)
        ly = -joy.get_axis(1)
        j5 = ly if abs(ly)>DEADZONE else 0.0
        j6 = lx if abs(lx)>DEADZONE else 0.0
        if abs(j5)>JOY_DZ and abs(j6)<=JOY_DZ: j6 = 0.0
        if abs(j6)>JOY_DZ and abs(j5)<=JOY_DZ: j5 = 0.0
        joint_vel[4] = round(j5 * JOINT_SCALE, 2)
        joint_vel[5] = round(j6 * JOINT_SCALE, 2)

        # → Gripper via triggers (±5° per loop)
        if joy.get_axis(4) > DEADZONE:
            grip_pos = max(GRIP_MIN, grip_pos - GRIP_RATE)
        if joy.get_axis(5) > DEADZONE:
            grip_pos = min(GRIP_MAX, grip_pos + GRIP_RATE)

        values = joint_vel[:]
        values[6] = int(grip_pos)

    else:  # DRIVE MODE
        # → Tank drive: left stick Y → slot 0, right stick Y → slot 1
        ly = -joy.get_axis(1)
        ry = -joy.get_axis(3)   # use axis(3) for right stick Y
        left = ly if abs(ly)>DEADZONE else 0.0
        right= ry if abs(ry)>DEADZONE else 0.0
        joint_vel[0] = round(left * JOINT_SCALE, 2)
        joint_vel[1] = round(right* JOINT_SCALE, 2)

        # → Ramped J3 via A(0)/Y(3) up to ±0.5 over 2s
        ramp = 0.5 / 2.0  # per second
        if joy.get_button(3):      # Y
            joint_vel[2] = min( 0.5, joint_vel[2] + ramp * dt )
        elif joy.get_button(0):    # A
            joint_vel[2] = max(-0.5, joint_vel[2] - ramp * dt )
        else:
            joint_vel[2] = 0.0

        # → X(2)/B(1) open/close gripper ±5° per loop
        if joy.get_button(2):
            grip_pos = max(GRIP_MIN, grip_pos - GRIP_RATE)
        if joy.get_button(1):
            grip_pos = min(GRIP_MAX, grip_pos + GRIP_RATE)

        # → LB(4)/RB(5) cycle J4–J6
        lb, rb = joy.get_button(4), joy.get_button(5)
        if lb and not last_lb:
            sel_idx = (sel_idx - 1) % len(sel_joints)
            for i in sel_joints: joint_vel[i] = 0.0
        if rb and not last_rb:
            sel_idx = (sel_idx + 1) % len(sel_joints)
            for i in sel_joints: joint_vel[i] = 0.0
        last_lb, last_rb = lb, rb

        # → triggers set selected joint velocity = trigger depth
        sj = sel_joints[sel_idx]
        lt, rt = joy.get_axis(4), joy.get_axis(5)
        if lt > DEADZONE:
            joint_vel[sj] = round(-lt * JOINT_SCALE, 2)
        elif rt > DEADZONE:
            joint_vel[sj] = round(rt * JOINT_SCALE, 2)
        else:
            joint_vel[sj] = 0.0

        values = joint_vel[:]
        values[6] = int(grip_pos)

    # — Zero out any -0.00 values
    for i,v in enumerate(values):
        if abs(v) < 1e-6:
            values[i] = 0.0

    # — Send over MQTT
    payload = "[" + ",".join(f"{v:.2f}" if i<6 else str(v)
                             for i,v in enumerate(values)) + "]"
    print("→", payload)
    if client:
        client.publish(MQTT_TOPIC, payload)

    time.sleep(SEND_RATE)
