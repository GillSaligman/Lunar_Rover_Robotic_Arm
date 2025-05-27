import paho.mqtt.client as mqtt
import threading
import time
import RPi.GPIO as GPIO

# =================== CONFIGURATION ===================
BROKER = "192.168.4.1"
PORT = 1883
TOPIC_DESIRED = "arm/joints/desired"
TOPIC_ACTUAL = "arm/joints/actual"
TOPIC_MASTER = "arm/joints/master"

# GPIO pin setup
LEFT_FWD = 18
LEFT_REV = 13
RIGHT_FWD = 19
RIGHT_REV = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup([LEFT_FWD, LEFT_REV, RIGHT_FWD, RIGHT_REV], GPIO.OUT)
lf_pwm = GPIO.PWM(LEFT_FWD, 1000)
lr_pwm = GPIO.PWM(LEFT_REV, 1000)
rf_pwm = GPIO.PWM(RIGHT_FWD, 1000)
rr_pwm = GPIO.PWM(RIGHT_REV, 1000)
lf_pwm.start(0)
lr_pwm.start(0)
rf_pwm.start(0)
rr_pwm.start(0)

# =================== GLOBAL STATE ===================
actual_positions = ["X"] * 7
desired_positions = ["X"] * 7
master_positions = ["X"] * 7
print_actual = False
master_mode = False

# =================== MOTOR CONTROL ===================
def set_motor_pwm(pwm_fwd, pwm_rev, value):
    duty = int(min(abs(value) * 100, 100))
    if value > 0:
        pwm_fwd.ChangeDutyCycle(duty)
        pwm_rev.ChangeDutyCycle(0)
    elif value < 0:
        pwm_fwd.ChangeDutyCycle(0)
        pwm_rev.ChangeDutyCycle(duty)
    else:
        pwm_fwd.ChangeDutyCycle(0)
        pwm_rev.ChangeDutyCycle(0)

def handle_drive(dl, dr):
    try:
        dl_val = float(dl)
        dr_val = float(dr)
        set_motor_pwm(lf_pwm, lr_pwm, dl_val)
        set_motor_pwm(rf_pwm, rr_pwm, dr_val)
    except:
        # One or both drive inputs invalid – stop motors
        set_motor_pwm(lf_pwm, lr_pwm, 0)
        set_motor_pwm(rf_pwm, rr_pwm, 0)

# =================== MQTT CALLBACKS ===================
def on_connect(client, userdata, flags, rc):
    print("✅ Connected to MQTT")
    client.subscribe(TOPIC_ACTUAL)
    client.subscribe(TOPIC_MASTER)

def on_message(client, userdata, msg):
    global actual_positions, master_positions, desired_positions

    payload = msg.payload.decode().strip('[]')
    parts = payload.split(',')

    if msg.topic == TOPIC_ACTUAL:
        if len(parts) == 2 and parts[0].startswith("J"):
            try:
                idx = int(parts[0][1:]) - 1
                actual_positions[idx] = f"{float(parts[1]):.2f}"
            except:
                pass

    elif msg.topic == TOPIC_MASTER:
        if len(parts) == 7:
            master_positions = parts
            if master_mode:
                desired_positions = parts[:]
                client.publish(TOPIC_DESIRED, ",".join(desired_positions))
                try:
                    handle_drive(desired_positions[0], desired_positions[1])
                except:
                    pass

# =================== PRINT POSITIONS ===================
def print_positions():
    while True:
        if print_actual:
            print("\n===== ROBOT ARM STATUS =====")
            if master_mode:
                print(f"🟡 MASTER  : {master_positions}")
            print(f"🔵 ACTUAL  : {actual_positions}")
            print(f"🟢 DESIRED : {desired_positions}")
        time.sleep(0.1)

# =================== HANDLE USER INPUT ===================
def handle_input():
    global print_actual, master_mode, desired_positions
    while True:
        cmd = input("Enter command (e.g., 'j5 45', 'm', 'p'): ").strip().lower()

        if cmd == 'p':
            print_actual = not print_actual
            print(f"📡 Position printing {'enabled' if print_actual else 'disabled'}")

        elif cmd == 'm':
            master_mode = not master_mode
            print(f"🔁 Master Controller Mode {'ON' if master_mode else 'OFF'}")
            if master_mode:
                desired_positions = master_positions[:]
                client.publish(TOPIC_DESIRED, ",".join(desired_positions))
                try:
                    handle_drive(desired_positions[0], desired_positions[1])
                except:
                    pass

        elif cmd.startswith('j'):
            try:
                parts = cmd.split()
                joint = int(parts[0][1:]) - 1
                value = parts[1]

                if 0 <= joint < 7:
                    if value.lower() == 'x':
                        desired_positions[joint] = 'X'
                    elif value.lower().startswith('v'):
                        desired_positions[joint] = f"V{float(value[1:]):.2f}"
                    else:
                        desired_positions[joint] = f"P{float(value):.2f}"

                    client.publish(TOPIC_DESIRED, ",".join(desired_positions))

                    # Only handle drive motors if we're modifying DL/DR
                    if joint == 0 or joint == 1:
                        handle_drive(desired_positions[0], desired_positions[1])

                    print(f"✅ Sent: {desired_positions}")
                else:
                    print("⚠️ Invalid joint number (1–7)")
            except:
                print("⚠️ Invalid command format")

# =================== MQTT STARTUP ===================
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT)
client.loop_start()

input_thread = threading.Thread(target=handle_input, daemon=True)
print_thread = threading.Thread(target=print_positions, daemon=True)
input_thread.start()
print_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    lf_pwm.stop()
    lr_pwm.stop()
    rf_pwm.stop()
    rr_pwm.stop()
    GPIO.cleanup()
    client.disconnect()
    print("🛑 Shutdown")
