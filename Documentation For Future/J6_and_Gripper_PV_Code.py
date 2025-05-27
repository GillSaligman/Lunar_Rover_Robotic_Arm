# main_j6_gripper.py

import machine, network, time
from umqtt.robust import MQTTClient

# ===== CONFIGURATION =====
WIFI_SSID     = "RoboticArm"
WIFI_PASSWORD = "ARM12345"
MQTT_BROKER   = "192.168.4.1"
MQTT_PORT     = 1883
TOPIC_DESIRED = b"arm/joints/desired"
TOPIC_ACTUAL  = b"arm/joints/actual"
PUBLISH_MS    = 100     # publish feedback every 100 ms
TIMEOUT_MS    = 1000    # timeout for velocity → switch to hold

# ===== HARDWARE SETUP =====
J6_POT   = machine.ADC(0)
J6_FWD   = machine.PWM(machine.Pin(5), freq=1000)
J6_REV   = machine.PWM(machine.Pin(4), freq=1000)

# Servo on GPIO14, 50 Hz
GRIP_PWM = machine.PWM(machine.Pin(14), freq=50)
LED      = machine.Pin(2, machine.Pin.OUT)

# ===== CALIBRATION for J6 pot ⇒ angle =====
J6_CALIB = sorted([
    (-270,  66),
    (-180, 240),
    (-90,  390),
    (0,    550),
    (90,   706),
    (180,  865),
    (260, 1010),
], key=lambda x: x[1])

# ===== PID PARAMETERS for J6 =====
Kp, Ki, Kd = 0.075, 0.0001, 0.1
MIN_PWM, MAX_PWM = 0.32, 0.95
ERROR_TOL      = 2

integral = 0.0
prev_err = 0.0

# ===== STATE =====
desired_mode  = "X"    # "P", "V" or "X"
desired_j6    = None
desired_grip  = None
current_angle = 0.0
last_cmd_ms   = time.ticks_ms()
last_pub_ms   = 0
last_led_ms   = 0

# ===== HELPERS =====
def adc_to_angle(adc):
    for (a1,v1),(a2,v2) in zip(J6_CALIB, J6_CALIB[1:]):
        if v1 <= adc <= v2:
            frac = (adc - v1)/(v2 - v1)
            return a1 + frac*(a2 - a1)
    return None

def set_motor(frac):
    frac = max(min(frac, MAX_PWM), -MAX_PWM)
    duty = int(abs(frac)*1023)
    if frac > 0:
        J6_FWD.duty(duty); J6_REV.duty(0)
    elif frac < 0:
        J6_FWD.duty(0);   J6_REV.duty(duty)
    else:
        J6_FWD.duty(0);   J6_REV.duty(0)

def pid_control(target, actual):
    global integral, prev_err
    err = target - actual
    integral += err
    deriv = err - prev_err
    prev_err = err
    out = Kp*err + Ki*integral + Kd*deriv
    mag = max(MIN_PWM, min(abs(out), MAX_PWM))
    return mag, err

def update_gripper():
    """
    At each loop, command the servo to desired_grip
    using the full 40–115 duty range (1–2 ms pulses).
    """
    if desired_grip is None:
        return

    # MicroPython on 50 Hz:
    # 1 ms pulse ≈ 40/1023 duty, 2 ms ≈ 115/1023 duty
    DUTY_MIN = 40
    DUTY_MAX = 115

    # clamp angle
    angle = max(0, min(180, desired_grip))
    duty = int(DUTY_MIN + (angle/180)*(DUTY_MAX - DUTY_MIN))
    GRIP_PWM.duty(duty)

# ===== NETWORK =====
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    for _ in range(20):
        if wlan.isconnected():
            return True
        time.sleep(0.5)
    return False

def mqtt_callback(topic, msg):
    global desired_mode, desired_j6, desired_grip, last_cmd_ms
    parts = msg.decode().strip("[]").split(",")
    if len(parts) != 7:
        return

    # J6 is slot 5
    cmd6 = parts[5].strip().upper()
    if   cmd6.startswith("P"):
        desired_mode = "P"
        desired_j6   = float(cmd6[1:])
    elif cmd6.startswith("V"):
        desired_mode = "V"
        desired_j6   = float(cmd6[1:])
    else:
        desired_mode = "X"
        desired_j6   = None

    # Gripper is slot 6 (raw 0–180°)
    try:
        desired_grip = float(parts[6])
    except:
        desired_grip = None

    last_cmd_ms = time.ticks_ms()

def connect_mqtt():
    client = MQTTClient("arm_j6", MQTT_BROKER, MQTT_PORT)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_DESIRED)
    print("✅ MQTT Connected")
    return client

# ===== MAIN LOOP =====
def control_loop(client):
    global current_angle, last_pub_ms, last_led_ms
    global desired_mode, desired_j6, desired_grip, last_cmd_ms

    print("✅ J6 + Gripper loop started")
    while True:
        now = time.ticks_ms()
        client.check_msg()

        # — J6 Position/Velocity —
        adc = J6_POT.read()
        ang = adc_to_angle(adc)
        if ang is not None:
            current_angle = ang

            # velocity timeout → switch to hold (position)
            if (desired_mode=="V" and
                time.ticks_diff(now, last_cmd_ms) > TIMEOUT_MS):
                desired_mode = "P"
                desired_j6   = current_angle

            if desired_mode=="P" and desired_j6 is not None:
                mag, err = pid_control(desired_j6, current_angle)
                frac = (1 if err>0 else -1)*mag if abs(err)>ERROR_TOL else 0
                set_motor(frac)

            elif desired_mode=="V" and desired_j6 is not None:
                set_motor(desired_j6)

            else:
                set_motor(0)
        else:
            set_motor(0)

        # — Gripper holds position —
        update_gripper()

        # — Publish J6 feedback —
        if time.ticks_diff(now, last_pub_ms) > PUBLISH_MS:
            client.publish(TOPIC_ACTUAL, f"[J6, {current_angle:.2f}]")
            last_pub_ms = now

        # — Blink LED —
        if time.ticks_diff(now, last_led_ms) > 500:
            LED.value(not LED.value())
            last_led_ms = now

        time.sleep_ms(10)

# ===== STARTUP =====
if connect_wifi():
    mqtt = connect_mqtt()
    control_loop(mqtt)
else:
    print("❌ WiFi failed")
