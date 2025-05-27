import machine, network, time
from umqtt.robust import MQTTClient

# === CONFIG ===
WIFI_SSID       = "RoboticArm"
WIFI_PASSWORD   = "ARM12345"
MQTT_BROKER     = "192.168.4.1"
TOPIC_DESIRED   = b"arm/joints/desired"
TOPIC_ACTUAL    = b"arm/joints/actual"
PUBLISH_MS      = 100
TIMEOUT_MS      = 1000

# === HARDWARE ===
J3_POT = machine.ADC(0)
J3_FWD = machine.PWM(machine.Pin(5))
J3_REV = machine.PWM(machine.Pin(4))
J3_FWD.freq(1000); J3_REV.freq(1000)
LED    = machine.Pin(2, machine.Pin.OUT)

# === CALIBRATION ===
CAL = sorted([(871,-90),(697,-45),(505,0),(319,45),(134,90),(10,135)])
ANGLE_MIN, ANGLE_MAX = -85, 120

# === PID GAINS ===
Kp, Ki, Kd = 0.055, 0.0005, 0.07
integral = 0.0
prev_error = 0.0
ERROR_TOL = 1.2

# === STATE ===
desired_mode    = "X"     # "P", "V", or "X"
desired_value   = None
current_angle   = 0.0
last_cmd_ms     = time.ticks_ms()
last_pub_ms     = 0
last_led_ms     = 0

# === UTILS ===
def adc_to_angle(v):
    if v <= CAL[0][0]: return CAL[0][1]
    if v >= CAL[-1][0]: return CAL[-1][1]
    for (a1,ang1),(a2,ang2) in zip(CAL, CAL[1:]):
        if a1 <= v <= a2:
            frac = (v - a1)/(a2 - a1)
            return round(ang1 + frac*(ang2-ang1),2)
    return None

def clamp_near_bounds(angle, vel):
    m = 10
    if angle < ANGLE_MIN+m and vel < 0:
        return vel * ((angle - ANGLE_MIN)/m)
    if angle > ANGLE_MAX-m and vel > 0:
        return vel * ((ANGLE_MAX - angle)/m)
    return vel

def stop():
    J3_FWD.duty(0); J3_REV.duty(0)

def drive(v):
    v = clamp_near_bounds(current_angle, v)
    pwm = int(min(abs(v), 0.95) * 1023)
    if v > 0:
        J3_FWD.duty(pwm); J3_REV.duty(0)
    elif v < 0:
        J3_FWD.duty(0);    J3_REV.duty(pwm)
    else:
        stop()

def pid_ctrl(target, actual):
    global integral, prev_error
    err = target - actual
    integral += err
    deriv = err - prev_error
    prev_error = err
    output = (Kp * err) + (Ki * integral) + (Kd * deriv)
    return output, err

def gravity_ff(angle):
    # 0° → +0.15; fades to 0 by ±70°
    return 0.15 * max(0, 1 - abs(angle)/70)

# === NETWORK ===
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    for _ in range(20):
        if wlan.isconnected(): return True
        time.sleep(0.5)
    return False

def mqtt_cb(topic, msg):
    global desired_mode, desired_value, last_cmd_ms
    try:
        parts = msg.decode().strip("[]").split(",")
        if len(parts) != 7: return
        j3 = parts[2].strip().upper()
        if j3.startswith("P"):
            desired_mode  = "P"
            desired_value = float(j3[1:])
        elif j3.startswith("V"):
            desired_mode  = "V"
            desired_value = float(j3[1:])
        else:
            desired_mode  = "X"
            desired_value = None
        last_cmd_ms = time.ticks_ms()
    except:
        pass

def connect_mqtt():
    c = MQTTClient("j3", MQTT_BROKER)
    c.set_callback(mqtt_cb)
    c.connect()
    c.subscribe(TOPIC_DESIRED)
    print("✅ MQTT Connected")
    return c

# === MAIN LOOP ===
def loop(client):
    global current_angle, last_pub_ms, last_led_ms
    global desired_mode, desired_value, last_cmd_ms

    while True:
        now = time.ticks_ms()
        client.check_msg()

        angle = adc_to_angle(J3_POT.read())
        if angle is None:
            stop()
            continue
        current_angle = angle

        # — Velocity-timeout → position-hold
        if desired_mode == "V" and time.ticks_diff(now, last_cmd_ms) > TIMEOUT_MS:
            desired_mode  = "P"
            desired_value = current_angle

        if desired_mode == "P" and desired_value is not None:
            raw_out, err = pid_ctrl(desired_value, current_angle)
            # Aggressive slow-down when very close
            if abs(err) < 7:
                raw_out *= 0.30
            # Additional damping at high angles
            if abs(current_angle) > 70:
                raw_out *= 0.9
            mag = min(abs(raw_out), 0.95)
            v_pid = (1 if raw_out>0 else -1) * mag
            drive(v_pid + gravity_ff(current_angle))

        elif desired_mode == "V" and desired_value is not None:
            drive(desired_value + gravity_ff(current_angle))

        else:
            # no command → hold with gravity compensation
            g = gravity_ff(current_angle)
            drive(g if g > 0.02 else 0)

        # — Publish feedback
        if time.ticks_diff(now, last_pub_ms) > PUBLISH_MS:
            client.publish(TOPIC_ACTUAL, f"[J3, {current_angle}]")
            last_pub_ms = now

        # — Blink LED
        if time.ticks_diff(now, last_led_ms) > 500:
            LED.value(not LED.value())
            last_led_ms = now

        time.sleep_ms(10)

# === START ===
if connect_wifi():
    c = connect_mqtt()
    if c:
        loop(c)
else:
    print("❌ WiFi failed")
