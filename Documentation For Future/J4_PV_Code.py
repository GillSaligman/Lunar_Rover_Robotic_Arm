# main_j4.py

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
TIMEOUT_MS    = 1000    # velocity timeout → position hold

# ===== HARDWARE SETUP =====
J4_POT = machine.ADC(0)
J4_FWD = machine.PWM(machine.Pin(5))
J4_REV = machine.PWM(machine.Pin(4))
J4_FWD.freq(1000); J4_REV.freq(1000)
LED    = machine.Pin(2, machine.Pin.OUT)

# ===== PID PARAMETERS =====
Kp = 0.075
Ki = 0.0001
Kd = 0.1
MIN_SPEED  = 0.3   # minimum PWM fraction
MAX_SPEED  = 0.95  # maximum PWM fraction
ERROR_TOL  = 0.75  # degrees tolerance

integral   = 0.0
prev_error = 0.0

# ===== STATE =====
desired_mode  = "X"   # "P", "V", or "X"
desired_value = None
current_angle = 0.0
last_cmd_ms   = time.ticks_ms()
last_pub_ms   = 0
last_led_ms   = 0

# ===== J4 CALIBRATION =====
# (adc_value, angle_deg)
J4_CALIBRATION = sorted([
    (600,   0),
    (393, -45),
    (330, -85),
    (810,  45),
    (890,  85),
], key=lambda x: x[0])

# ===== UTILS =====
def adc_to_angle(adc):
    if   adc <= J4_CALIBRATION[0][0]: return J4_CALIBRATION[0][1]
    elif adc >= J4_CALIBRATION[-1][0]: return J4_CALIBRATION[-1][1]
    for (a1,ang1),(a2,ang2) in zip(J4_CALIBRATION, J4_CALIBRATION[1:]):
        if a1 <= adc <= a2:
            frac = (adc - a1)/(a2 - a1)
            return round(ang1 + frac*(ang2 - ang1), 2)
    return None

def set_motor(frac):
    # clamp to ±MAX_SPEED
    frac = max(min(frac, MAX_SPEED), -MAX_SPEED)
    duty = int(abs(frac) * 1023)
    if frac > 0:
        J4_FWD.duty(duty); J4_REV.duty(0)
    elif frac < 0:
        J4_FWD.duty(0);    J4_REV.duty(duty)
    else:
        J4_FWD.duty(0);    J4_REV.duty(0)

def pid_control(target, actual):
    global integral, prev_error
    err = target - actual
    integral += err
    deriv = err - prev_error
    prev_error = err
    out = (Kp * err) + (Ki * integral) + (Kd * deriv)
    mag = min(max(abs(out), MIN_SPEED), MAX_SPEED)
    return mag, err

# ===== NETWORK =====
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    for _ in range(20):
        if wlan.isconnected(): return True
        time.sleep(0.5)
    return False

def mqtt_callback(topic, msg):
    global desired_mode, desired_value, last_cmd_ms
    parts = msg.decode().strip("[]").split(",")
    if len(parts) != 7:
        return
    slot = parts[3].strip().upper()  # J4 is index 3
    if slot.startswith("P"):
        desired_mode  = "P"
        desired_value = float(slot[1:])
    elif slot.startswith("V"):
        desired_mode  = "V"
        desired_value = float(slot[1:])  # now a raw –1…+1 fraction
    else:
        desired_mode  = "X"
        desired_value = None
    last_cmd_ms = time.ticks_ms()

def connect_mqtt():
    client = MQTTClient("arm_j4", MQTT_BROKER, MQTT_PORT)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_DESIRED)
    print("✅ MQTT Connected")
    return client

# ===== MAIN CONTROL LOOP =====
def control_loop(client):
    global current_angle, last_pub_ms, last_led_ms
    global desired_mode, desired_value, last_cmd_ms

    print("✅ J4 control loop started")
    while True:
        now = time.ticks_ms()
        client.check_msg()

        adc = J4_POT.read()
        angle = adc_to_angle(adc)
        if angle is None:
            set_motor(0)
            continue
        current_angle = angle

        # velocity timeout → switch to position hold
        if desired_mode == "V" and time.ticks_diff(now, last_cmd_ms) > TIMEOUT_MS:
            desired_mode  = "P"
            desired_value = current_angle

        if desired_mode == "P" and desired_value is not None:
            mag, err = pid_control(desired_value, current_angle)
            if abs(err) > ERROR_TOL:
                frac = (1 if err > 0 else -1) * mag
                set_motor(frac)
            else:
                set_motor(0)

        elif desired_mode == "V" and desired_value is not None:
            # drive at raw fraction
            set_motor(desired_value)

        else:
            set_motor(0)

        # publish feedback
        if time.ticks_diff(now, last_pub_ms) > PUBLISH_MS:
            client.publish(TOPIC_ACTUAL, f"[J4, {current_angle}]")
            last_pub_ms = now

        # blink LED
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
