# main_j5.py

import machine, network, time
from umqtt.robust import MQTTClient

# ===== CONFIGURATION =====
WIFI_SSID       = "RoboticArm"
WIFI_PASSWORD   = "ARM12345"
MQTT_BROKER     = "192.168.4.1"
MQTT_PORT       = 1883
TOPIC_DESIRED   = b"arm/joints/desired"
TOPIC_ACTUAL    = b"arm/joints/actual"
PUBLISH_MS      = 100     # feedback every 100 ms
TIMEOUT_MS      = 1000    # velocity timeout

# ===== HARDWARE SETUP =====
J5_POT = machine.ADC(0)
J5_FWD = machine.PWM(machine.Pin(5))
J5_REV = machine.PWM(machine.Pin(4))
J5_FWD.freq(1000); J5_REV.freq(1000)
LED    = machine.Pin(2, machine.Pin.OUT)

# ===== PID GAINS (your original) =====
Kp = 0.075
Ki = 0.0001
Kd = 0.1
MIN_SPEED = 0.3
MAX_SPEED = 0.95
ERROR_TOL = 0.75

# ===== STATE =====
integral     = 0.0
prev_error   = 0.0
desired_mode = "X"     # "P", "V", or "X"
desired_value= None
current_angle= 0.0
last_cmd_ms  = time.ticks_ms()
last_pub_ms  = 0
last_led_ms  = 0

# ===== CALIBRATION =====
J5_CAL = sorted([
    (223,  85),
    (346,  45),
    (526,   0),
    (736, -45),
    (889, -90)
], key=lambda x: x[0])

# ===== HELPERS =====
def adc_to_angle(adc):
    if   adc <= J5_CAL[0][0]: return J5_CAL[0][1]
    elif adc >= J5_CAL[-1][0]: return J5_CAL[-1][1]
    for (a1,ang1),(a2,ang2) in zip(J5_CAL, J5_CAL[1:]):
        if a1 <= adc <= a2:
            frac = (adc - a1)/(a2 - a1)
            return round(ang1 + frac*(ang2-ang1), 2)
    return None

def pid_control(target, actual):
    global integral, prev_error
    err = target - actual
    integral += err
    deriv = err - prev_error
    prev_error = err
    out = Kp*err + Ki*integral + Kd*deriv
    pwm = max(MIN_SPEED, min(abs(out), MAX_SPEED))
    return pwm, err

def set_motor(frac):
    # clamp to ±0.95
    frac = max(min(frac, 0.95), -0.95)
    duty = int(abs(frac)*1023)
    if frac > 0:
        J5_FWD.duty(duty); J5_REV.duty(0)
    elif frac < 0:
        J5_FWD.duty(0);    J5_REV.duty(duty)
    else:
        J5_FWD.duty(0);    J5_REV.duty(0)

# ===== NETWORK =====
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
    parts = msg.decode().strip("[]").split(",")
    if len(parts) != 7: return
    cmd = parts[4].strip().upper()  # J5 is slot 4
    if   cmd.startswith("P"):
        desired_mode  = "P"
        desired_value = float(cmd[1:])
    elif cmd.startswith("V"):
        desired_mode  = "V"
        desired_value = float(cmd[1:])  # this is the raw -1→1 fraction
    else:
        desired_mode  = "X"
        desired_value = None
    last_cmd_ms = time.ticks_ms()

def connect_mqtt():
    c = MQTTClient("arm_j5", MQTT_BROKER, MQTT_PORT)
    c.set_callback(mqtt_cb)
    c.connect()
    c.subscribe(TOPIC_DESIRED)
    print("✅ MQTT Connected")
    return c

# ===== CONTROL LOOP =====
def control_loop(client):
    global current_angle, last_pub_ms, last_led_ms
    global desired_mode, desired_value, last_cmd_ms

    print("✅ J5 control loop started")
    while True:
        now = time.ticks_ms()
        client.check_msg()

        adc = J5_POT.read()
        ang = adc_to_angle(adc)
        if ang is None:
            set_motor(0)
            continue
        current_angle = ang

        # Velocity timeout → position-hold
        if desired_mode=="V" and time.ticks_diff(now, last_cmd_ms) > TIMEOUT_MS:
            desired_mode  = "P"
            desired_value = current_angle

        if desired_mode=="P" and desired_value is not None:
            speed, err = pid_control(desired_value, current_angle)
            if abs(err) > ERROR_TOL:
                frac = (1 if err>0 else -1) * speed
                set_motor(frac)
            else:
                set_motor(0)

        elif desired_mode=="V" and desired_value is not None:
            # desired_value is already -1…1, so drive directly
            set_motor(desired_value)

        else:
            set_motor(0)

        # Publish actual angle
        if time.ticks_diff(now, last_pub_ms) > PUBLISH_MS:
            client.publish(TOPIC_ACTUAL, f"[J5, {current_angle}]")
            last_pub_ms = now

        # Blink LED
        if time.ticks_diff(now, last_led_ms) > 500:
            LED.value(not LED.value())
            last_led_ms = now

        time.sleep_ms(10)

# ===== STARTUP =====
if connect_wifi():
    mqtt = connect_mqtt()
    if mqtt:
        control_loop(mqtt)
else:
    print("❌ WiFi failed")
