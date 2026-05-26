#!/usr/bin/env python3
"""
test_mqtt.py — Standalone MQTT test for ThingsBoard
Run on Jetson directly (no ROS2 needed).

Tests:
  1. Connection to broker
  2. Send plain telemetry (no image)
  3. Send one small JPEG image as base64
  4. Check local capture folder
  5. (Optional) send a real image from Logitech camera

Usage:
  python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_mqtt.py
"""

import sys, os, time, json, base64, traceback
import cv2
import yaml

# ── Load config ────────────────────────────────────────────────────────
CFG_PATH = os.path.expanduser('~/Documents/Visual_Inspection_ws/config/mqtt_config.yaml')

try:
    with open(CFG_PATH) as f:
        cfg = yaml.safe_load(f)
    print(f'[✓] Config loaded from {CFG_PATH}')
except Exception as e:
    print(f'[✗] Cannot load config: {e}')
    print('    Using defaults (localhost)')
    cfg = {
        'broker': 'demo.thingsboard.io', 'port': 1883,
        'access_token': '34bbvq0ix4u2licucuq0',
        'topic': 'v1/devices/me/telemetry',
        'timeout': 10, 'qos': 1, 'jpeg_quality': 85
    }

BROKER = cfg.get('broker', 'demo.thingsboard.io')
PORT   = cfg.get('port', 1883)
TOKEN  = cfg.get('access_token', '')
TOPIC  = cfg.get('topic', 'v1/devices/me/telemetry')
QOS    = cfg.get('qos', 1)

print(f'\n━━━━ MQTT Config ━━━━━━━━━━━━━━━━━━━━━━━')
print(f'  Broker : {BROKER}:{PORT}')
print(f'  Token  : {TOKEN[:8]}...{TOKEN[-4:]}  ({len(TOKEN)} chars)')
print(f'  Topic  : {TOPIC}')
print(f'  QoS    : {QOS}')
print(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n')

# ── Test 1: Connection ─────────────────────────────────────────────────
print('TEST 1: Connecting to broker...')
try:
    import paho.mqtt.client as mqtt

    connected = False
    connect_rc = None

    def on_connect(c, u, f, rc):
        global connected, connect_rc
        connected  = True
        connect_rc = rc

    client = mqtt.Client()
    client.on_connect = on_connect
    if TOKEN:
        client.username_pw_set(TOKEN, '')
    client.connect(BROKER, PORT, keepalive=cfg.get('timeout', 10))
    client.loop_start()

    deadline = time.time() + 10
    while not connected and time.time() < deadline:
        time.sleep(0.1)

    if not connected:
        print('[✗] Connection timeout (10s)')
        sys.exit(1)

    RC_MSGS = {0:'OK', 1:'Wrong protocol', 2:'Client ID rejected',
               3:'Server unavailable', 4:'Bad credentials', 5:'Not authorised'}
    rc_msg = RC_MSGS.get(connect_rc, f'Unknown rc={connect_rc}')

    if connect_rc == 0:
        print(f'[✓] Connected! rc=0 ({rc_msg})')
    else:
        print(f'[✗] Connection refused: rc={connect_rc} — {rc_msg}')
        sys.exit(1)
except Exception as e:
    print(f'[✗] Connection error: {e}')
    traceback.print_exc()
    sys.exit(1)

# ── Test 2: Send plain telemetry ───────────────────────────────────────
print('\nTEST 2: Sending plain telemetry (no image)...')
try:
    payload = json.dumps({
        'test':       True,
        'timestamp':  time.time(),
        'message':    'Visual Inspection System test ping',
        'class_name': 'test_object',
        'session':    'test_mqtt_script'
    })
    result = client.publish(TOPIC, payload, qos=QOS)
    try:
        result.wait_for_publish(timeout=5)
    except TypeError:
        result.wait_for_publish()
    print(f'[✓] Telemetry sent ({len(payload)} bytes)')
    print(f'    → Check ThingsBoard: https://demo.thingsboard.io')
    print(f'      Devices → inspection → Latest Telemetry → look for "message" key')
except Exception as e:
    print(f'[✗] Publish error: {e}')

# ── Test 3: Send small test image (generated) ─────────────────────────
print('\nTEST 3: Sending synthetic test image as base64...')
try:
    # Create a 100x100 test image with text
    img = cv2.imread('/dev/null') if False else None
    img = 255 * __import__('numpy').ones((100, 100, 3), dtype='uint8')
    img[:, :] = (50, 50, 200)   # blue-ish
    cv2.putText(img, 'TEST', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,255,255), 2)
    _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    b64 = base64.b64encode(buf.tobytes()).decode()

    payload = json.dumps({
        'session':    'test_mqtt_script',
        'object_id':  0,
        'class_name': 'test_synthetic',
        'image_idx':  1,
        'total':      1,
        'timestamp':  time.time(),
        'image_b64':  b64
    })
    result = client.publish(TOPIC, payload, qos=QOS)
    try:
        result.wait_for_publish(timeout=5)
    except TypeError:
        result.wait_for_publish()
    print(f'[✓] Synthetic image sent ({len(b64)//1024}KB base64)')
except Exception as e:
    print(f'[✗] Image publish error: {e}')

# ── Test 4: Send real camera frame ─────────────────────────────────────
print('\nTEST 4: Capturing real frame from Logitech camera...')
LOGI_PATH = '/dev/logitech'
if not os.path.exists(LOGI_PATH):
    LOGI_PATH = '/dev/video2'  # fallback
    print(f'  /dev/logitech not found, trying {LOGI_PATH}')

try:
    cap = cv2.VideoCapture(LOGI_PATH)
    if not cap.isOpened():
        print(f'[!] Cannot open {LOGI_PATH} — skipping real camera test')
    else:
        ret, frame = cap.read()
        cap.release()
        if ret:
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            b64 = base64.b64encode(buf.tobytes()).decode()
            payload = json.dumps({
                'session':    'test_mqtt_script',
                'object_id':  1,
                'class_name': 'real_logitech_frame',
                'image_idx':  1,
                'total':      1,
                'timestamp':  time.time(),
                'image_b64':  b64
            })
            result = client.publish(TOPIC, payload, qos=QOS)
            try:
                result.wait_for_publish(timeout=5)
            except TypeError:
                result.wait_for_publish()
            print(f'[✓] Real Logitech frame sent ({len(b64)//1024}KB base64) — {frame.shape}')
        else:
            print('[!] Camera opened but could not read frame')
except Exception as e:
    print(f'[!] Camera test skipped: {e}')

# ── Test 5: Check captures folder ─────────────────────────────────────
print('\nTEST 5: Checking captures folder...')
cap_dir = os.path.expanduser(
    cfg.get('capture_dir', '~/Documents/Visual_Inspection_ws/captures'))
if os.path.exists(cap_dir):
    files = []
    for root, dirs, fnames in os.walk(cap_dir):
        for f in fnames:
            files.append(os.path.join(root, f))
    print(f'[✓] captures/ exists: {len(files)} file(s)')
    for f in files[-10:]:  # show last 10
        size = os.path.getsize(f) // 1024
        print(f'    {f.replace(cap_dir, "captures")}  ({size}KB)')
    if not files:
        print('    (empty — run the pipeline first to capture images)')
else:
    print(f'[!] Folder does not exist yet: {cap_dir}')
    print('    It will be created on first inspection run')

# ── Cleanup ────────────────────────────────────────────────────────────
client.loop_stop()
client.disconnect()

print(f'\n━━━━ SUMMARY ━━━━━━━━━━━━━━━━━━━━━━━━━━')
print(f'  ✓ Tests complete')
print(f'  → ThingsBoard: https://demo.thingsboard.io')
print(f'    Devices → "inspection" → Latest Telemetry')
print(f'  → Look for keys: test, message, class_name, image_idx')
print(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n')
