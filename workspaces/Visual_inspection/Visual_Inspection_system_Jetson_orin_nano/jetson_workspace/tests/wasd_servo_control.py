#!/usr/bin/env python3
"""
Pan-Tilt Servo Controller — Gamepad (F710) + WASD Keyboard
============================================================
Prefers Logitech F710 gamepad for TRUE simultaneous analog control.
Falls back to WASD keyboard if no gamepad found.

GAMEPAD (recommended):
  Left stick  = Pan (X axis) + Tilt (Y axis) simultaneously
  Right bumper (RB) = increase speed
  Start/B     = Center servos
  Analog control = gentle push → slow, full push → fast

WASD KEYBOARD (fallback):
  W/S = Tilt    A/D = Pan    R = Center    Q = Quit

  DIAGONAL SINGLE-KEY SHORTCUTS (always work!):
    Q = W+A  (Tilt UP   + Pan LEFT )
    E = W+D  (Tilt UP   + Pan RIGHT)
    Z = A+S  (Pan LEFT  + Tilt DOWN)
    X = S+D  (Tilt DOWN + Pan RIGHT)
"""

import serial, glob, os, sys, time, tty, termios, select, threading, struct

# ── Config ────────────────────────────────────────────────────────────────────
BAUD              = 9600
PAN_MIN, PAN_MAX  = 0, 180
TILT_MIN, TILT_MAX = 20, 160
LOOP_HZ           = 30

# Gamepad
AXIS_DEADZONE     = 0.08    # ignore stick input below this
GAMEPAD_SPEED     = 45.0    # max deg/s for gamepad

# Keyboard
KB_VEL_MAX        = 60.0
KB_ACCEL          = 55.0
KB_DECEL          = 90.0
KB_HOLD_WINDOW    = 0.30    # seconds — key stays active after last press

# ── Port Detection ────────────────────────────────────────────────────────────
def find_port():
    if os.path.exists('/dev/arduino'): return '/dev/arduino'
    for port in sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')):
        try:
            check = os.path.realpath(f'/sys/class/tty/{os.path.basename(port)}')
            for _ in range(8):
                vf = os.path.join(check, 'idVendor')
                if os.path.exists(vf):
                    if open(vf).read().strip() in {'1a86','2341','0403'}: return port
                    break
                check = os.path.dirname(check)
        except: pass
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    return ports[0] if ports else None

# ── Gamepad Detection ─────────────────────────────────────────────────────────
def find_gamepad():
    """Find joystick device — tries /dev/input/js* """
    # Look for F710 specifically
    try:
        for js in sorted(glob.glob('/dev/input/js*')):
            if os.access(js, os.R_OK):
                return js
    except: pass
    return None

# ── Display ───────────────────────────────────────────────────────────────────
def draw(pan, tilt, vp, vt, mode, extra, msg):
    B = 28
    def b(v,lo,hi): p=max(0,min(B,int(((v-lo)/(hi-lo))*B))); return '['+('─'*p)+'●'+('─'*(B-p))+']'
    def vb(v,mx):   p=max(0,min(B,int((v/mx)*B)));             return '['+('█'*p)+('░'*(B-p))+']'
    spd = max(abs(vp), abs(vt))
    mx  = GAMEPAD_SPEED if mode=='GAMEPAD' else KB_VEL_MAX
    print('\033[H\033[J', end='')
    print("╔══════════════════════════════════════╗")
    print(f"║  🎮  PAN-TILT  [{mode:<10}]         ║")
    print("╠══════════════════════════════════════╣")
    if mode == 'GAMEPAD':
        print("║  Left Stick = Pan + Tilt (both axes) ║")
        print("║  Start/B = Center   Trig=Speed boost ║")
    else:
        print("║  W↑ Tilt   S↓ Tilt                   ║")
        print("║  A← Pan    D→ Pan   R=Center  Q=Quit  ║")
    print("╠══════════════════════════════════════╣")
    print(f"║  PAN  :{pan:>6.1f}°  {b(pan, PAN_MIN,PAN_MAX)} ║")
    print(f"║  TILT :{tilt:>6.1f}°  {b(tilt,TILT_MIN,TILT_MAX)} ║")
    print(f"║  SPEED:{spd:>5.0f}°/s  {vb(spd,mx)} ║")
    print(f"║  {'INPUT: '+extra:<36} ║")
    print("╠══════════════════════════════════════╣")
    print(f"║  {(msg or '')[:36]:<36}  ║")
    print("╚══════════════════════════════════════╝")

# ── Serial Send ───────────────────────────────────────────────────────────────
def send(ser, pan, tilt):
    ser.write(f"{180-int(round(tilt))},{int(round(pan))}\n".encode())

# ── Gamepad Control Loop ──────────────────────────────────────────────────────
def run_gamepad(ser, js_path):
    """
    Read joystick events (Linux js protocol: 8-byte structs).
    Format: 4B time | 2B value | 1B type | 1B number
    type 1 = button, type 2 = axis
    F710 axes: 0=LX, 1=LY, 2=LT, 3=RX, 4=RY, 5=RT
    Buttons: 0=A,1=B,2=X,3=Y,4=LB,5=RB,6=LT,7=RT,8=Back,9=Start
    """
    pan, tilt = 90.0, 90.0
    vp = vt = 0.0
    axes = {}
    msg = 'Left stick for pan+tilt simultaneously!'
    dt  = 1.0 / LOOP_HZ

    send(ser, pan, tilt)

    try:
        js = open(js_path, 'rb')
    except PermissionError:
        print(f"❌ Permission denied: {js_path}")
        print(f"   Fix: sudo chmod a+r {js_path}  or  sudo usermod -a -G input $USER")
        return False

    print(f"✅ Gamepad: {js_path}")
    print("   Left stick = Pan + Tilt  |  Start/B = Center")
    time.sleep(0.5)

    try:
        while True:
            t0 = time.time()

            # Read all pending events
            while True:
                r, _, _ = select.select([js], [], [], 0)
                if not r: break
                data = js.read(8)
                if len(data) < 8: break
                _, value, etype, number = struct.unpack('IhBB', data)
                etype &= ~0x80  # strip init flag

                if etype == 2:  # axis
                    axes[number] = value / 32767.0

                if etype == 1 and value == 1:  # button press
                    if number in (0, 1, 9):   # A, B, Start = center
                        pan, tilt = 90.0, 90.0
                        vp = vt = 0.0
                        msg = '🏠 Center (90,90)'
                        send(ser, pan, tilt)
                    if number == 7:   # RT button = quit
                        return True

            # Apply axes
            lx = axes.get(0, 0.0)   # left stick X → pan
            ly = axes.get(1, 0.0)   # left stick Y → tilt

            # Deadzone
            if abs(lx) < AXIS_DEADZONE: lx = 0.0
            if abs(ly) < AXIS_DEADZONE: ly = 0.0

            # Normalize (input is -1 to +1, apply speed)
            vp = lx * GAMEPAD_SPEED   # pan velocity
            vt = -ly * GAMEPAD_SPEED  # tilt velocity (invert Y: push up = tilt up)

            if lx != 0.0 or ly != 0.0:
                pan  = max(PAN_MIN,  min(PAN_MAX,  pan  + vp * dt))
                tilt = max(TILT_MIN, min(TILT_MAX, tilt + vt * dt))
                msg  = f'⬅➡P={pan:.0f}°  ⬆⬇T={tilt:.0f}°  ({lx:+.2f},{ly:+.2f})'
                send(ser, pan, tilt)

            draw(pan, tilt, vp, vt, 'GAMEPAD',
                 f'LX={lx:+.2f} LY={ly:+.2f}', msg)

            elapsed = time.time() - t0
            time.sleep(max(0, dt - elapsed))

    except KeyboardInterrupt:
        pass
    finally:
        js.close()
        send(ser, 90.0, 90.0)

    return True

# ── Keyboard Control Loop ─────────────────────────────────────────────────────
def run_keyboard(ser):
    pan, tilt = 90.0, 90.0
    vp = vt = 0.0
    stamps = {}
    msg = 'Hold W then D together for diagonal!'
    dt  = 1.0 / LOOP_HZ

    send(ser, pan, tilt)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)

    try:
        while True:
            t0 = time.time()

            while True:
                r,_,_ = select.select([sys.stdin],[],[],0)
                if not r: break
                ch = os.read(fd,1).decode('utf-8',errors='ignore')
                if ch == '\x1b':
                    r2,_,_ = select.select([sys.stdin],[],[],0.02)
                    if r2:
                        os.read(fd,1)
                        r3,_,_ = select.select([sys.stdin],[],[],0.02)
                        ch = {'A':'w','B':'s','C':'d','D':'a'}.get(
                            os.read(fd,1).decode('utf-8',errors='ignore') if r3 else '', '') if r2 else 'q'
                    else: ch='q'
                k = ch.lower()
                if k == '\x03': raise KeyboardInterrupt
                if k == 'r':
                    pan=tilt=90.0; vp=vt=0.0; stamps.clear()
                    msg='🏠 Center (90,90)'; send(ser, pan, tilt)
                elif k in ('w','a','s','d'):
                    now = time.time()
                    stamps[k] = now
                    # Cross-refresh: if other axis was recently active, keep it alive
                    # This makes W+D held together work — each key refreshes the other
                    if k in ('a', 'd'):        # pan key → refresh tilt if active
                        for tk in ('w', 's'):
                            if tk in stamps and now - stamps[tk] < KB_HOLD_WINDOW:
                                stamps[tk] = now
                    elif k in ('w', 's'):      # tilt key → refresh pan if active
                        for pk in ('a', 'd'):
                            if pk in stamps and now - stamps[pk] < KB_HOLD_WINDOW:
                                stamps[pk] = now

                # ── Diagonal single-key shortcuts ──────────────
                elif k == 'q':   # W+A  → tilt up   + pan left
                    stamps['w'] = stamps['a'] = time.time()
                elif k == 'e':   # W+D  → tilt up   + pan right
                    stamps['w'] = stamps['d'] = time.time()
                elif k == 'z':   # A+S  → pan left  + tilt down
                    stamps['a'] = stamps['s'] = time.time()
                elif k == 'x':   # S+D  → tilt down + pan right
                    stamps['s'] = stamps['d'] = time.time()

            now    = time.time()
            active = {k for k,t in stamps.items() if now-t < KB_HOLD_WINDOW}

            mt = 'w' in active or 's' in active
            mp = 'a' in active or 'd' in active
            vt = min(KB_VEL_MAX, vt+KB_ACCEL*dt) if mt else max(0.0, vt-KB_DECEL*dt)
            vp = min(KB_VEL_MAX, vp+KB_ACCEL*dt) if mp else max(0.0, vp-KB_DECEL*dt)

            parts, chg = [], False
            if 'w' in active and vt>0:
                tilt=min(TILT_MAX,tilt+vt*dt); parts.append(f'⬆T={tilt:.0f}°'); chg=True
            elif 's' in active and vt>0:
                tilt=max(TILT_MIN,tilt-vt*dt); parts.append(f'⬇T={tilt:.0f}°'); chg=True
            if 'a' in active and vp>0:
                pan=min(PAN_MAX, pan+vp*dt);   parts.append(f'⬅P={pan:.0f}°');  chg=True
            elif 'd' in active and vp>0:
                pan=max(PAN_MIN, pan-vp*dt);   parts.append(f'➡P={pan:.0f}°');  chg=True

            if parts: msg = '  '.join(parts)+f'  {max(vp,vt):.0f}°/s'
            if chg or vp>0 or vt>0: send(ser, pan, tilt)

            ks = '+'.join(sorted(active&{'w','a','s','d'})).upper() or 'none'
            draw(pan, tilt, vp, vt, 'KEYBOARD', f'keys:[{ks}]', msg)
            time.sleep(max(0, dt-(time.time()-t0)))

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        send(ser, 90.0, 90.0)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print("🔍 Finding servo controller...")
    port = find_port()
    if not port:
        print("❌ No serial port found."); sys.exit(1)
    if not os.access(port, os.R_OK|os.W_OK):
        os.system(f'sudo chmod 666 {port}')
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
        time.sleep(2)
        print(f"✅ Serial: {port}")
    except Exception as e:
        print(f"❌ {e}"); sys.exit(1)

    js = find_gamepad()
    if js:
        print(f"🎮 Gamepad found: {js} — using analog control!")
        print("   (For keyboard instead: run with --keyboard flag)")
        if '--keyboard' not in sys.argv:
            run_gamepad(ser, js)
            ser.close()
            return

    print("⌨️  No gamepad found — using WASD keyboard")
    run_keyboard(ser)
    ser.close()
    print("\n✅ Done!")

if __name__ == '__main__':
    main()
