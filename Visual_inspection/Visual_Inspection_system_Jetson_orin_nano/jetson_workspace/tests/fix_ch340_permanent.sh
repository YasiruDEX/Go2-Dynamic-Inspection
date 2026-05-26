#!/bin/bash
# ============================================================
#  PERMANENT CH340 SERIAL FIX FOR JETSON ORIN NANO
#  Run ONCE as:  sudo bash fix_ch340_permanent.sh
#  After this, the PCB/Arduino will ALWAYS be at /dev/arduino
#  on every boot — no manual modprobe needed ever again.
# ============================================================

set -e
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║   CH340 PERMANENT FIX — JETSON ORIN NANO     ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# ── Step 1: Check if ch34x module is already installed ──────────────────────
echo "▶ Step 1: Checking ch34x module..."
if find /lib/modules/$(uname -r) -name "ch34x.ko*" 2>/dev/null | grep -q ch34x; then
    echo "  ✅ ch34x module already installed"
else
    echo "  ⚠️  ch34x not installed — building from source..."

    # Check if CH341SER source exists
    if [ ! -d "/home/$SUDO_USER/CH341SER" ]; then
        echo "  📥 Cloning CH341SER..."
        cd /home/$SUDO_USER
        sudo -u $SUDO_USER git clone https://github.com/juliagoda/CH341SER.git
    fi

    cd /home/$SUDO_USER/CH341SER

    # Install kernel headers if needed
    if [ ! -d "/usr/src/linux-headers-$(uname -r)" ]; then
        echo "  📦 Installing kernel headers..."
        apt-get install -y nvidia-l4t-kernel-headers 2>/dev/null || \
        apt-get install -y linux-headers-$(uname -r) 2>/dev/null || \
        echo "  ⚠️  Headers not found via apt — trying to build anyway..."
    fi

    echo "  🔨 Building ch34x driver..."
    sudo -u $SUDO_USER make clean 2>/dev/null || true
    sudo -u $SUDO_USER make

    echo "  📦 Installing ch34x driver permanently..."
    make install
    depmod -a

    echo "  ✅ ch34x driver installed"
fi

# ── Step 2: Auto-load ch34x on every boot ───────────────────────────────────
echo ""
echo "▶ Step 2: Setting ch34x to auto-load on boot..."
echo 'ch34x' > /etc/modules-load.d/ch34x.conf
echo "  ✅ Created /etc/modules-load.d/ch34x.conf"

# ── Step 3: Disable brltty (steals CH340 devices) ───────────────────────────
echo ""
echo "▶ Step 3: Disabling brltty (the CH340 thief)..."
systemctl stop brltty 2>/dev/null || true
systemctl disable brltty 2>/dev/null || true
systemctl mask brltty 2>/dev/null || true
# Also mask brltty-udev if it exists
systemctl mask brltty-udev 2>/dev/null || true
apt-get remove -y brltty 2>/dev/null || true
echo "  ✅ brltty disabled and removed"

# ── Step 4: Permanent udev rule → /dev/arduino symlink ──────────────────────
echo ""
echo "▶ Step 4: Creating permanent udev rule..."
cat > /etc/udev/rules.d/99-ch340-arduino.rules << 'EOF'
# CH340 USB Serial Converter (PCB / Arduino clone)
# Always creates /dev/arduino symlink with open permissions
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    SYMLINK+="arduino", MODE="0666", \
    TAG+="systemd"
EOF
echo "  ✅ Created /etc/udev/rules.d/99-ch340-arduino.rules"

# ── Step 5: Reload udev rules ────────────────────────────────────────────────
echo ""
echo "▶ Step 5: Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger
echo "  ✅ udev rules reloaded"

# ── Step 6: Load ch34x NOW (no reboot needed for this session) ───────────────
echo ""
echo "▶ Step 6: Loading ch34x driver now..."
modprobe ch34x 2>/dev/null || insmod /home/$SUDO_USER/CH341SER/ch34x.ko 2>/dev/null || true
sleep 1
udevadm trigger
sleep 1

# ── Step 7: Verify ───────────────────────────────────────────────────────────
echo ""
echo "▶ Step 7: Verifying..."
if ls /dev/ttyUSB* 2>/dev/null | head -1; then
    echo "  ✅ /dev/ttyUSB* detected"
else
    echo "  ⚠️  No /dev/ttyUSB* yet — replug the USB cable"
fi

if ls -la /dev/arduino 2>/dev/null; then
    echo "  ✅ /dev/arduino symlink ready"
else
    echo "  ⚠️  /dev/arduino not yet — replug USB cable once"
fi

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║           ✅ ALL DONE!                        ║"
echo "║                                              ║"
echo "║  Replug USB cable once, then:                ║"
echo "║    ls /dev/arduino       → should appear     ║"
echo "║    python3 wasd_servo_control.py             ║"
echo "║                                              ║"
echo "║  After every reboot: /dev/arduino is READY   ║"
echo "║  No modprobe, no chmod, nothing!             ║"
echo "╚══════════════════════════════════════════════╝"
echo ""
