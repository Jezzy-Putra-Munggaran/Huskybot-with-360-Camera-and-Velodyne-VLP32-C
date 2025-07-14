#!/bin/bash
# filepath: /home/kmp-orin/jezzy/huskybot/scripts/ultimate_camera_recovery.sh

echo "🚨 ULTIMATE Camera Recovery - TOTAL SYSTEM RESET"

# Auto password
PASS="kmporin"

echo "🔄 Step 1: FORCE kill ALL camera-related processes..."
echo "$PASS" | sudo -S pkill -9 -f nvargus-daemon || true
echo "$PASS" | sudo -S pkill -9 -f video_source || true  
echo "$PASS" | sudo -S pkill -9 -f gstreamer || true
echo "$PASS" | sudo -S pkill -9 -f argus || true
echo "$PASS" | sudo -S pkill -9 -f nvcamera || true
sleep 3

echo "🔄 Step 2: COMPLETE argus cleanup..."
echo "$PASS" | sudo -S rm -rf /tmp/.argus* || true
echo "$PASS" | sudo -S rm -rf /var/run/argus* || true
echo "$PASS" | sudo -S rm -rf /tmp/argus* || true
echo "$PASS" | sudo -S rm -rf /var/lib/argus* || true
sleep 2

echo "🔄 Step 3: Reset ALL camera hardware..."
echo "$PASS" | sudo -S systemctl stop nvargus-daemon
sleep 5
echo "$PASS" | sudo -S systemctl daemon-reload
sleep 2
echo "$PASS" | sudo -S systemctl start nvargus-daemon
sleep 8

echo "🔄 Step 4: Verify daemon status..."
if systemctl is-active --quiet nvargus-daemon; then
    echo "✅ nvargus-daemon is ACTIVE"
else
    echo "❌ nvargus-daemon FAILED to start"
    echo "$PASS" | sudo -S systemctl restart nvargus-daemon
    sleep 10
fi

echo "🔄 Step 5: Reset device permissions..."
echo "$PASS" | sudo -S chmod 666 /dev/video* 2>/dev/null || true
echo "$PASS" | sudo -S chown root:video /dev/video* 2>/dev/null || true

echo "🔄 Step 6: Test camera devices..."
for i in {0..5}; do
    if ls /dev/video$i &>/dev/null; then
        echo "✅ /dev/video$i exists"
    else
        echo "❌ /dev/video$i missing"
    fi
done

echo "🔄 Step 7: Reset kernel modules..."
echo "$PASS" | sudo -S modprobe -r uvcvideo || true
echo "$PASS" | sudo -S modprobe uvcvideo || true
sleep 2

echo "✅ ULTIMATE Camera recovery completed!"
echo "📝 nvargus-daemon status:"
systemctl status nvargus-daemon --no-pager -l
