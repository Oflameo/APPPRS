# This script makes laptop ready to be a wifi clinet
# Run this if the wifi option is greyed out
kill -9 $(cat hostapd.pid)
wait 3
rm hostapd.pid
ip addr del dev wlan0
rfkill unblock wlan
nmcli nm wifi on
