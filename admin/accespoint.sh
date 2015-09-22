# This script turns the laptop into an access point
touch hostapd.pid
nmcli nm wifi off
rfkill unblock wlan
ip addr add 192.168.16.1/24 dev wlan0
hostapd -B -P hostapd.pid hostapd.conf
