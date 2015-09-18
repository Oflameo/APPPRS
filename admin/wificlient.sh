# This script makes laptop ready to be a wifi clinet
# Run this if the wifi option is greyed out
sudo rfkill unblock wlan
nmcli nm wifi on
