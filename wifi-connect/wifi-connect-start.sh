#!/usr/bin/env bash
# WiFi Connect start script for Verter Robot
# Based on official: https://github.com/balena-os/wifi-connect/blob/master/scripts/start.sh

set -e

# Configuration
: "${PORTAL_INTERFACE:=wlan0}"
: "${PORTAL_SSID:=Verter-Setup}"
: "${PORTAL_PASSPHRASE:=verter123}"
: "${PORTAL_GATEWAY:=192.168.42.1}"
: "${PORTAL_DHCP_RANGE:=192.168.42.2,192.168.42.254}"
: "${UI_DIRECTORY:=/home/verter/verter-robot/wifi-connect/ui}"
: "${ACTIVITY_TIMEOUT:=0}"

WIFI_CONNECT="/home/verter/verter-robot/wifi-connect/wifi-connect"

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$1"
}

# Optional: wait for NetworkManager to establish connection after boot
# Recommended by balena docs: give time for auto-connection to happen
sleep 15

# Check if already connected to WiFi (official balena approach)
# iwgetid returns 0 if connected, 1 if not
if iwgetid -r &>/dev/null; then
    log 'Already connected to WiFi - skipping WiFi Connect'
    exit 0
fi

log 'No WiFi connection detected - starting WiFi Connect'

# Stop system dnsmasq to avoid port conflicts
DNSMASQ_WAS_RUNNING=false
if systemctl is-active --quiet dnsmasq 2>/dev/null; then
    log "Stopping system dnsmasq to avoid conflicts..."
    systemctl stop dnsmasq
    DNSMASQ_WAS_RUNNING=true
fi

# wifi-connect will:
# 1. Check if internet is available (double-check)
# 2. If yes - exit with code 0
# 3. If no - start AP and captive portal
# 4. When user connects - save to NetworkManager and exit with code 0
"${WIFI_CONNECT}" \
    --portal-interface "${PORTAL_INTERFACE}" \
    --portal-ssid "${PORTAL_SSID}" \
    --portal-passphrase "${PORTAL_PASSPHRASE}" \
    --portal-gateway "${PORTAL_GATEWAY}" \
    --portal-dhcp-range "${PORTAL_DHCP_RANGE}" \
    --ui-directory "${UI_DIRECTORY}" \
    --activity-timeout "${ACTIVITY_TIMEOUT}"

EXIT_CODE=$?

# Restart system dnsmasq if it was running
if [ "${DNSMASQ_WAS_RUNNING}" = "true" ]; then
    log "Restarting system dnsmasq..."
    systemctl start dnsmasq
fi

if [ $EXIT_CODE -eq 0 ]; then
    log "WiFi Connect completed successfully"
    
    # Enable autoconnect for the newly connected network
    # This ensures auto-reconnection after reboot
    CONNECTED_WIFI=$(nmcli -t -f NAME,TYPE,DEVICE connection show --active | grep ":802-11-wireless:${PORTAL_INTERFACE}" | cut -d: -f1)
    if [ -n "$CONNECTED_WIFI" ]; then
        log "Enabling autoconnect for: $CONNECTED_WIFI"
        nmcli connection modify "$CONNECTED_WIFI" connection.autoconnect yes
        log "Autoconnect enabled"
    fi
else
    log "WiFi Connect exited with code $EXIT_CODE"
fi

exit $EXIT_CODE
