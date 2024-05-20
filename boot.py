import time
import network

SSID = 'JAISTALL'
SSID_PASSWORD = ''


def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    print(f"MAC: {wlan.config('mac').hex()}")
    if not wlan.isconnected():
        print(f"Connecting to network {SSID}...")
        wlan.connect(SSID, SSID_PASSWORD)
        for _ in range(30):  # attempt wifi connection for 30seconds
            if wlan.isconnected():
                break
            print(".", end="")
            time.sleep(1)
    if wlan.isconnected():
        print("\nConnected to WiFi")
        print(f"network config: {wlan.ifconfig()}")
    else:
        print("\nFailed to connect to WiFi")
        raise


connect_to_wifi()
