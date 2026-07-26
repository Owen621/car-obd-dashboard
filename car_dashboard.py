import obd
import time

# Connect to the dongle over WiFi
connection = obd.OBD("socket://192.168.0.10:35000")

if not connection.is_connected():
    print("Failed to connect - check WiFi connection to dongle and IP/port")
    exit()

print("Connected! Reading data...\n")

while True:
    rpm = connection.query(obd.commands.RPM)
    speed = connection.query(obd.commands.SPEED)
    coolant_temp = connection.query(obd.commands.COOLANT_TEMP)
    throttle = connection.query(obd.commands.THROTTLE_POS)

    print(f"RPM: {rpm.value}")
    print(f"Speed: {speed.value}")
    print(f"Coolant Temp: {coolant_temp.value}")
    print(f"Throttle: {throttle.value}")
    print("-" * 30)

    time.sleep(1)