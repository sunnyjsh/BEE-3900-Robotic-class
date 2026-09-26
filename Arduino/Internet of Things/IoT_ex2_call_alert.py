# IoT_ex2_call_alert.py
# Builds on IoT_ex1.py: reads DHT11 humidity and temperature from the Arduino,
# uploads them to ThingSpeak, and PHONES someone (Twilio voice call) when the
# temperature goes above a limit, e.g. 25 degrees C.
#
# Setup (once, on the Raspberry Pi):
#   pip install pyserial requests twilio
#   (on newer Raspberry Pi OS you may need: pip install --break-system-packages ...)
#
# Secrets are read from environment variables so they never end up on GitHub.
# Put them in a file called alert.env (see alert.env.example), then run:
#   set -a; source alert.env; set +a
#   python3 IoT_ex2_call_alert.py
#
# Test without making a real call:
#   DRY_RUN=1 python3 IoT_ex2_call_alert.py

import os
import time

import requests
import serial

# --- Configuration ---
SERIAL_PORT = os.environ.get(
    "SERIAL_PORT",
    "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_XXXXXXXXXXXXXXXXXXX-if00",
)
BAUD_RATE = 115200  # must match Serial.begin(115200) in dht11_thingspeak.ino

# ThingSpeak: Field 1 = Temperature, Field 2 = Humidity
THINGSPEAK_API_KEY = os.environ.get("THINGSPEAK_API_KEY", "API KEY")
THINGSPEAK_URL = "https://api.thingspeak.com/update"

# --- Alert settings ---
TEMP_LIMIT = float(os.environ.get("TEMP_LIMIT", "25.0"))   # call when temperature > this (deg C)
READINGS_IN_A_ROW = 2      # must be above the limit this many readings in a row (ignores one noisy spike)
RESET_BELOW = TEMP_LIMIT - 1.0   # alarm resets once temperature drops below this
REPEAT_MINUTES = 30        # while it stays hot, call again at most this often

# Twilio (voice call) settings, from environment variables
TWILIO_ACCOUNT_SID = os.environ.get("TWILIO_ACCOUNT_SID", "")
TWILIO_AUTH_TOKEN = os.environ.get("TWILIO_AUTH_TOKEN", "")
TWILIO_FROM_NUMBER = os.environ.get("TWILIO_FROM_NUMBER", "")  # your Twilio number, e.g. +16075550100
ALERT_TO_NUMBER = os.environ.get("ALERT_TO_NUMBER", "")        # person to call, e.g. +16075550199
DRY_RUN = os.environ.get("DRY_RUN", "0") == "1"                # 1 = print instead of calling


class TemperatureAlarm:
    """
    Decides WHEN to call. Kept separate from the phone code so it is easy to test.

    - Calls after READINGS_IN_A_ROW readings above TEMP_LIMIT.
    - While it stays hot, calls again only every REPEAT_MINUTES.
    - Resets when the temperature falls below RESET_BELOW.
    """

    def __init__(self, limit, readings_in_a_row, reset_below, repeat_seconds):
        self.limit = limit
        self.readings_in_a_row = readings_in_a_row
        self.reset_below = reset_below
        self.repeat_seconds = repeat_seconds
        self.hot_count = 0
        self.last_call_time = None  # None = no call made since the last reset

    def should_call(self, temperature, now):
        if temperature > self.limit:
            self.hot_count += 1
        else:
            self.hot_count = 0
            if temperature < self.reset_below:
                self.last_call_time = None  # back to normal: arm the alarm again
            return False

        if self.hot_count < self.readings_in_a_row:
            return False
        if self.last_call_time is not None and now - self.last_call_time < self.repeat_seconds:
            return False

        self.last_call_time = now
        return True


def make_phone_call(temperature):
    """Phones ALERT_TO_NUMBER and reads the temperature out loud."""
    message = (
        f"Warning from your sensor. The temperature is {temperature:.1f} degrees Celsius, "
        f"which is above the limit of {TEMP_LIMIT:.0f} degrees."
    )
    twiml = f'<Response><Say>{message}</Say><Pause length="1"/><Say>{message}</Say></Response>'

    if DRY_RUN:
        print(f"[DRY RUN] Would call {ALERT_TO_NUMBER or '(no number set)'}: {message}")
        return

    if not all([TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN, TWILIO_FROM_NUMBER, ALERT_TO_NUMBER]):
        print("Cannot call: set TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN, TWILIO_FROM_NUMBER and ALERT_TO_NUMBER.")
        return

    try:
        from twilio.rest import Client  # imported here so DRY_RUN works without the library
        client = Client(TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN)
        call = client.calls.create(to=ALERT_TO_NUMBER, from_=TWILIO_FROM_NUMBER, twiml=twiml)
        print(f"Calling {ALERT_TO_NUMBER} (call SID {call.sid})")
    except Exception as e:
        # A failed call must not stop the data logging.
        print(f"Phone call failed: {e}")


def check_internet_connection():
    try:
        requests.get("http://www.google.com", timeout=5)
        print("Internet connection verified.")
        return True
    except (requests.ConnectionError, requests.Timeout):
        print("--- NO INTERNET CONNECTION --- Check the Pi's WiFi/Ethernet, then try 'ping google.com'.")
        return False


def upload_to_thingspeak(humidity, temperature):
    payload = {"api_key": THINGSPEAK_API_KEY, "field1": temperature, "field2": humidity}
    try:
        response = requests.get(THINGSPEAK_URL, params=payload, timeout=10)
        if response.status_code == 200:
            print(f"Data uploaded successfully! humidity={humidity}, temperature={temperature}")
        else:
            print(f"Error from ThingSpeak. Status code: {response.status_code}, Response: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"An error occurred during the request: {e}")


def parse_line(line):
    """'DHT11, \tOK,\t54.0,\t23.5' -> (54.0, 23.5), or None if the line has no data."""
    parts = line.split(",")
    if len(parts) < 3:
        return None
    try:
        humidity = float(parts[-2].strip())
        temperature = float(parts[-1].strip())
    except ValueError:
        return None
    return humidity, temperature


# --- Main Loop ---
if __name__ == "__main__":
    if not check_internet_connection():
        exit()

    alarm = TemperatureAlarm(TEMP_LIMIT, READINGS_IN_A_ROW, RESET_BELOW, REPEAT_MINUTES * 60)
    print(f"Alert: call {ALERT_TO_NUMBER or '(not set)'} when temperature > {TEMP_LIMIT} C"
          f"{' [DRY RUN]' if DRY_RUN else ''}")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)  # Arduino resets when the port opens
        print(f"Serial connection established on {SERIAL_PORT} at {BAUD_RATE} baud.")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(f'Received from Arduino: "{line}"')
                    reading = parse_line(line)
                    if reading is None:
                        print("  (no data in this line, skipping)")
                    else:
                        humidity, temperature = reading
                        upload_to_thingspeak(humidity, temperature)
                        if alarm.should_call(temperature, time.time()):
                            make_phone_call(temperature)
            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")
