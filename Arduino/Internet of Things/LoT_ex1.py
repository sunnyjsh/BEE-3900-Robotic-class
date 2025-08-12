# pi_uploader_dht11.py
# This script is modified to read humidity and temperature from the DHT11
# Arduino sketch provided by the user.
# VERSION 3: Updated API Key and swapped fields as per user request.

import serial
import requests
import time

# --- Configuration ---
# The serial port your Arduino is connected to.
# On Linux, you can get the ID by typing "ls -l /dev/serial/by-id/" in terminal. 
SERIAL_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_XXXXXXXXXXXXXXXXXXX-if00' 

# BAUD RATE MUST MATCH THE ARDUINO SKETCH (Serial.begin(115200))
BAUD_RATE = 115200

# Your ThingSpeak Write API Key
# Field 1 = Temperature
# Field 2 = Humidity
THINGSPEAK_API_KEY = 'API KEY' # <--- Updated with your key

# ThingSpeak API URL
THINGSPEAK_URL = 'https://api.thingspeak.com/update'

def check_internet_connection():
    """
    Checks for a live internet connection by trying to connect to Google.
    Returns True if connected, False otherwise.
    """
    try:
        # Use a reliable and fast-responding server for the check
        requests.get("http://www.google.com", timeout=5)
        print("Internet connection verified.")
        return True
    except (requests.ConnectionError, requests.Timeout):
        print("--- NO INTERNET CONNECTION ---")
        print("The script cannot reach the internet. Please check:")
        print("  1. Is the Raspberry Pi connected to your network (WiFi or Ethernet)?")
        print("  2. Can the Raspberry Pi access the internet? Try running 'ping google.com' in the terminal.")
        return False

def upload_to_thingspeak(humidity, temperature):
    """
    Uploads sensor data to ThingSpeak.
    """
    # Note: field1 is Temperature, field2 is Humidity
    payload = {'api_key': THINGSPEAK_API_KEY, 'field1': temperature, 'field2': humidity}
    try:
        response = requests.get(THINGSPEAK_URL, params=payload)
        if response.status_code == 200:
            print(f"Data uploaded successfully! humidity={humidity}, temperature={temperature}")
        else:
            # Provide more info if ThingSpeak returns an error
            print(f"Error from ThingSpeak. Status code: {response.status_code}, Response: {response.text}")
    except requests.exceptions.RequestException as e:
        # This error is now more likely a temporary network blip, since we checked at the start
        print(f"An error occurred during the request: {e}")
        print("This could be a temporary network issue. The script will continue to try.")

# --- Main Loop ---
if __name__ == "__main__":
    # First, check for an internet connection before doing anything else.
    if not check_internet_connection():
        # Exit the script if there's no internet.
        exit()

    # Initialize serial connection with a try-except block
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        # Allow time for the serial port to open and Arduino to reset
        time.sleep(2)
        print(f"Serial connection established on {SERIAL_PORT} at {BAUD_RATE} baud.")
        print("Listening for data from Arduino...")

        while True:
            # Check if there is data in the serial buffer
            if ser.in_waiting > 0:
                # Read a line of data from the Arduino
                line = ser.readline().decode('utf-8').strip()

                # Make sure the line is not empty
                if line:
                    print(f"Received from Arduino: \"{line}\"")
                    # The expected successful line is like: "DHT11, 	OK,	54.0,	23.5"
                    # We split the line by the comma character
                    parts = line.split(',')

                    # A successful read should have 3 or more parts.
                    if len(parts) >= 3:
                        try:
                            # The second to last part is humidity, last part is temperature.
                            humidity_val = parts[-2].strip()
                            temperature_val = parts[-1].strip()
                            
                            # A final check to ensure we have valid numbers
                            float(humidity_val)
                            float(temperature_val)

                            # Upload the data
                            upload_to_thingspeak(humidity_val, temperature_val)

                        except (ValueError, IndexError) as e:
                            # This will catch lines that don't have the expected data format,
                            # like the header lines from the Arduino's setup() function.
                            print(f"Could not parse data from line: \"{line}\". Skipping. Error: {e}")
                    else:
                        print(f"Line does not have enough parts: \"{line}\". Skipping.")
            
            # A small delay to prevent the script from using 100% CPU
            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
        print("Please check the port, permissions, and make sure the Arduino is connected.")
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        # Ensure the serial port is closed on exit
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

