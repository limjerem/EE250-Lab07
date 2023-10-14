import time
import RPi.GPIO as GPIO
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Constants for GPIO and ADC pins
LED_PIN = 11
LIGHT_SENSOR_CHANNEL = 0
SOUND_SENSOR_CHANNEL = 1

# Using physical pin 11 to blink an LED
GPIO.setmode(GPIO.BOARD)
chan_list = [11]
GPIO.setup(chan_list, GPIO.OUT)

# Hardware SPI configuration:
SPI_PORT = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Constants for time intervals
LED_BLINK_DELAY = 0.5
SENSOR_READ_INTERVAL = 0.1
LED_BLINK_SHORT_DELAY = 0.2
LED_ON_DURATION = 0.1

# Thresholds for light and sound sensors (adjust these values based on experimentation)
LIGHT_THRESHOLD = 50  # Adjust this value based on your light sensor's behavior
SOUND_THRESHOLD = 70  # Adjust this value based on your sound sensor's behavior

try:
    # LED blinks 5 times with 500ms on/off times at the beginning of the test
    for _ in range(5):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(LED_BLINK_DELAY)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(LED_BLINK_DELAY)

    # Read the light sensor at 100 ms intervals for 5 seconds
    start_time = time.time()
    while time.time() - start_time < 5:
        light_value = mcp.read_adc(LIGHT_SENSOR_CHANNEL)
        light_status = "bright" if light_value > LIGHT_THRESHOLD else "dark"
        print(f"Light Sensor: Raw Value={light_value}, Status={light_status}")
        time.sleep(SENSOR_READ_INTERVAL)

    # LED blinks 4 times with 200ms on/off times between sensor tests
    for _ in range(4):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(LED_BLINK_SHORT_DELAY)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(LED_BLINK_SHORT_DELAY)

    # Read the sound sensor at 100 ms intervals for 5 seconds
    start_time = time.time()
    while time.time() - start_time < 5:
        sound_value = mcp.read_adc(SOUND_SENSOR_CHANNEL)
        print(f"Sound Sensor: Raw Value={sound_value}")

        # Check if sound threshold is exceeded
        if sound_value > SOUND_THRESHOLD:
            GPIO.output(LED_PIN, GPIO.HIGH)
            time.sleep(LED_ON_DURATION)
            GPIO.output(LED_PIN, GPIO.LOW)

        time.sleep(SENSOR_READ_INTERVAL)

except KeyboardInterrupt:
    GPIO.cleanup()
