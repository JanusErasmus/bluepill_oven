import time
from umqttsimple import MQTTClient
import machine
import ubinascii
import micropython
import network
import esp
import uos
import errno
import utime
from machine import UART

esp.osdebug(None)
import gc
gc.collect()

running = True
uart = UART(0,115200)

HEARTBEAT_TIMEOUT = 4 * 60 * 1000

ssid = 'J-C'
password = 'VictorHanny862'
mqtt_server = 'jcerasmus.ddns.net'

mac_string = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
client_id = ubinascii.hexlify(machine.unique_id())
topic_sub = b'/down/esp/' + mac_string
topic_pub = b'/up/esp/' + mac_string

#give time for UART to settle after boot
time.sleep(2)

print("My MAC is: " + mac_string)

client = None

station = network.WLAN(network.AP_IF)
station.active(False)

station = network.WLAN(network.STA_IF)

station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
  print("We are waiting for WIFI connection")
  time.sleep(1)
  pass

print('Connection successful')
print(station.ifconfig())

led = machine.Pin(13, machine.Pin.OUT)
led.on()
relay = machine.Pin(12, machine.Pin.OUT)
relay.off()

# Complete project details at https://RandomNerdTutorials.com

def sub_cb(topic, msg):
  print((topic, msg))
  if topic == topic_sub:
    if msg == b'set':
        print('Set relay received')
        relay.on()
        return
    
    if msg == b'reset':
        print('Reset relay received')
        relay.off()
        return

    if msg == b'exit':
        global running
        print("We will reset to terminal")
        running = False
        return

def connect_and_subscribe():
  global client_id, mqtt_server, topic_sub, topic_pub
  client = MQTTClient(client_id, mqtt_server, 1883, 'janus', 'Janus506', keepalive=300)
  client.set_last_will(topic_pub, '{\"msg\":\"offline\"}')
  client.set_callback(sub_cb)
  client.connect()
  client.subscribe(topic_sub)
  print('Connected to %s MQTT broker, subscribed to %s topic' % (mqtt_server, topic_sub))
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  time.sleep(10)
  machine.reset()

try:
  client = connect_and_subscribe()
  led.off()
  client.publish(topic_pub, '{\"msg\":\"online\"}')
except OSError as e:
  print("Exception in connect")
  restart_and_reconnect()

print ("MQTT client connected, listening for messages")

connection_aborted = False
something_fucky = False
rx_buffer = None
heartbeat = utime.ticks_ms() + HEARTBEAT_TIMEOUT

def handleSerialLine(inputLine):
  global running
  global uart
  if inputLine == "exit":     
    uart.write("stopping: \n\r")
    uos.dupterm(uart, 1)
    running = False
    return
  
  uart.write(inputLine)
  uart.write("\n\rKO\n\r")

def publishSerialData(inputLine):
  global client
  global heartbeat
  global uart
  heartbeat = utime.ticks_ms() + HEARTBEAT_TIMEOUT
  client.publish(topic_pub, inputLine)
  uart.write("OK\n\r")
  

# Input frames are framed with 0x7E
def waitSerialFrame():
  global uart
  global rx_buffer
  if uart.any() > 0:
    rx_bytes = uart.read(64)  
    for b in rx_bytes:
      if b == 0x7E: # 126 is a MQTT message
        if rx_bytes is not None and len(rx_bytes) > 0:
          publishSerialData(str(rx_buffer))
          rx_buffer = None
      elif (b == 0x0D) or (b == 0x0A):
        if rx_bytes is not None and len(rx_bytes) > 0:
          handleSerialLine(str(rx_buffer))
          rx_buffer = None
      else:
        if rx_buffer:
          rx_buffer += chr(b)
        else:
          rx_buffer = chr(b)

  if rx_buffer is not None and len(rx_buffer) > 64:
    rx_buffer = None

def handleMQTTclient():
  global client
  global running
  global something_fucky
  global connection_aborted
  try:
    client.check_msg()      
  except OSError as e:
    print("Exception in main loop ", e)
    connection_aborted = True
    something_fucky = True
    running = False

def toggleLED():
  global led
  value = led.value()
  if value == 1:
    led.value(0)
  else:
    led.value(1)

# Disable terminal so we can read input from serial
uos.dupterm(None, 1)

try:
  while running:
    handleMQTTclient()
    toggleLED()
    waitSerialFrame()

    # Flash LED faster while relay is set
    if relay.value() == 1:
      time.sleep(0.3)
      toggleLED()
      time.sleep(0.4)
      toggleLED()
      time.sleep(0.3)
    else:
      time.sleep(1)

    if utime.ticks_ms() > heartbeat:
      heartbeat = utime.ticks_ms() + HEARTBEAT_TIMEOUT
      client.publish(topic_pub, '{\"msg\":\"hb\"}')

except Exception as e:
	print("BAD node!!!" + str(e))
	something_fucky = True


uos.dupterm(uart, 1)

if not connection_aborted:
  client.publish(topic_pub, '{\"msg\":\"offline\"}')
  client.disconnect()

station.disconnect()
led.value(1)
relay.off()

if something_fucky:
  print("Something Fucky, hard reset");
  machine.reset()

print("DONE")

