import time
from umqttsimple import MQTTClient
import machine
import ubinascii
import micropython
import network
import esp
esp.osdebug(None)
import gc
gc.collect()

running = True

ssid = 'J_C'
password = 'VictorHanny874'
mqtt_server = 'jcerasmus.ddns.net'
#EXAMPLE IP ADDRESS
#mqtt_server = '192.168.1.144'
mac_string = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
client_id = ubinascii.hexlify(machine.unique_id())
topic_sub = b'/down/' + mac_string
topic_pub = b'/up/' + mac_string

print("My MAC is: " + mac_string)

last_message = 0
message_interval = 1

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

# Complete project details at https://RandomNerdTutorials.com

def sub_cb(topic, msg):
  print((topic, msg))
  if topic == topic_sub:
    if msg == b'set':
        print('Set relay received')
        relay.on()
    
    if msg == b'reset':
        print('Reset relay received')
        relay.off()

    if msg == b'exit':
        global running
        print("We will reset to terminal")
        running = False

def connect_and_subscribe():
  global client_id, mqtt_server, topic_sub
  client = MQTTClient(client_id, mqtt_server, 1883, 'janus', 'Janus506')
  client.set_callback(sub_cb)
  client.set_last_will(topic_sub, 'offline')
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
  client.publish(topic_pub, 'Online')
except OSError as e:
  print("Exception in connect")
  restart_and_reconnect()

print ("MQTT client connected, listening for messages")

while running:
  try:
    client.check_msg()
    if (time.time() - last_message) > message_interval:
      if relay.value() == 1:      
        value = led.value()
        if value == 1:
          led.value(0)
        else:
          led.value(1)


      last_message = time.time()
      
  except OSError as e:
    print("Exception in main loop")
    #restart_and_reconnect()

  print("running")
  time.sleep(1)
  #running = False
  pass

client.disconnect()
station.disconnect()

print("DONE")