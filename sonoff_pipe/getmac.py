import network
import ubinascii

print(ubinascii.hexlify(network.WLAN().config('mac'),':').decode())




