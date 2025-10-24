import socket
import controller2
from time import sleep


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#addr = ('10.242.187.175',3000)
addr = ('192.168.0.10',3000)
conts = controller2.find_controllers()
print(conts)
cont = controller2.LogitechController(conts[0])

while True:
    x = 16384 * cont.button_data.dpad_right - 16384 * cont.button_data.dpad_left
    y = 16384 * cont.button_data.dpad_up - 16384 * cont.button_data.dpad_down
    print(x,y)
    #print(cont.button_data.stick_left_x, cont.button_data.stick_left_y)
    sock.sendto(b"x"+str(x).encode("utf-8")+b"\r\n", addr)
    sock.sendto(b"y"+str(y).encode("utf-8")+b"\r\n", addr)
    sleep(0.05)
