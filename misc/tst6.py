import socket

UDP_IP = "192.168.11.2"
UDP_PORT = 9750
MESSAGE1 = "xa-0050b-0000c-0000d+0000e+0000f+0000z"
# MESSAGE1 = "xg+0500h+0300i+0000l+0000z"
MESSAGE0 = "xm4sz"
# MESSAGE0 = "xm6sz"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE1

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(MESSAGE0, (UDP_IP, UDP_PORT))
sock.sendto(MESSAGE1, (UDP_IP, UDP_PORT))