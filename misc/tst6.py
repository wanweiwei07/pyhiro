import socket

UDP_IP = "192.168.11.2"
UDP_PORT = 9750
MESSAGE1 = "xg+0000h+1000i+0000j+0000k+0000l+0000z"
# MESSAGE = "xm1z"
MESSAGE0 = "xm3z"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(MESSAGE0, (UDP_IP, UDP_PORT))
sock.sendto(MESSAGE1, (UDP_IP, UDP_PORT))