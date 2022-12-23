import socket 
from socket import AF_INET, SOCK_DGRAM
from json import dumps, loads 
from time import sleep


ip = '0.0.0.0'
port = 37171

b2s = dumps('hello server', indent=2).encode('utf-8')

s = socket.socket(AF_INET, SOCK_DGRAM)

while True:
    s.sendto(b2s,(ip,port))
    feedback = loads(s.recvfrom(2048)[0].decode('utf-8'))
    print(feedback)
    sleep(.1)
