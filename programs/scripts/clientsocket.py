import socket
import datetime as dt
import time

HOST = "192.168.43.150"
PORT = 12345

def transmit_var(s, var):
    sep ='\n'
    val = str(var) + sep 
    s.send(val.encode("utf-8"))
    print(f"sent {val}")
    

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST,PORT))

    frequency = 868.1
    sf = 7
    bw = 125
    device_list = ['21','14', '13', '10']
    starting_time = time.perf_counter() + 2 
    period = 2
    cycles = 10 

    transmit_var(s, frequency)
    transmit_var(s, sf)
    transmit_var(s, bw)
    transmit_var(s, device_list)
    transmit_var(s, starting_time)
    transmit_var(s, period)
    transmit_var(s, cycles)
