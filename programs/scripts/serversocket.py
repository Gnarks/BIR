import socket
import datetime as dt
import time

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 12343  # Port to listen on (non-privileged ports are > 1023)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))

sock.listen(5)
conn, addr = sock.accept()
print("Connected established with transmitter")

def get_next_var_from_sock(socket):

    sep = '\n'
    data = socket.recv(1).decode("utf-8")
    buf = data
    # if deconnected
    if data == '':
        return "ERROR"

    # while not seen the separator
    while sep not in buf and data:
        buf += socket.recv(1).decode("utf-8")
        print(f"buf {buf}")

    if buf != '':
        data = eval(buf)
        print(f"received : {data}")
        return data



with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"frequency is {get_next_var_from_sock(conn)* 1e6}")
        print(f"sf is {get_next_var_from_sock(conn)}")
        print(f"bw is {get_next_var_from_sock(conn) * 1e3}")
        print(f"device list is {get_next_var_from_sock(conn)}")
        print(f"strating_time is {get_next_var_from_sock(conn)}")
        print(f"period is {get_next_var_from_sock(conn)}")
        print(f"cycles is {get_next_var_from_sock(conn)}")




