import serial,time
import threading
from threading import Lock

def keyboard():
    global serial, mutex
    while(True):
        mutex.acquire()
        serial.write(b"d")
        mutex.release()
        time.sleep(0.001)
        print(serial.read(50))

serial = serial.Serial("/dev/ttyACM0", 9600, timeout=0.001)
thread1 = threading.Thread(target=keyboard)
mutex = Lock()
thread1.start()

try:
    while(True):
        char = input()
        if(char == 'f'):
            mutex.acquire()
            serial.write(b"f")
            mutex.release()
            time.sleep(0.001)
        elif(char == 'b'):
            mutex.acquire()
            serial.write(b"b")
            mutex.release()
            time.sleep(0.001)
        elif(char == 's'):
            mutex.acquire()
            serial.write(b"s")
            mutex.release()
            time.sleep(0.001)
except KeyboardInterrupt:
    print("exiting...")
    thread1.join()
    exit(1)
