import serial, time
serial_port = serial.Serial()
serial_port.port = 'COM3'
serial_port.baudrate = 115200
serial_port.open()
time.sleep(0.5)
print('writing')

serial_port.write('S041000'.encode('utf-8'))

#time.sleep(0.1)
serial_port.write('G'.encode())
print('written')
#print(serial_port.read())
# while True:
#     print(serial_port.read())

if 1:
    serial_port.write('B6250000'.encode())
    #serial_port.write('B49600'.encode())
    print(serial_port.readline())
    serial_port.baudrate = 250000
    time.sleep(0.5)

serial_port.write('G'.encode())
#serial_port.readline()
#serial_port.read_until('\r')
print(serial_port.read_all())
serial_port.setDTR(0)
#serial_port.setRTS(0)
serial_port.setDTR(1)
#serial_port.setRTS(1)

iter = 0

while True:
    iter += 1
    start = time.clock()
    serial_port.write('R'.encode())
    #time.sleep(0.01)
    #line = serial_port.readline()
    line = serial_port.read_until('\r\n'.encode())
    print(line)
    #print(serial_port.readline())
    print(time.clock() - start)
    # if iter%10 == 0:
    #         print(time.clock() - start)


