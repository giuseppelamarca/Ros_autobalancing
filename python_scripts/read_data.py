import serial
import matplotlib.pylab as plt
import numpy as np

log = {'Current': [], 'Speed': [], 'Voltage': []}
log_data = -1

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
data = ""
while data != "FINE":
    data = ser.readline()[:-2].decode('utf-8')
    if data == "FINE":
        continue
    if data == 'Current':
        log_data = 'Current'
        print('Current got')
    elif data == 'Speed':
        log_data = 'Speed'
        print('Speed got')
    elif data == 'Voltage':
        log_data = 'Voltage'
        print('Voltage got')
    else:
        if log_data == -1:
            continue
        log[log_data].append(float(data))

for key in log:
    print(len(log[key]))

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

out = moving_average(log['Current'], 10)
plt.plot(log['Voltage'])
plt.plot(log['Speed'])
#plt.plot(log['Current'])
plt.plot(out)
plt.show()