import numpy as np

gps_val=np.loadtxt('Graph1.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
accel_val=np.loadtxt('Graph2.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]

gps_std=np.std(gps_val)
accel_std=np.std(accel_val)

print('MeasuredStdDev_GPSPosXY = ,',gps_std)
print('MeasuredStdDev_AccelXY = ,',accel_std)
