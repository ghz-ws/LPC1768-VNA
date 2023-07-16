import serial
import time
import numpy
import csv
import matplotlib.pyplot as plt

vna=serial.Serial("COM17",115200)
Start_Freq=100  ##MHz unit
Stop_Freq=110   ##MHz unit
Step_Freq=1     ##MHz unit
Wait=0.001      ##sec
Port=2        ##Stimulus port. S11(0) S12(1) S13(2) S14(3), S21(4) S22(5) S23(6) S24(7), S31(8) S32(9) S33(10) S34(11), S41(12) S42(13) S43(14) S44(15)
att=0           ##0=-0dB, 1=-10dB, 2=-20dB, 3=-30dB
integ=1         ##integ time

step=int((Stop_Freq-Start_Freq)/Step_Freq)
data=numpy.zeros((step,3))
for i in range(step):
    freq=Start_Freq+i*Step_Freq
    buf=f'{freq*1000:07}'+f'{integ:+02}'+f'{att:01}'+f'{Port:+02}'
    vna.write(buf.encode())
    time.sleep(Wait)
    buf=vna.read(9)
    re=float(buf)
    buf=vna.read(9)
    im=float(buf)
    print('Freq=',freq,'MHz, Re=',re,'Im=',im)
    data[i][0]=float(freq)
    data[i][1]=float(re)
    data[i][2]=float(im)
    
with open('data.csv','w',newline="") as f:
    writer=csv.writer(f)
    writer.writerows(data)
    
plt.subplot(1,2,1)
plt.plot(data[:,0],data[:,1])
plt.subplot(1,2,2)
plt.plot(data[:,0],data[:,2])
plt.show()