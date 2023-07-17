import matplotlib.pyplot as plt
#import csv
import pandas as pd
import numpy as np
from scipy.signal import find_peaks
from scipy.signal import find_peaks_cwt
from matplotlib.ticker import MaxNLocator
from scipy.ndimage import uniform_filter1d

dataframe = pd.read_csv('07_07_2023__11_27_17.csv')

colors = ['blue', 'green', 'purple', 'tan', 'pink', 'red']
x = dataframe['CurrentAccelX']
y = dataframe['CurrentAccelY']
z = dataframe['CurrentAccelZ']
angvelY = dataframe['CurrentAngVelY']
time = dataframe['TimeInMs']
fig, ax = plt.subplots(tight_layout=True)
#fig = plt.figure(figsize =(6, 6))
#ax = fig.add_subplot(111)
modifiedTime = time.sub(time[0])

#plt.yticks(fontsize=15)
#plt.xticks(fontsize=15)
smoothedY = y.pow(2).ewm(span = 360).mean()
#y_smooth = uniform_filter1d(angvelY,size=360)

peaks = find_peaks(smoothedY, distance=150)
print("Peaks position:", peaks[0])

#sumofsquares = np.sqrt(x.pow(2) + y.pow(2) + z.pow(2))

ax.plot(modifiedTime, smoothedY, label = "Angular Velocity (Y Axis)")

for peak in peaks[0]:
    ax.axvline(modifiedTime[peak], color='b', label='axvline - full height')

#print(len(peaks[0]))
print("Respiratory Frequency was one breath every: " + str(modifiedTime[len(modifiedTime)-1]/len(peaks[0])) + " milliseconds.")

#plt.legend(loc = 'lower left', fontsize = '15', title = 'Facial Detection Confidence Level', title_fontsize = '15')
#plt.title("Running Bystander Protection Rate (10 second avg)", fontsize = 15, wrap=True)
ax.set_xlabel('Time (in milliseconds)', fontsize = 15)
ax.set_ylabel('Angular Velocity (Y Axis)', fontsize = 15)
plt.tight_layout()
#plt.savefig('BystanderProtectionRunningTotal90Conf.jpg', format="jpg", bbox_inches="tight")
plt.show()
