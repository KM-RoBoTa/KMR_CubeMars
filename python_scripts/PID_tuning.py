# Module Imports
import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
import matplotlib as mpl

mpl.style.use('default')


times_list = []
commands_list = []
fbckPos_list = []

Kp_list = []
Kd_list = []


with open('PID_tuning.csv', newline='') as csvfile:

    reader = csv.reader(csvfile)

    for row in reader:
        times_list.append(float(row[0]))
        commands_list.append(float(row[1]))
        fbckPos_list.append(float(row[2]))
        Kp_list.append(float(row[3]))        
        Kd_list.append(float(row[4]))

# Converting lists to arrays
times = np.array(times_list)
commands = np.array(commands_list)
fbckPos = np.array(fbckPos_list)
Kps = np.array(Kp_list)
Kds = np.array(Kd_list)

# Extract Kp and Kd
Kp = Kps[0]
Kd = Kds[0] 

# Plot predicted goal front Rmax
fig, ax = plt.subplots()
ax.plot(times, commands, label = "Command")
ax.plot(times, fbckPos, label = "Feedback")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Angle [rad]")
plt.title('PID tuning (Kp = %1.0f' %Kp + ', Kp = %1.0f)' %Kd)
plt.legend()

fig.savefig("PID_tuning.png")



# Second part: tracking

times_list = []
commands_list = []
fbckPos_list = []

Kp_list = []
Kd_list = []
with open('tracking.csv', newline='') as csvfile:

    reader = csv.reader(csvfile)

    for row in reader:
        times_list.append(float(row[0]))
        commands_list.append(float(row[1]))
        fbckPos_list.append(float(row[2]))
        Kp_list.append(float(row[3]))        
        Kd_list.append(float(row[4]))

# Converting lists to arrays
times = np.array(times_list)
commands = np.array(commands_list)
fbckPos = np.array(fbckPos_list)
Kps = np.array(Kp_list)
Kds = np.array(Kd_list)

# Extract Kp and Kd
Kp = Kps[0]
Kd = Kds[0] 

# Plot predicted goal front Rmax
fig, ax = plt.subplots()
ax.plot(times, commands, label = "Command")
ax.plot(times, fbckPos, label = "Feedback")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Angle [rad]")
plt.title('Tracking (Kp = %1.0f' %Kp + ', Kp = %1.0f)' %Kd)
plt.legend()

fig.savefig("tracking.png")

plt.show()