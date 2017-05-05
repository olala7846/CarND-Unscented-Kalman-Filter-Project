import matplotlib.pyplot as plt
import pandas as pd


data_frame = pd.read_table('./build/output.txt')

radar_nis = []
lidar_nis = []

for index, row in data_frame.iterrows():
    sensor_type = row['sensor_type']
    nis = row['NIS']
    if sensor_type == 'lidar':
        lidar_nis.append(nis)
    else:
        radar_nis.append(nis)

plt.figure(figsize=(12, 9))
plt.plot(radar_nis)
plt.show()
