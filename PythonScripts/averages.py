import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# Load your data here
open_250_SF7 = pd.read_csv("Dataset_final/open_sf7_250m.csv")
open_500_SF7 = pd.read_csv("Dataset_final/open_sf7_500m.csv")
open_1000_SF7 = pd.read_csv("Dataset_final/open_sf7_1000m.csv")
wood_250_SF7 = pd.read_csv("Dataset_final/wood_sf7_250m.csv")
wood_500_SF7 = pd.read_csv("Dataset_final/wood_sf7_500m.csv")
open_250_SF10 = pd.read_csv("Dataset_final/open_sf10_250m.csv")
open_500_SF10 = pd.read_csv("Dataset_final/open_sf10_500m.csv")
open_1000_SF10 = pd.read_csv("Dataset_final/open_sf10_1000m.csv")
wood_250_SF10 = pd.read_csv("Dataset_final/wood_sf10_250m.csv")
wood_500_SF10 = pd.read_csv("Dataset_final/wood_sf10_500m.csv")
wood_1000_SF10 = pd.read_csv("Dataset_final/wood_sf10_1000m.csv")

# calculate averages
open_250_SF7_avg = open_250_SF7['rssi'].mean()
open_500_SF7_avg = open_500_SF7['rssi'].mean()
open_1000_SF7_avg = open_1000_SF7['rssi'].mean()
wood_250_SF7_avg = wood_250_SF7['rssi'].mean()
wood_500_SF7_avg = wood_500_SF7['rssi'].mean()
open_250_SF10_avg = open_250_SF10['rssi'].mean()
open_500_SF10_avg = open_500_SF10['rssi'].mean()
open_1000_SF10_avg = open_1000_SF10['rssi'].mean()
wood_250_SF10_avg = wood_250_SF10['rssi'].mean()
wood_500_SF10_avg = wood_500_SF10['rssi'].mean()
wood_1000_SF10_avg = wood_1000_SF10['rssi'].mean()

print('open250SF7AVG',open_250_SF7_avg)
print('open250SF7MIN', open_250_SF7['rssi'].min())
print('open250SF7MAX', open_250_SF7['rssi'].max())
print('open500SF7AVG',open_500_SF7_avg)
print('open500SF7MIN', open_500_SF7['rssi'].min())
print('open500SF7MAX', open_500_SF7['rssi'].max())
print('open1000SF7AVG',open_1000_SF7_avg)
print('open1000SF7MIN', open_1000_SF7['rssi'].min())
print('open1000SF7MAX', open_1000_SF7['rssi'].max())
print('--------------------------------------------')
print('open250SF10AVG',open_250_SF10_avg)
print('open250SF10MIN', open_250_SF10['rssi'].min())
print('open250SF10MAX', open_250_SF10['rssi'].max())
print('open500SF10AVG',open_500_SF10_avg)
print('open500SF10MIN', open_500_SF10['rssi'].min())
print('open500SF10MAX', open_500_SF10['rssi'].max())
print('open1000SF10AVG',open_1000_SF10_avg)
print('open1000SF10MIN', open_1000_SF10['rssi'].min())
print('open1000SF10MAX', open_1000_SF10['rssi'].max())
print('--------------------------------------------')
print('wood250SF7AVG',wood_250_SF7_avg)
print('wood250SF7MIN', wood_250_SF7['rssi'].min())
print('wood250SF7MAX', wood_250_SF7['rssi'].max())
print('wood500SF7AVG',wood_500_SF7_avg)
print('wood500SF7MIN', wood_500_SF7['rssi'].min())
print('wood500SF7MAX', wood_500_SF7['rssi'].max())
print('--------------------------------------------')
print('wood250SF10AVG',wood_250_SF10_avg)
print('wood250SF10MIN', wood_250_SF10['rssi'].min())
print('wood250SF10MAX', wood_250_SF10['rssi'].max())
print('wood500SF10AVG',wood_500_SF10_avg)
print('wood500SF10MIN', wood_500_SF10['rssi'].min())
print('wood500SF10MAX', wood_500_SF10['rssi'].max())
print('wood1000SF10AVG',wood_1000_SF10_avg)
print('wood1000SF10MIN', wood_1000_SF10['rssi'].min())
print('wood1000SF10MAX', wood_1000_SF10['rssi'].max())
print('--------------------------------------------')




open_SF7 = [open_250_SF7_avg, open_500_SF7_avg, open_1000_SF7_avg]
wood_SF7 = [wood_250_SF7_avg, wood_500_SF7_avg]
open_SF10 = [open_250_SF10_avg, open_500_SF10_avg, open_1000_SF10_avg]
wood_SF10 = [wood_250_SF10_avg, wood_500_SF10_avg, wood_1000_SF10_avg]

distance = [250, 500, 1000]

distanceText = ["250", "500", "1000"]

# Plotting the data
plt.plot(distanceText, open_SF7, marker='o', label="Open Land SF7", color='palegreen', linewidth=3)
plt.plot(distanceText[0:2], wood_SF7, marker='o', label="Wood Land SF7", color='darkolivegreen', linewidth=3)
plt.plot(distanceText, open_SF10, marker='o', label="Open Land SF10", color='skyblue', linewidth=3)
plt.plot(distanceText, wood_SF10, marker='o', label="Wood Land SF10", color='steelblue', linewidth=3)

for i, txt in enumerate(open_SF7):
    plt.annotate(round(txt, 2), (distanceText[i], open_SF7[i]),
                 textcoords="offset points", xytext=(-10, -18), ha='center', fontsize=16)

for i, txt in enumerate(wood_SF7):
    plt.annotate(round(txt, 2), (distanceText[i], wood_SF7[i]),
                 textcoords="offset points", xytext=(-8, -28), ha='center', fontsize=16)

for i, txt in enumerate(open_SF10):
    plt.annotate(round(txt, 2), (distanceText[i], open_SF10[i]), textcoords="offset points", xytext=(
        8, 8), ha='center', fontsize=16)

for i, txt in enumerate(wood_SF10):
    plt.annotate(round(txt, 2), (distanceText[i], wood_SF10[i]), textcoords="offset points", xytext=(
        7, 15), ha='center', fontsize=16)


# # Set x-axis ticks and limit
# plt.xticks(distance, distance)  # Set x-axis ticks to show 250, 500, and 1000
# plt.xlim(200, 1100)  # Adjust x-axis limit slightly for better visualization

# Adjust interval as needed

plt.rcParams.update({'font.size': 14})

# plt.title("Averages Values RSSI", fontsize=22)
plt.legend(loc='upper right', fontsize=16)
plt.xlabel("Distance [m]", fontsize=18)
plt.ylabel("RSSI [dBm]", fontsize=18)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.show()
