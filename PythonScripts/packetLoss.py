import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV files into DataFrames
open_250_SF7 = pd.read_csv("Dataset_final/open_sf7_250m.csv")
open_500_SF7 = pd.read_csv("Dataset_final/open_sf7_500m.csv")
open_1000_SF7 = pd.read_csv("Dataset_final/open_sf7_1000m.csv")
open_250_SF10 = pd.read_csv("Dataset_final/open_sf10_250m.csv")
open_500_SF10 = pd.read_csv("Dataset_final/open_sf10_500m.csv")
open_1000_SF10 = pd.read_csv("Dataset_final/open_sf10_1000m.csv")
wood_250_SF7 = pd.read_csv("Dataset_final/wood_sf7_250m.csv")
wood_500_SF7 = pd.read_csv("Dataset_final/wood_sf7_500m.csv")
wood_250_SF10 = pd.read_csv("Dataset_final/wood_sf10_250m.csv")
wood_500_SF10 = pd.read_csv("Dataset_final/wood_sf10_500m.csv")
wood_1000_SF10 = pd.read_csv("Dataset_final/wood_sf10_1000m.csv")

# Calculate error packet loss percentage for each DataFrame
error_packet_loss_percentage_open_250_SF7 = (
    open_250_SF7['rssi'].isnull().sum() / len(open_250_SF7['rssi'])) * 100

error_packet_loss_percentage_open_500_SF7 = (
    open_500_SF7['rssi'].isnull().sum() / len(open_500_SF7['rssi'])) * 100

error_packet_loss_percentage_open_1000_SF7 = (
    open_1000_SF7['rssi'].isnull().sum() / len(open_1000_SF7['rssi'])) * 100

error_packet_loss_percentage_open_250_SF10 = (
    open_250_SF10['rssi'].isnull().sum() / len(open_250_SF10['rssi'])) * 100

error_packet_loss_percentage_open_500_SF10 = (
    open_500_SF10['rssi'].isnull().sum() / len(open_500_SF10['rssi'])) * 100

error_packet_loss_percentage_open_1000_SF10 = (
    open_1000_SF10['rssi'].isnull().sum() / len(open_1000_SF10['rssi'])) * 100

error_packet_loss_percentage_wood_250_SF7 = (
    wood_250_SF7['rssi'].isnull().sum() / len(wood_250_SF7['rssi'])) * 100

error_packet_loss_percentage_wood_500_SF7 = (
    wood_500_SF7['rssi'].isnull().sum() / len(wood_500_SF7['rssi'])) * 100

error_packet_loss_percentage_wood_250_SF10 = (
    wood_250_SF10['rssi'].isnull().sum() / len(wood_250_SF10['rssi'])) * 100

error_packet_loss_percentage_wood_500_SF10 = (
    wood_500_SF10['rssi'].isnull().sum() / len(wood_500_SF10['rssi'])) * 100

error_packet_loss_percentage_wood_1000_SF10 = (
    wood_1000_SF10['rssi'].isnull().sum() / len(wood_1000_SF10['rssi'])) * 100

# Create a dictionary of the error packet loss percentage
error_packet_loss_percentage = {
    "open_sf7": (
        error_packet_loss_percentage_open_250_SF7,
        error_packet_loss_percentage_open_500_SF7,
        error_packet_loss_percentage_open_1000_SF7),
    "open_sf10": (
        error_packet_loss_percentage_open_250_SF10,
        error_packet_loss_percentage_open_500_SF10,
        error_packet_loss_percentage_open_1000_SF10),
    "wood_sf7": (
        error_packet_loss_percentage_wood_250_SF7,
        error_packet_loss_percentage_wood_500_SF7,
        None),
    "wood_sf10": (
        error_packet_loss_percentage_wood_250_SF10,
        error_packet_loss_percentage_wood_500_SF10,
        error_packet_loss_percentage_wood_1000_SF10)
}

print(error_packet_loss_percentage)


def round_tuple_values(tup):
    return tuple(round(val, 2) if isinstance(val, float) else val for val in tup)


# Round floats in the dictionary values
rounded_error_packet_loss_percentage = {key: round_tuple_values(
    value) for key, value in error_packet_loss_percentage.items()}

# Display the updated dictionary with rounded float values
print(rounded_error_packet_loss_percentage)

# Update the rounded_error_packet_loss_percentage_display dictionary to handle 'None' values
rounded_error_packet_loss_percentage_display = {key: [
    val if val is not None else 'None' for val in value] for key, value in rounded_error_packet_loss_percentage.items()}


distances = ['250', '500', '1000']
width = 0.22
x_axis = np.arange(len(distances))
multiplier = 0

fig, ax = plt.subplots(layout='constrained')

colors = ['palegreen', 'darkolivegreen', 'skyblue', 'steelblue']

for attribute, measurement in rounded_error_packet_loss_percentage_display.items():
    offset = width * multiplier
    measurement = [0 if val is 'None' else val for val in measurement]
    rects = ax.bar(x_axis + offset, measurement, width, align='center',
                   label=attribute, color=colors[multiplier])
    ax.bar_label(rects, labels=[
                 '' if val == 0 else val for val in measurement], padding=3, fontsize=18)
    multiplier += 1

    ax.text(x_axis[0], 1, '0', ha='center', fontsize=18)
    ax.text(x_axis[0] + width, 1, '0', ha='center', fontsize=18)
    ax.text(x_axis[2] + 2*width, 1, 'None', ha='center', fontsize=18)
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Packet loss [%]', fontsize=18)
ax.set_xlabel('Distance [m]', fontsize=18)
# ax.set_title('Comparison Packet Loss', fontsize=22)
ax.set_xticks(x_axis + 1.5 * width)
ax.set_xticklabels(distances)
legend = ax.legend(loc='upper right', ncols=2)
for text in legend.get_texts():
    text.set_fontsize(16)

ax.xaxis.set_tick_params(labelsize=18)
ax.yaxis.set_tick_params(labelsize=18) 

ax.set_ylim(0, 100)

plt.tight_layout()
plt.show()
