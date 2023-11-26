import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# Load your data here
open_250_SF7 = pd.read_csv("Dataset_final/open_sf7_250m.csv")
open_500_SF7 = pd.read_csv("Dataset_final/open_sf7_500m.csv")
open_1000_SF7 = pd.read_csv("Dataset_final/open_sf7_1000m.csv")

# Convert timestamp strings to datetime objects
for df in [open_250_SF7, open_500_SF7, open_1000_SF7]:
    df['ts'] = pd.to_datetime(df['ts'])


# Find the maximum duration among all samples
end_time_open_SF7 = max(df['ts'].max() - df['ts'].min()
                        for df in [open_250_SF7, open_500_SF7, open_1000_SF7])

# Plotting the data
for df, label in [(open_250_SF7, "250m"), (open_500_SF7, "500m"), (open_1000_SF7, "1000m")]:
    df['ts'] = (df['ts'] - df['ts'].min()).dt.total_seconds()
    plt.plot(df['ts'], df['rssi'], marker='.', label=label)

# Set x-axis ticks and limit
# Adjust interval as needed
plt.xticks(range(0, int(end_time_open_SF7.total_seconds()) + 1, 15))
plt.xlim(-5, end_time_open_SF7.total_seconds()+5)

plt.rcParams.update({'font.size': 14})

# plt.title("Open Land SF7", fontsize=22)
plt.legend(loc='upper right', fontsize=16)
plt.xlabel("time [s]", fontsize=18)
plt.ylabel("RSSI [dBm]", fontsize=18)

plt.xticks(fontsize=16)
plt.yticks(fontsize=18)

plt.show()
