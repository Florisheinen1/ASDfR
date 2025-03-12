import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd

# Function to read data from a text file
def read_data(file_path):
    with open(file_path, 'r') as file:
        return [int(line.strip().strip(',')) for line in file.readlines()]

# Read data from two files
posix_data = read_data('posix_results.csv')
evl_data = read_data('evl_results.csv')

# Create a DataFrame for visualization
df = pd.DataFrame({
    'Time': range(1, len(posix_data) + 1),
    'posix': posix_data,
    'evl': evl_data
})

# Melt data for seaborn compatibility
melted_df = df.melt(id_vars=['Time'], var_name='Type', value_name='Delay (microseconds)')

# Create subplots
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Boxplot
sns.boxplot(x='Type', y='Delay (microseconds)', data=melted_df, ax=axes[0])
axes[0].set_title("Boxplot Comparison")

# Line plot
sns.lineplot(x='Time', y='Delay (microseconds)', hue='Type', data=melted_df, ax=axes[1], marker='')
axes[1].set_title("Delay over time")
axes[1].set_xticks([])

# Show plots
plt.tight_layout()
plt.show()