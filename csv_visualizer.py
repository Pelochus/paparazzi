import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('./var/logs/24_04_29-1.csv', delimiter='\t')

pd.set_option("display.max.columns", None)

# Display the first few rows of the DataFrame
print(df.head())

# Check columns names
print(df.columns.tolist()) 

# Plot an XY chart of the first 3 columns
df.plot(x="Time", y="SONAR:sonar_distance")
df.plot(x="Time", y="ENERGY:throttle")

# Show the plot
plt.show()