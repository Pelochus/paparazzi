import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('./var/logs/VideoUltimo30Abril.csv', delimiter='\t')

pd.set_option("display.max.columns", None)

# Display the first few rows of the DataFrame
print(df.head())

# Check columns names
print(df.columns.tolist()) 

# Plot an XY chart
df.plot(x="Time", y=["SONAR:sonar_distance", "INS:ins_z", "ENERGY:throttle"])

# Show the plot
plt.show()