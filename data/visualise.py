import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# cd C:\Users\Will\Engineering\_Ongoing\CarND\Term_2\VS_Project_2.1\VS_Project_2.1\data

filePath = 'output.txt'

df = pd.read_csv(filePath, delimiter='\t')

#print(df)

min_time = df['time_stamp'].min()

df['time_stamp'] -= min_time
df['time_stamp'] /= 1000000.0

time_ = df['time_stamp']
nis_ = df['NIS']

print(nis_)

#toPlot = pd.Series(nis_, index=time_)

#print(toPlot)
#toPlot.plot()

#plt.show()

#print(df.loc[:, 'time_stamp'].min())
