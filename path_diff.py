import matplotlib.pyplot as plt
import numpy as np
import csv
import os

def subcategorybar(X, vals, labels, width=0.8):
    n = len(vals)
    _X = np.arange(len(X))
    for i in range(n):
        plt.bar(_X - width/2. + i/float(n)*width, vals[i], 
                width=width/float(n), align="edge", label = labels[i])   
    plt.xticks(_X, X)

username = os.environ['USER']
num = int(input("How many files would you like to plot?\n"))
files = []

for i in range(num):
	path = input("Input folder name?\n")
	files.append("/home/"+username+"/Myhal_Simulation/simulated_runs/" + path + "/logs-" + path + "/"+input("Input file name (" + str(i+1) + "/" + str(num) + ")\n"))

x_labels = []
y_data = []
labels = []

for filename in files:
    x = []
    y = []
    series = ""

    with open(filename,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        count = 0
        for row in plots:
            if (count < 2):
                if count == 0:
                    series = row[0]
                count+=1
                continue
            x.append(row[0])
            optimal_length = float(row[1])
            success = int(row[2])
            actual_length = 0
            if (success):
                actual_length = float(row[3])


            percent_diff = ((actual_length - optimal_length)/optimal_length)*100
            y.append(percent_diff)
            count+=1
    
    x_labels = x
    y_data.append(y)
    labels.append(series)
    #plt.bar(x,y, label= series)


subcategorybar(x_labels , y_data, labels)

plt.xlabel('Target Number')
plt.ylabel('Difference From Optimal Path (%)')
plt.title('Length Difference Between Actual Path and Optimal Path')
plt.legend()

plt.show()