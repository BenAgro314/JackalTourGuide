import matplotlib
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
    filename = input("Input file name (" + str(i+1) + "/" + str(num) + "). Note default = path_data.csv\n")
    if (filename == ''):
        filename = "path_data.csv"
    files.append("/home/"+username+"/Myhal_Simulation/simulated_runs/" + path + "/logs-" + path + "/"+ filename)

x_labels = []
y_data = []
success_labels = []
labels = []

for filename in files:
    x = []
    y = []
    suc = []
    series = ""

    
    with open(filename,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        count = 0

        last_success = True
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
                if (last_success):
                    suc.append("")
                    actual_length = float(row[3])
                else:
                    suc.append("Failure")
                    actual_length = optimal_length*2

                
                last_success = True
            else:
                suc.append("Failure")
                actual_length = optimal_length*2
                last_success = False

            percent_diff = ((actual_length - optimal_length)/optimal_length)*100
            y.append(percent_diff)
            count+=1
    
    x_labels = x
    y_data.append(y)
    labels.append(series)
    success_labels.append(suc)

x = np.arange(len(x_labels))
width = 0.8


fig,ax=plt.subplots()
n = len(y_data)

rects_list = []

for i in range(len(y_data)):
    c = ["g"]*len(x_labels)
    if (labels[i] == "Ground Truth Demon"):
        c = ["r"]*len(x_labels)
        j =0
        for l in success_labels[i]:
            if (success_labels[i][j] != ""):
                c[j] = "rosybrown"
            j+=1
    elif (labels[i] == "No Demon"):
        c = ["b"]*len(x_labels)
        j =0
        for l in success_labels[i]:
            if (success_labels[i][j] != ""):
                c[j] = "lightsteelblue"
            j+=1

    
    rects_list.append(ax.bar(x- width/2. + i/float(n)*width, y_data[i], width/float(n), align="edge",  label = labels[i], color = c))

ax.set_ylabel('Percent Deviation From Optimal Path (%)')   
ax.set_xlabel('Target Number')  
ax.set_title('Length Difference Between Actual Path and Optimal Path')
ax.set_xticks(x)
ax.set_xticklabels(x_labels)
ax.legend()

def autolabel(rects, labels):
    """Attach a text label above each bar in *rects*, displaying its height."""
    i =0
    for rect in rects:
        height = rect.get_height()
       
        ax.annotate(labels[i],xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')
        i+=1


i =0
for rects in rects_list:
    autolabel(rects, success_labels[i])
    i+=1


fig.tight_layout()
plt.show()


