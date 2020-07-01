import matplotlib.pyplot as plt
import csv
import os

username = os.environ['USER']
num = int(input("How many files would you like to plot?\n"))
inp = input("Input 1 for translation plot and 2 for rotation plot\n")

col = 1

if (inp == "2"):
    col = 2
    plt.title('Rotation Error vs. Distance Travelled')
    plt.ylabel("Rotation Error (rad)")
else:
    plt.title('Translation Error vs. Distance Travelled')
    plt.ylabel("Translation Error (m)")


files = []

for i in range(num):
	path = input("Input folder name?\n")
	filename = input("Input file name (" + str(i+1) + "/" + str(num) + "). Note default = localization_error.csv\n")
	if (filename == ""):
		filename = "localization_error.csv"
	files.append("/home/"+username+"/Myhal_Simulation/simulated_runs/" + path + "/logs-" + path + "/"+filename)



f_label = False
nf_label = False

for filename in files:

	x = []
	y = []

	series = ""

	with open(filename,'r') as csvfile:
		plots = csv.reader(csvfile, delimiter=',')
		count = 0
		for row in plots:
			if (count > 1):
				x.append(float(row[0]))
				y.append(float(row[col]))
			elif (count == 0):
				series = row[0]
				
			count+=1

	print(filename, ":", series)
	if (series == "Ground Truth Demon"):
		plt.plot(x,y, 'r',label=series)
	elif (series == "No Demon"):
		plt.plot(x,y, 'b--',label=series)
	elif ((series != "Ground Truth Demon") and (series != "No Demon")):
		plt.plot(x,y, label=series)
	else:
		if (series == "Filtering: true"):
			plt.plot(x,y, 'r')
		else:
			plt.plot(x,y, 'b--')

	

plt.xlabel("Distance Travelled (m)")


plt.legend()
plt.show()