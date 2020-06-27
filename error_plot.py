import matplotlib.pyplot as plt
import csv
import os

username = os.environ['USER']
num = int(input("How many files would you like to plot?\n"))
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
				y.append(float(row[1]))
			elif (count == 0):
				series = row[0]
				
			count+=1

	if ((not f_label) and series == "Ground Truth Demon"):
		plt.plot(x,y, 'r',label=series)
		f_label = True
	elif ((not nf_label) and series == "No Demon"):
		plt.plot(x,y, 'b--',label=series)
		nf_label = True
	elif ((series != "Ground Truth Demon") and (series != "No Demon")):
		plt.plot(x,y, 'g-.',label=series)
	else:
		if (series == "Filtering: true"):
			plt.plot(x,y, 'r')
		else:
			plt.plot(x,y, 'b--')

	

plt.xlabel("Distance Travelled (m)")
plt.ylabel("Localization Error (m)")
plt.title('Localization Error vs. Distance Travelled')
plt.legend()
plt.show()