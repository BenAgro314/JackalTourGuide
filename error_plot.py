import matplotlib.pyplot as plt
import csv
import os

username = os.environ['USER']
num = int(input("How many files would you like to plot?\n"))
inp = int(input("Input 1 for translation plot and 2 for rotation plot\n"))

col = 1

title = ""
y_label = ""

if (inp == 2):
	col = 2
	title = 'Rotation Error vs. Distance Travelled'
	y_label = "Rotation Error (rad)"
else:
	title = 'Translation Error vs. Distance Travelled'
	y_label = "Translation Error (m)"


files = []

for i in range(num):
	path = input("Input folder name?\n")
	filename = input("Input file name (" + str(i+1) + "/" + str(num) + "). Note default = localization_error.csv,\n (1) = gt_filter_localization_error_gmapping,\n (2) = no_filter_localization_error_gmapping,\n(3) = gt_filter_localization_error_amcl,\n(4) = no_filter_localization_error_amcl\n")
	if (filename == ""):
		filename = "localization_error.csv"
	elif (filename == "1"):
		filename = "gt_filter_localization_error_gmapping.csv"
	elif (filename == "2"):
		filename = "no_filter_localization_error_gmapping.csv"
	elif (filename == "3"):
		filename = "gt_filter_localization_error_amcl.csv"
	elif (filename == "4"):
		filename = "no_filter_localization_error_amcl.csv"
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
plt.ylabel(y_label)
plt.title(title)
plt.legend()
plt.show()
