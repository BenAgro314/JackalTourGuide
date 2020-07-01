import matplotlib.pyplot as plt
import csv
import os

username = os.environ['USER']
path = input("Input folder name?\n")

files = []

filename = "trajectories.csv"
files.append("/home/"+username+"/Myhal_Simulation/simulated_runs/" + path + "/logs-" + path + "/"+filename)

x_min = 1000.0
x_max = -1000.0
y_max = -1000.0
y_min = 1000.0

for filename in files:

	x = []
	y = []

	series = ""

	with open(filename,'r') as csvfile:
		plots = csv.reader(csvfile, delimiter=',')
		
		skip = False
		count = 0

		for row in plots:
			if (skip):
				skip = False
				count+=1
				continue

			if (len(row) == 1):
				
				if (count > 0):
					print("Plotted:", series)
					plt.plot(x,y,label =series)
					x.clear()
					y.clear()
					
				series = row[0]
				skip = True
				
				count+=1
				continue
			
			y_min = min(float(row[1]),y_min)
			y_max = max(float(row[1]),y_max)
			x_min = min(float(row[0]),x_min)
			x_max = max(float(row[0]),x_max)

			x.append(float(row[0]))
			y.append(float(row[1]))

			count+=1
			
	print("Plotted:", series)
	plt.plot(x,y,label =series)


plt.title("Path Estimation vs Ground Truth")
plt.xlabel("x position (m)")
plt.ylabel("y position (m)")
plt.xlim((x_min-0.5,x_max+0.5))
plt.ylim((y_min-0.5,y_max+0.5))
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.show()