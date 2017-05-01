from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import csv


x = np.linspace(-2,10,200)
y = np.linspace(-2,10,200)

X, Y = np.meshgrid(x,y)

Z=-( 4*np.exp(-(X-4)**2 - (Y-4)**2)+2*np.exp(-(X-2)**2 - (Y-2)**2) )

#Make a 3D plot
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, Z,linewidth=0,cmap='coolwarm')

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')




#adding path to the max
x_path_to_max=[]
y_path_to_max=[]
z_path_to_max=[]
csv_trajectory="/home/behnam/optimisation_based_path_planner_ws/devel/lib/lmcma_path_planner/path_to_min.csv"


with open(csv_trajectory,'r') as csv_file:
    reader = csv.DictReader(csv_file)
    for row in reader:
        x_path_to_max.append(float(row['x']) )
        y_path_to_max.append(float(row['y']) )
        z_path_to_max.append(float(row['z']) )

line1=plt.plot(x_path_to_max,y_path_to_max,z_path_to_max)
plt.setp(line1,color='g',linewidth=0.5)


plt.show()
