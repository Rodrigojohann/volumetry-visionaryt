from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from plyfile import PlyData, PlyElement
import scipy
from scipy import spatial
infile = '/home/rodrigo/Volumetry/volumetry-visionaryt/testcloud.ply'
plydata = PlyData.read(infile)
elements = plydata.elements[0]
datalist = list()



for i in range(len(elements.data)):
    item = elements.data[i]
    if item[0] < 0:
        datalist.append(np.array([item[0],item[1],item[2]]))

dataarray = np.array(datalist)

convexhull = spatial.ConvexHull(dataarray)



x,y,z = dataarray[:,0], dataarray[:,1], dataarray[:,2] #returns X,Y,Z points skipping the first 12 lines
# 	
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#for i in convexhull.simplices:
#    plt.plot(dataarray[i,0], dataarray[i,1], dataarray[i,2], 'r-')

ax.scatter(x, y, z, c='r', marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()	
	