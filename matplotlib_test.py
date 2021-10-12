from matplotlib import pyplot as plt
import shapely
from shapely.geometry import LinearRing, LineString, Point
x = []
y = []

ray = LineString([(0,0), (1,1)])

x.append((list(ray.coords)[0][0]))
x.append((list(ray.coords)[1][1]))
y.append((list(ray.coords)[0][1]))
y.append((list(ray.coords)[1][1]))

plt.axis([-1, 1, -1, 1])

plt.plot(x, y)
plt.plot([0,1], [0,0])
   
plt.show()