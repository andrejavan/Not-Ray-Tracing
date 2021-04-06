import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import ConvexHull
import numpy as np

def norm2(p):
  sum = 0
  for i in range(len(p)):
      sum+= p[i]**2
  return sum**.5

def getR(points, alpha, viewpoint):
  return alpha*max(list(map(lambda p:norm2(np.subtract(p,viewpoint)),points)))

def sphereflip(p,R):
  n = norm2(p)
  pflip = []
  try:
    pflip=list(map(lambda x:x+2*(R-n)*x/n,p))
  except:
    print("shit")
  return pflip

def plot3d(points):
  ax = plt.axes(projection='3d')
  #plot sphere
  theta = np.linspace(0, 2 * np.pi, 100)
  x=list(map(lambda x:np.cos(x),theta))
  y=list(map(lambda x:np.sin(x),theta))
  z=list(map(lambda x,y:(1-x**2-y**2)**.5,x,y))
  ax.plot_wireframe(x,y,z)
  plt.plot(list(map(lambda x:np.cos(x),theta)),list(map(lambda x:np.sin(x),theta)))
  x = list(map(lambda x:x[0],points))
  y = list(map(lambda x:x[1],points))
  z = list(map(lambda x:x[2],points))
  
  ax.scatter3D(x, y, z, c=z, cmap='ro');
  plt.show()  
  
def flip2d(points,alpha):
  R = getR(points, alpha, [0,0])
  #show origin
  plt.scatter([0],[0],marker = '*')
  plt.title('alpha = ' + str(alpha))
  #plot circle
  theta = np.linspace(0, 2 * np.pi, 100)
  plt.plot(list(map(lambda x:R*np.cos(x),theta)),list(map(lambda x:R*np.sin(x),theta)))
  #plot original points
  x = list(map(lambda x:x[0],points))
  y = list(map(lambda x:x[1],points))
  plt.plot(x,y)
  #plot new 
  points2 = list(map(lambda p:sphereflip(p,R),points))
  x = list(map(lambda x:x[0],points2))
  y = list(map(lambda x:x[1],points2))
  plt.scatter(x,y,marker='^')
  #show convexhull
  pts = np.asarray([[0,0]]+points2)
  hull = ConvexHull(pts)
  plt.plot(pts[hull.vertices,0], pts[hull.vertices,1], 'r--', lw=2)
  plt.plot(pts[hull.vertices,0], pts[hull.vertices,1], 'ro')
  #show original visible points
  vispts = np.asarray(list(map(lambda p:sphereflip(p,R),pts[hull.vertices])))
  plt.plot(vispts[:,0], vispts[:,1], 'bo')
  plt.show()

def flip3d(points,alpha):
  R = getR(points, alpha, [0,0,0])
  ax = plt.axes(projection='3d')
  #show origin
  ax.scatter3D([0],[0],[0],marker = '*')
  plt.title('alpha = ' + str(alpha))
  #plot sphere
  u = np.linspace(0, np.pi, 10)
  v = np.linspace(0, 2 * np.pi, 10)
  x = R*np.outer(np.sin(u), np.sin(v))
  y = R*np.outer(np.sin(u), np.cos(v))
  z = R*np.outer(np.cos(u), np.ones_like(v))
  ax.plot_surface(x,y,z, alpha=0.1)
  #plot original points
  ax.scatter3D(points[:,0],points[:,1],points[:,2])
  #plot new
  points2 = np.asarray(list(map(lambda p:sphereflip(p,R),points)))
  ax.scatter3D(points2[:,0],points2[:,1],points2[:,2])
  #show convexhull
  pts = np.append(points2,[[0,0,0]],axis=0)
  hull = ConvexHull(pts)
  ax.scatter3D(pts[hull.vertices,0],pts[hull.vertices,1],pts[hull.vertices,2])
  #show original visible points
  vispts = np.asarray(list(map(lambda p:sphereflip(p,R),pts[hull.vertices])))
  ax.scatter3D(vispts[:,0],vispts[:,1],vispts[:,2])
  plt.show()

##2DGRAPHS##
def theta(k):
  return np.linspace(0,2*np.pi,int(10**k))
def circle(theta):
  return list(map(lambda x:[4+np.cos(x),np.sin(x)],theta))
def spiral(theta):
  return list(map(lambda x:[6+x*np.cos(2*x),x*np.sin(2*x)],theta))
def cannabisCurve(t):
  r=4.2*(1+0.9*np.cos(8*t))*(1+0.1*np.cos(24*t))*(.9+.1*np.cos(200*t))*(1+np.sin(t))
  return [15+r*np.cos(t),r*np.sin(t)]
##3DGRAPHS##
def slinky(theta):
  return np.asarray(list(map(lambda x:[4+np.cos(2*x),np.sin(2*x),x],theta)))
def sphere(theta,R, shift):
  z=np.linspace(-R,R,len(theta))
  return np.asarray(list(map(lambda t,z: [shift+np.sqrt(R**2-z**2)*np.cos(100*t),np.sqrt(R**2-z**2)*np.sin(100*t),z],theta,z)))
#testing
#flip2d(spiral(theta(2)),10)
flip2d(list(map(cannabisCurve,theta(4))), 1100)
#flip3d(slinky(theta(2)),2)
#flip3d(sphere(theta(3.5),1,4),1.1)