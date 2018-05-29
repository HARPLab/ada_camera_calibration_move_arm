import numpy as np
#from scipy.optimize import lsq_linear
import transforms3d as t3d

# given two 3d points curLoc and endLoc
# return the transformation associated with the frame
# with origin at curLoc and with the 
# z-axis pointing toward endLoc
# the other two axes are arbitrarily chosen
def get_motion_frame_matrix(curTransform, endLoc):
  curLoc = curTransform[0:3,3]
  curRot = curTransform[0:3,:][:,0:3]
  zDir = endLoc - curLoc
  xDir = perpendicular_vector(zDir)
  yDir = np.cross(zDir, xDir)
  zNorm = normalize(zDir) 
  xNorm = normalize(xDir)
  yNorm = normalize(yDir)
  rot = curRot.dot(np.concatenate(([xNorm], [yNorm], [zNorm]), axis=0).transpose())
  rot = np.concatenate(([xNorm], [yNorm], [zNorm]), axis=0).transpose()
  T = np.concatenate((np.concatenate((rot, np.zeros((3,1))), axis=1),[[0,0,0,1]]), axis=0)
  return(T)

# given a numpy vector
# return the numpy vector in the same direction of length 1
def normalize(v):
    norm = distance(v,np.zeros(v.shape))
    if norm == 0:
       raise ValueError('zero vector')
    return v / norm

def iszero(x):
  eps = 1e-15
  return np.absolute(x) < eps

# https://codereview.stackexchange.com/questions/43928/algorithm-to-get-an-arbitrary-perpendicular-vector
def perpendicular_vector(v):
  if iszero(v[0])  and iszero(v[1]):
      if iszero(v[2]):
          # v is Vector(0, 0, 0)
          raise ValueError('zero vector')
      # v is Vector(0, 0, v.z)
      return np.array([0, 1, 0])
  return np.array([-v[1], v[0], 0])

def distance(curLoc, endLoc):
  diff = endLoc - curLoc
  dist = np.sqrt(sum(diff**2))
  return dist
  
if __name__=="__main__":
  import unittest

  def tuple_are_equal(tup1, tup2):
    allTrue = True
    for i,_ in enumerate(tup1):
      allTrue &= numpy_are_equal(tup1[i], tup2[i])
    return allTrue

  def numpy_are_equal(mat1, mat2):
    epsilon = 0.00000001
    if np.all(np.abs(mat1 - mat2) < epsilon):
      return True
    else:
      print("The following matrices were not equal: %s \n %s"% (mat1, mat2))
      return False
  
  class TestMethods(unittest.TestCase):
    def test_get_motion_frame_matrix(self):
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),np.array([1,0,0])), 
                       np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])))
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[1,0,0,0],[0,1,0,1],[0,0,1,1],[0,0,0,1]]),np.array([1,1,1])), 
                       np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])))
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[1,0,0,1],[0,1,0,1],[0,0,1,0],[0,0,0,1]]),np.array([1,1,1])), 
                       np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])))
      self.assertTrue(numpy_are_equal(get_motion_frame_matrix(np.array([[0,1,0,0],[-1,0,0,0],[0,0,1,0],[0,0,0,1]]),np.array([1,0,0])), 
                       np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])))

    def test_distance(self):
      self.assertAlmostEqual(distance(np.array([0,0,1]),np.array([0,0,0])),1)
      self.assertAlmostEqual(distance(np.array([1,2,3]),np.array([2,3,4])),np.sqrt(3))

  unittest.main()

