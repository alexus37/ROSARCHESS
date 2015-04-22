import build/libGetGlCamera as cam 
import numpy as np 

if __name__ == '__main__': 
	fx = 1.; 
	fy = 1.; 
	cx = 0.; 
	cy = 0.; 
	imX = 100; 
	imY = 100; 
	glX = 100; 
	glY = 100; 

	glP = cam.getGlCamera

	#P = np.array([0.0, 1.1, 2.2])
	#P = cam.stdVecToNumpyArray() 

