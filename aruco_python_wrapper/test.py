import libGetGlCamera as cam 
import numpy as np 

if __name__ == '__main__': 
	#P = np.array([0.0, 1.1, 2.2])
	#cam.stdVecToNumpyArray() 
	cam.init()
	
	fx = 1.; 
	fy = 1.; 
	cx = 0.; 
	cy = 0.; 
	imX = 100; 
	imY = 100; 
	glX = 100; 
	glY = 100; 

	glP = cam.getGlCamera(fx,cx,fy,cy,imX,imY,glX,glY)

	print glP

	#P = cam.mywrapper() 
	#cam.mywrapper2(P)
	#print P

