#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <fstream>
#include <sstream>
#include <boost/python.hpp>
#include <numpy/ndarrayobject.h>
#include <boost/numpy.hpp>
#include <vector>

using namespace std;
using namespace aruco;

namespace bp = boost::python;
namespace bn = boost::numpy;

void Init(){
Py_Initialize(); 
bn::initialize();
}


bn::ndarray GetGlCamera(double fx, double cx, double fy, double cy, 
  int imSizeX, int imSizeY, int glSizeX, int glSizeY){

  // openCV camera matrix
  cv::Mat cam = cv::Mat::zeros(3, 3, CV_32FC1); 
  cam.at<float>(0,0) = fx;
  cam.at<float>(1,1) = fy;
  cam.at<float>(0,2) = cx;
  cam.at<float>(1,2) = cy;

  cout << cam << endl;

  // dummy distortion coefficients - not needed
  cv::Mat dist = cv::Mat::zeros(1, 4,  CV_32FC1); 


  // camera image size
  cv::Size imSize(imSizeY, imSizeX);
  // gl image size
  cv::Size glSize(glSizeY, glSizeX);

  // initialize aruco camera
  CameraParameters camPar(cam,dist,imSize);
  double proj_matrix[16]; 
  double glNear = 0.05, glFar = 10.; 

  // get the cameras OpenGL matrix
  camPar.glGetProjectionMatrix(imSize,
    glSize,proj_matrix,glNear,glFar);

  cout << "glCam:" << endl;
  for(int i = 0; i < 16; i++) cout << proj_matrix[i] << endl;

  // numpy array init
  Py_intptr_t shape[2] = { 4, 4 };
  bn::ndarray result = bn::zeros(2, shape, 
      bn::dtype::get_builtin<double>());

  // copu proj_matrix into numpy array
  std::copy(proj_matrix, proj_matrix+16, 
      reinterpret_cast<double*>(result.get_data()));
    return result;
}

boost::python::object stdVecToNumpyArray()
{
	//import_array();	
	double data[5] = {0,1,2,3,4}; 
	npy_intp size = 3;
	/*const std::vector<double> vec{0.0,1.0,2.0};      
	npy_intp size = vec.size();*/

     /* const_cast is rather horrible but we need a writable pointer
        in C++11, vec.data() will do the trick
        but you will still need to const_cast
      */

      /*double * data = size ? const_cast<double *>(&vec[0]) 
        : static_cast<double *>(NULL); */

    // create a PyObject * from pointer and data 
	cout << "got here" << endl;
      PyObject * pyObj = PyArray_SimpleNewFromData( 1, &size, NPY_DOUBLE, data );
//cout << "got here" << endl;
      boost::python::handle<> handle( pyObj );
//cout << "got here" << endl;
      boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness 
      Secondly the lifetime of the data is managed by the C++ API and not the 
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */
	//cout << "got here" << endl;
       return arr.copy(); // copy the object. numpy owns the copy now.
  }


// example functions
bn::ndarray mywrapper() {
    
    cout << "hello " << endl;
    std::vector<double> v{0,1,2,3};
    Py_intptr_t shape[1] = { v.size() };
 cout << "hello " << endl;
    bn::ndarray result = bn::zeros(1, shape, bn::dtype::get_builtin<double>());
 cout << "hello " << endl;
    std::copy(v.begin(), v.end(), reinterpret_cast<double*>(result.get_data()));
    return result;
}

void mywrapper2(bn::ndarray& x) {
    
   x[0] += 0.5;
}




BOOST_PYTHON_MODULE(libGetGlCamera)
{
  using namespace boost::python;
  def("init", Init);
  def("getGlCamera", GetGlCamera);
  def("stdVecToNumpyArray", stdVecToNumpyArray);
  def("mywrapper", mywrapper);
  def("mywrapper2", mywrapper2);

}
