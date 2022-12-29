# 3D Projection on a 2D Image and camera calibration

The purpose of the project described in this document is the implementation of a simple Augmented Reality (AR) application, using either a pre-recorded or a real-time video stream. Afterwards, a virtual 2D figure is projected simulating a 3D real visualitzation. In robotics, knowing the location of a point in the real world based on a captured image is something really useful and necessary in some cases, allowing, for instance, the control of a manipulator arm   elative positon in relation to a work table using cameras, giving the robot consciousness about its surrounding environment and the ways it can be performed on. To accomplish this, it is necessary to calibrate the camera device used in the application. In this project, the calibration process is made through a chessboard pattern, since the chessboard pattern is commonly used for its simplicity and easy-to-find documentation, and a set of images where the pattern is presented in different positions and perspectives. Camera calibration is the process of estimating the intrinsic and/or extrinsic values that allow 2D projections of 3D real world points. This values are formally represented within a camera matrix K:
K = [fx, s, Rx; 
      0, fy, Ry; 
      0,  0, 1]

Henceforth, the calibration parameters will allow future transformations, and extrinsic parameters calculations, of captured frames regarding the calibration pattern spot in the shot, in order to obtain a 2D projection of a desired 3D figure.

# Code Usage

-c With this argument, the code will only execute the camera calibration
mode, saving the outputs into a XML file named "parameters.xml". The
calibration process will run through all the jpeg images in the directory
/images previously created in the workspace. The image names have
to be renamed to ascendant integers in order to be collected as inputs
by the script.


-r This argument sets the script mode to reconstruct a 3D object and
project it into a 2D image, in this mode, the code will use as input
for the camera parameters a pre-existing file named "parameters.xml"
to work with. Furthermore, if a video file is specified behind this ar-
gument, the code will run the reconstruction over the video, otherwise
the reconstruction will take the default camera input of the computer.


-s This argument will make the code show the calibration results for each
image, drawing the found corners, if any. It has to be used alongside
the [-c] argument.


-h With this argument, any arguments used coupled with this will be
ignored, and only a help menu will be shown.


Except the [-h] argument, all arguments can be mixed, i.e. Mixing -c -r
arguments will cause a calibration and reconstrucion execution of the code.
A result video with will be saved in the workspace as ’outputVideo.avi’.


# Results
Webcam results can be seen here: https://youtu.be/TutqxRgyedw

More Results: https://drive.google.com/file/d/1sGosJyvvoe0aRIkTDE42oxgL5FzLuDf_/view?usp=share_link

# Further information
In first instance, calibration is made. OpenCV implements a function called
calibrateCamera() that receives the following inputs as parameters:

• Vector of points that stores 3D points about the chessboard corners
using its own reference system, this vector is known since the sizes of
the board are known.

• Vector of 2D points storing the image points where board corners are
found. This corners are previously extracted using OpenCV’s algorithm
bool findChessboardCorners().

• Size of the image.

And returns the subsequent output parameters:
• Camera matrix K (intrinsic parameters).
• Distortion coefficients (intrinsic parameters).
• Rotation vectors (extrinsic parameters).
• Translation vectors (extrinsic parameters).

The more images used in the calibration process, the more precision the
application will have.

Once the camera is succesfully calibrated, the 2D projection of the 3D
object process can start. Every single frame of the input video stream is
analized in search of corners using findChessboardCorners(), and, if a
pattern is found, the function returns true and the process continues using
bool solvePnP(), receiving this parameters:

• Vector of 3D points storing the chessboard corners using its own ref-
erence system, this vector is known since the sizes of the board are
known.

• Vector of 2D points storing the image points where board corners are
found. This corners are previously extracted using OpenCV’s algorithm
bool findChessboardCorners().

• Camera matrix obtained previously.

• Distortion coefficients obtained previously.


And returning as output parameters:

• Rotation vectors, indicating the orientation of the camera with respect
to origin axis.

• Translation vectors, indicating the position of the camera with respect
to origin axis.


After calculating both rotation and translation vectors, the next step
is the actual 3D projection atop the 2D image. For accomplishing this
task, OpenCV provides a function named void projectPoints(), this func-
tion uses the earlier calculated rotation and translation vectors, the camera
matrix, the distortion coefficients and the points that are going to be pro-
jected, this 3D points are pre-defined according to the board reference system.
OpenCV’s function projectPoints() returns as output a 2D Points vector
containing the projected points. The last steps are drawing those points with
one of many OpenCV functions and showing the results.

