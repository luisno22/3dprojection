# 3dprojection

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
