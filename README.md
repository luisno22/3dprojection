# 3dprojection

The purpose of the project described in this document is the implementation of a simple Augmented Reality (AR) application, using either a pre-recorded or a real-time video stream. Afterwards, a virtual 2D figure is projected simulating a 3D real visualitzation. In robotics, knowing the location of a point in the real world based on a captured image is something really useful and necessary in some cases, allowing, for instance, the control of a manipulator arm   elative positon in relation to a work table using cameras, giving the robot consciousness about its surrounding environment and the ways it can be performed on. To accomplish this, it is necessary to calibrate the camera device used in the application. In this project, the calibration process is made through a chessboard pattern, since the chessboard pattern is commonly used for its simplicity and easy-to-find documentation, and a set of images where the pattern is presented in different positions and perspectives. Camera calibration is the process of estimating the intrinsic and/or extrinsic values that allow 2D projections of 3D real world points. This values are formally represented within a camera matrix K:
K = [fx, s, Rx; 
      0, fy, Ry; 
      0,  0, 1]

Henceforth, the calibration parameters will allow future transformations, and extrinsic parameters calculations, of captured frames regarding the calibration pattern spot in the shot, in order to obtain a 2D projection of a desired 3D figure.
