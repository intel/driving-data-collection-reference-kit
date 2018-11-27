Extrinsic Calibration Tools:
----------------------------
Unified code to perform extrinsic calibration of different cameras[usb and ptgrey].


Code Functionality:
------------------
apriltags: The calibration code uses apriltag library which helps to detect the tag locations in the image.

extrinsic_calib_multicam.cpp - Measures relationship between camera pair using single or multiple apriltags.
single_cam_to_ground.cpp     - Measure the relationship between camera's image plane to ground plane using   
                               single or multiple apriltags.
cameras/ptgrey.cpp           - Contains functions to retreive image in raw or rectified from point grey cameras.
cameras/ptgrey.h             - header file for ptgrey.cpp.
cameras/usb_cam.cpp          - Contains functions to retreive image in raw or rectified from usb cameras.
cameras/usb_cam.h            - header file for usb_cam.cpp.
cameras/param.h              - Contains the structure to hold the camera parameters.


Usage:
------
1. extrinsic_calib_singlecam: 
	Asks for selecting the reference camera, tag type,tag size and path to configuration file. once all parameters are given it looks for the apriltag and measure the extrinsics according to the tag position.

2. extrinsic_calib_multicam: 
	Asks to choose primary and secondary cameras,tag type, tag size and configuration path. performs extrinsic calibration for a camera pair and generates a matrix that relates secondary image to primary.

