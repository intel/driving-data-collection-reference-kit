Extrinsic Calibration Tools for non overlapping cameras:
-------------------------------------------------------
Unified code to perform extrinsic calibration for non overlapping cameras[usb and ptgrey].


Code Functionality:
------------------
apriltags: The calibration code uses apriltag library which helps to detect the tag locations in the image.

sheet_detail_creator.cpp  - code asks user input to rpovide sheet details[tag size and space], then determine world coordinates and stores the tag details in yaml file at config_path.
save_tag_details.cpp      - takes config path and tag details as input and saves images of tags taken from different angles, and their extrinsic params at each angle, using single camera. The saved details helps to find the tag relation using find_tag_rel.cpp

find_tag_rel.cpp          - takes the tag details and the extrinsics saved using save_tag_details.cpp, then finds the tag relation in world by optimization.
save_cam_details.cpp      - takes config path and tag details as input and saves images of tags taken from different angles, and their extrinsic params at each angle using cam pair.
find_cam_rel.cpp          - takes the tag details and the extrinsics saved using save_cam_details.cpp, then finds the relation between cameras by optimization.


Usage:
------
1. ./sheet_detail_creator: 
	provide tag size, space, number of tags, tagfamily and config path as command line arguments. user can choose the type of camera to capture the images.

2. ./save_tag_details: 
	provide configpath, first tag yaml path, second tag yaml path,and path to save images as command line arguments. user can choose the type of camera to capture the images.

3. ./save_cam_details:
	provide configpath, first tag yaml path, second tag yaml path,and path to save images as command line arguments. user can choose the type of camera to capture the images.
4. ./find_tag_rel:
	reads details saved using "save_tag_details.cpp" and perform optimization to find the relation between tags.

5. ./find_cam_rel:
	reads details saved using "save_cam_details.cpp" and  "find_tag_rel.cpp" , then perform optimization to find the relation between cameras.
