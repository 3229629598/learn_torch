# learn_torch

You could run the following instruction to calibrate the camera:  
```
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.015 left:=/left_cam/image_raw left_camera:=/left_cam right:=/right_cam/image_raw right_camera:=/right_cam --no-service-check
```