# Toroboeye ROS changelog
This projects adheres to [Semantic Versioning](https://semver.org/)

# 0.5.3
- support for pytoroboeye 0.5.3
    - add depth_multiple_exposure_times in GetCaptureSettings.srv, SetCaptureSettings.srv
    - add depth_multiple_exposure_times in default parameters
    - add adequate_score in GetCaptureSettings.srv, SetCaptureSettings.srv
    - add adequate_score in default parameters
- change publish_rate from 1 hz to 5 hz in toroboeye_camera_publisher.py

# 0.4.0
- Initial release of ROS driver for Toroboeye cameras. For more information, see [README.md](./README.md)