# visy_neopixel_pkg

ROS package for vision system (visy) to control neopixels

![CI](https://github.com/deltarobotone/visy_neopixel_pkg/workflows/CI/badge.svg?branch=master) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/abd24320e658402a8bc27d9f3d8c80c6)](https://app.codacy.com/gh/deltarobotone/visy_neopixel_pkg?utm_source=github.com&utm_medium=referral&utm_content=deltarobotone/visy_neopixel_pkg&utm_campaign=Badge_Grade_Dashboard)

This package is part of the visy ROS workspace:

[<img src="https://raw.githubusercontent.com/deltarobotone/image_database/master/visy_doc/visy_doc%20(1).PNG" width="1000">](https://raw.githubusercontent.com/deltarobotone/image_database/master/visy_doc/visy_doc%20(1).PNG)

- [visy_sorting_app_pkg](https://github.com/deltarobotone/visy_sorting_app_pkg)
- [visy_detector_pkg](https://github.com/deltarobotone/visy_detector_pkg)
- [visy_user_interface_pkg](https://github.com/deltarobotone/visy_user_interface_pkg)
- [one_easy_protocol_pkg](https://github.com/deltarobotone/one_easy_protocol_pkg)
- [visy_neopixel_pkg](https://github.com/deltarobotone/visy_neopixel_pkg)
- [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node)

## Nodes

### neo_pixel_node.py

ROS node connects neopixel devices like neopixel ring to ROS. Node provides subscriber and message to control neopixel hardware. If multiple neopixel devices are connected in row count all pixels and use param to define it. Use only one node to connect all pixels to ROS. Setup light control nodes for every device to control the pixels. Example: neopixel.launch at this package.

#### Parameters

##### ~number_of_pixels (Required)

Full number of connected pixels. If multiple neopixel devices connected in row count all pixels.

#### Subscribed topics

##### neo_pixels (visy_neopixel_pkg/Neopixels)

Get neopixel control states from light control nodes.

### light_ctrl_node.py

ROS node to control neopixel devices via predefined light functions like spin, fade, blink, etc. If multiple neopixel devices are connected in row start one of this nodes for every neopixel device and define first and last pixel in row. Count all pixels and set as param at neopixel node. Example: neopixel.launch at this package.

#### Advertised Services

##### light_ctrl (visy_neopixel_pkg/LightCtrl)

Select predefined light control functions for neopixel. Provided light functions are FULL, SPIN_SINGLE_CW, FADE_FULL, BLINK_FULL etc.

##### pixel_ctrl (visy_neopixel_pkg/PixelCtrl)

Service to control single pixel of a neopixel device. Control one pixel and let the other pixels in state (cleanup=false).

#### Parameters

##### ~first_pixel (Required)

Number of first pixel in row. If multiple neopixel devices are connected in row this param is for seperating the devices.

##### ~last_pixel (Required)

Number of last pixel in row. If multiple neopixel devices are connected in row this param is for seperating the devices.

#### Published topics

##### ~/neo_pixels (visy_neopixel_pkg/Neopixels)

Publishes control states to neopixel node which connects the state to hardware.

