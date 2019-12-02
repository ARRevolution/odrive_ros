Starting point for this was Neomanic's odrive node with some parts added from https://github.com/williammlu/odrive_ros
Thank you both!!

# odrive_ros
ROS driver for the [ODrive motor driver](https://odriverobotics.com/)

This is a basic first pass at a ROS driver. It's Python-based, so not super-fast, but it'll get you started. Maybe in time this will have an optimised C++ version, but I woudn't count on it anytime soon. ;)

Future plans do include: 

- pull encoder parameter back from ODrive rather than specifying it manually
- publishing odometry back to ROS

Feedback, issues and pull requests welcome.

## Usage
There is a sample launch file showing the use of the wheelbase, tyre circumference, and encoder count parameters too, try `roslaunch odrive_ros odrive.launch` or copy it into your own package and edit the values to correspond to your particular setup.

If you want to test out the driver separate from ROS, you can also install as a regular Python package.

```sh
roscd odrive_ros
sudo pip install -e .
```

Fire up a Python shell. See below for the import which exposes the ODrive with setup and drive functions. You can use either ODriveInterfaceAPI (uses USB) or ODriveInterfaceSerial (requires the /dev/ttyUSBx corresponding to the ODrive as parameter, but not tested recently).

```python
from odrive_ros import odrive_interface
od = odrive_interface.ODriveInterfaceAPI()
od.connect()
od.setup()          # does a calibration
od.engage()         # get ready to drive
od.drive(1000,1000) # drive axis0 and axis1
od.release()        # done
```

## Acknowledgements

Thanks to the ODrive team for a great little bit of hardware and the active community support. Hope this helps!

- [ODrive homepage](https://odriverobotics.com)
- [ODrive getting started](https://docs.odriverobotics.com)
- [ODrive main repo](https://github.com/madcowswe/ODrive)

Thanks to Simon Birrell for his tutorial on [how to package a Python-based ROS package](http://www.artificialhumancompanions.com/structure-python-based-ros-package/).

