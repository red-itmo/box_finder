# box\_finder
box\_finder is a ROS package for students' project based on the ROS navigation stack. It gives a mobile robot the ability to 1) automatically create a map of unknown environment, 2) find a rectangular box with specified sizes within it using a created map, 3) come to found box.

The project has three "main" branches: _master_, _stage\_config_ and _robotino\_config_. They are designed for using package's software with the [Kuka Youbot](http://www.kuka-robotics.com/en/products/education/youbot/), the [Stage simulator](http://wiki.ros.org/stage_ros) and the [Robotino](http://www.festo-didactic.com/int-en/learning-systems/education-and-research-robots-robotino/the-complete-robotino-package.htm?fbid=aW50LmVuLjU1Ny4xNy4xOC44NTguNDc1Ng) respectively and so differ only in the content of configuration and launch files.

The package was designed and tested in the ROS Indigo. In other distros its operability wasn't verified.

For additional information about the package refer to its [wiki](https://github.com/red-itmo/box_finder/wiki).
