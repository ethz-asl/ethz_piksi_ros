piksi_rtk_kml
======
ROS node to write kml file from Piksi messages.

Keyhole Markup Language ([KML](https://developers.google.com/kml/)) is an XML notation for expressing geographic annotation and visualization within Internet-based, two-dimensional maps and three-dimensional Earth browsers. KML was developed for use with Google Earth.

## Usage
You can either generate onlie a KML file or post-process RTK data that you recorded in a [rosbag](http://wiki.ros.org/rosbag) to generate a KML file, that can be later opened in Google Earth. 

The following launch file can be used to generate offline a KML file using a rosbag containing `/piksi/navsatfix_rtk_fix` topic. Settings of this file has been tuned for a ground vehicle, i.e. the path has been forced to stick to the ground. Once the rosbag is finished, just kill the launched file. A KML file will be generated in the folder [kml](./kml) and that can be imported in Google Earth.
```
roslaunch piksi_rtk_kml offline_ground_vehicle.launch
```

## Example of output
The following image has been taken from Google Earth, where a KML file has been generated using the above launch file.
![example_kml_image_output](https://user-images.githubusercontent.com/5328930/41427607-c496cc26-7007-11e8-9ccb-2f6ec6b25ba6.png)
