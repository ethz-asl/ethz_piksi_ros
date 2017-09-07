rqt_gps_rtk_plugin
======

This package provides an rqt interface with status information about the GPS RTK status.

Dependencies
------
[piksi_rtk_msgs](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_msgs)

General Usage
------
To launch the gui 'standalone', run
```
#!bash

rosrun rqt_gps_rtk_plugin gui.launch
```

Note that this gui is compiled as an rqt plugin and may therefore be implemented into any existing rqt perspective.

GUI
------
Gui on startup             |  Gui with running piksi node
:-------------------------:|:-------------------------:
![Gui on startup](https://github.com/ethz-asl/mav_rtk_gps/blob/a5fbdcdbca7cdc35833e33e94b635006998afc06/rqt_gps_rtk_plugin/gps_gui_startup.png?raw=true)  |  ![Gui with running node](https://github.com/ethz-asl/mav_rtk_gps/blob/a5fbdcdbca7cdc35833e33e94b635006998afc06/rqt_gps_rtk_plugin/gps_gui_running.png?raw=true)

License
-------
The source code is released under a [BSD 3-Clause license](https://github.com/ethz-asl/mav_rtk_gps/blob/master/LICENSE).

Credits
-------
Marco Tranzatto, Kai Holtmann - ETHZ ASL & RSL - 07 September 2017

Contact
-------
Kai Holtmann kaiho(at)ethz.ch


Bugs & Feature Requests
-------
Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/mav_rtk_gps/issues).
