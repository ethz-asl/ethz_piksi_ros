rqt_gps_rtk_plugin
======

This package provides an rqt interface with status information about GPS RTK.

Dependencies
------
[piksi_rtk_msgs](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_msgs)
[Qt](https://wiki.qt.io/Install_Qt_5_on_Ubuntu)
[any_node](https://bitbucket.org/leggedrobotics/any_common/src/master/any_node/)

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

- `Last message (UTC)`: The (UTC) time stamp of the last received piksi message.

- `Fix type`: [Fix] or [Float]: Fix, if an rtk fix exists and float if single point position information is received (i.e., no rtk fix).

- `Num satellites`: The number of satellites visible to the GPS antenna attached to the piksi module.

- `Num satellites (RTK)`: The number of satellites used for the RTK fix. An additional graphical evaluation about the number of used rtk satellites is given: 'bad' if 4 or less satellites are used, 'ok' for 5-6 satellites and 'good' if 7 or more satellites are used for rtk.

- `NED baseline [m]`: The N(orth)E(ast)D(own) distance values in [m] from the (stationary) base station to the (mobile) piksi module.

- `Navsatfix altitude (avg)[m]`: The averaged altitude in [m] of the GPS RTK fix.

- `Num Wifi corrections`: The number of corrections received by the (mobile) piksi module via wifi.

- `Wifi correction rate [Hz]`: The rate in [Hz] with which wifi corrections are received.

- `Base Station ping [ms]`: The time in [ms] needed for the (mobile) robot to ping the base station, which is sending rtk correction data.

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
