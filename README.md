# mavlink_udp_windows_vicon
Todo:
Update program so that if Vicon loses track of an object, the program reports the last known position instead of all 0s

Work in progress program that packages data from a Vicon mocap system and sends UDP packets using mavlink.
Requires Vicon Nexus 1.4+, Vicon Blade 1.6+, or Tracker 1.0+. Tested on Windows 10 running Vicon Tracker 1.3.1.
Developed using Visual Studio 17.
This is a fork of the mavlink_udp repo from James Strawson: https://github.com/StrawsonDesign/mavlink_udp

