# mavlink_udp_windows_vicon

How to use the software:

Make sure your cameras are calibrated, and that the objects you want to track are selected in the Vicon software.
Refer to the user manual for the Vicon software you're using for more detailed instructions.

Objects must be in the format "name@IPaddress" for this software to parse them properly. For example, "mydrone@192.168.5.5".
Ping your drone's onboard computer to find the static IP address.

Unzip the folder, make sure that all .dll files and the .exe are in the same directory. Once all the setup is done, simply run the .exe and witness the data. Currently, it only prints to the screen one object at a time, but rest assured that multiple objects are being tracked and each one is only receiving the relevant data. 


This is a work in progress program that packages data from a Vicon mocap system and sends UDP packets using mavlink.
Requires Vicon Nexus 1.4+, Vicon Blade 1.6+, or Tracker 1.0+. Tested on Windows 10 running Vicon Tracker 1.3.1.
Developed using Visual Studio 17.
This is a fork of the mavlink_udp repo from James Strawson: https://github.com/StrawsonDesign/mavlink_udp

