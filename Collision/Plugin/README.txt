This is a rough plugin for the lidar sensor in the collision branch of 
the multi rotor robot design team at Missouri University of Science and
Technology. This plugin was built using a tutorial from gazebosim.org.

URL:
http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

Files in this folder and their purposes:

LIDAR_plugin.cc.txt:
This is NOT the plugin. This file is where changes are made to the plugin.
To make a change to the plugin write your changes into this file then rename
it to 'LIDAR_plugin.cc'. I highly suggest that before you rename the file
you make a copy of it in case something goes wrong. The only changes that 
have been made to this file from what was copied over in the turorial is 
changing the name "Velodyne" to "LIDAR".

LIDAR_plugin.cc:
This IS the plugin. This is the file that is run when the plugin is used.
To make changes to this file refer to LIDAR_plugin.cc.txt

CMakeLists.txt:
I think this file build the actual plugin. Please refer to the tutorial
referenced above for more information.