If you want the plugin to move roombas to work you must first run in the terminal window:

cd build

cmake ..

make -j3

cd ..

export GAZEBO_PLUGIN_PATH=$HOME/Desktop/IARCWorld/build:$GAZEBO_PLUGIN_PATH

where $HOME/Desktop/IARCWorld/build is the path to the *build* folder in the *IARCWorld* folder on your machine.
This command will only affect the terminal window your are running in.  To run this command for every terminal window, put it in your .bashrc file.

To open the model in gazebo run:
$ gazebo -u IARCWorld.world
The -u flag opens the gazebo world paused.
