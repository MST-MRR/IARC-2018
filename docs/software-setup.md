# Software Setup

#### Terms Overview

* [ArduPilot](https://github.com/ArduPilot/ardupilot)
  * Firmware used on a Flight Control Board \(PixHawk.\) This software running on the PixHawk comprises the low-level brain of the vehicle. It is responsible for translating high-level commands to the motors and interfacing with the IMU \(Inertial Measurement Unit\) to get gyroscopic angles, compass headings, or velocities and accelerations from the accelerometer. It also provides built-in modes and settings for changing flight characteristics of the vehicle.
* Ground Control Station \(GCS\)
  * Software like [QGroundControl](http://qgroundcontrol.com/), [APM Planner](http://ardupilot.org/planner2/), or[ Mission Planner](http://ardupilot.org/planner/docs/mission-planner-overview.html). This software allows us to send, receive, and make sense of data from the vehicle with a nice user interface. It supports sending takeoff and land \(plus a plethora of other\) flight commands, viewing altitude readings and flight logs, and even modifying settings specific to a vehicle in the firmware.
* [Gazebo](http://gazebosim.org/)
  * Simulator for robots. Gazebo connects to a simulated version of ArduPilot. From there, you can also connect to the simulated version of ArduPilot using a GCS to send commands the simulated vehicle. You can see the outcome of the commands without having to test on a real vehicle in the Gazebo environment.
* [MAVLink](http://mavlink.org/messages/common)
  * This is the language specification that a GCS uses to communicate with a vehicle. For instance, if a GCS requests battery information from the vehicle, the vehicle will send[ MAVLink message \#147](http://mavlink.org/messages/common#BATTERY_STATUS) to the GCS. It can also be used to communicate between sub-components of the vehicle. An off the shelf Optical Flow sensor is hard-coded to send a [specific MAVLink message](http://mavlink.org/messages/common#OPTICAL_FLOW) directly to the PixHawk over an I2C connection.

## Using a Virtual Machine

#### Requirements:

* Windows or macOS
* At least a dual-core processor
* 8GB's or more
* Good internet connection

Start by downloading the following applications for your machine:

* VirtualBox for Windows \(or whatever operating system you use\) [http://download.virtualbox.org/virtualbox/5.1.26/VirtualBox-5.1.26-117224-Win.exe](http://download.virtualbox.org/virtualbox/5.1.26/VirtualBox-5.1.26-117224-Win.exe)
* VirtualBox Extensions \(must be installed after VirtualBox\) [http://download.virtualbox.org/virtualbox/5.1.26/Oracle\_VM\_VirtualBox\_Extension\_Pack-5.1.26-117224.vbox-extpack](http://download.virtualbox.org/virtualbox/5.1.26/Oracle_VM_VirtualBox_Extension_Pack-5.1.26-117224.vbox-extpack)
* Vagrant for Windows  \(or whatever operating system you use\) [https://releases.hashicorp.com/vagrant/1.9.8/vagrant\_1.9.8\_x86\_64.msi](https://releases.hashicorp.com/vagrant/1.9.8/vagrant_1.9.8_x86_64.msi)

Next, open Terminal.app on macOS or PowerShell on Windows and create a folder on your desktop which will hold the virtual machine's files:

```
cd ~/Desktop
mkdir dronedev
cd dronedev
```

Then, initialize the vagrant box:

```
vagrant init kurron/xenial-x64-ubuntu-desktop
```

This command will take some time as it downloads an Ubuntu image to your machine. After it is finished, bring up the VM with:

```
vagrant up
```

This command will **fail** with Authentication failure. Just press CTRL-C, even on Mac.

![](/assets/auth_failure_vagrant.png)

After you return to a prompt, run the following command:

```
vagrant halt
```

Now, close your terminal or PowersShell window and open VirtualBox.

You should see a VM with the name of the folder it is in. In this case, it's **dronedev**. Start the VM.

![](/assets/import.png)

Once the VM comes up, you will be prompted to login as the user Vagrant. The password is also **vagrant**, but in all lowercase. After you are logged in, open terminal and run the following commands:

```
wget https://raw.githubusercontent.com/MST-MRR/IARC-2018/master/Control/Configuration/bootstrap.sh
chmod +x bootstrap.sh
./bootstrap.sh
```

This script will ask you to create a password. Enter any password you like and press enter to continue. It may also ask for odd things like a room number. Just leave those fields blank and press enter to continue.

After the script prints **Done, **log out by using the gear-like icon in the top right hand corner.

Once you reach the login screen, you will now see your user. Login with the password that you chose from the previous step.

Open a terminal once you are logged in and execute the following command:

```
./setup_vm.sh
```

This will take some time to run as it will download several gigabytes of packages. After the script prints **Done, **close the terminal window and re-open it.

Then type:

```
start_simulators
```

The first time you start the simulator will take some time because it will have to first compile the program. Subsequent starts should be much faster. Look** **for **Ready to Fly!** to appear in the terminal window.

You should also see a program called Gazebo launch with a drone in the viewport. Congratulations, you're all set up!

## Compiling from scratch on Ubuntu

#### Requirements:

* Ubuntu 16.04 LTS or greater
* At least a dual-core processor
* 8GB's or more
* Good internet connection

Start by cloning ArduPilot to your machine:

```
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
```

Next, switch to a stable release branch: \(At the time of writing, the most stable branch is Copter-3.5\)

```
git checkout Copter-3.5
```

Then, install some required packages:

```
sudo apt-get install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
sudo apt-get install python-scipy python-opencv ccache gawk git python-pip python-pexpect
sudo pip install future pymavlink MAVProxy
sudo pip install -U pymavlink MAVProxy
```

Add some directories to your search path:

```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
. ~/.bashrc
```

Move into the directory for ArduCopter

```
cd ardupilot/ArduCopter
```

Start the simulator:

```
sim_vehicle.py -w
```

The first time you start the simulator will take some time because it will have to first compile the program. Subsequent starts should be much faster. The first time you run it you should use the -w option as above to wipe the memory of the simulator and load the right default parameters for your vehicle.

After the default parameters are loaded you can start the simulator normally. First kill the sim\_vehicle.py you are running using Ctrl-C. Then:

```
sim_vehicle.py --console
```

Once you are in the MAVProxy console, you can load a parameter set extracted from a real vehicle by doing:

```
param load name_of_file.parm
```

Now you'll compile Gazebo:

```
curl -ssL http://get.gazebosim.org | sh
sudo apt-get install libgazebo8-dev
```

Download some custom models for a drone:

```
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

Set the path of your Gazebo models:

```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/gazebo_models' >> ~/.bashrc
source ~/.bashrc
```

Then, copy your Gazebo worlds to the correct directory:

```
sudo cp -a ~/ardupilot_gazebo/gazebo_worlds/. /usr/share/gazebo-7/worlds
```

Finally, create a way to run both Gazebo and ArduPilot easily:

```
echo "source ~/.bashrc
gazebo --verbose worlds/iris_arducopter_demo.world &> /dev/null &
cd ~/ardupilot/ArduCopter
sim_vehicle.py -f gazebo-iris
kill \$!
killall gzserver" >> ~/Desktop/start_simulators
chmod +x ~/Desktop/start_simulators
sudo mv ~/Desktop/start_simulators /usr/local/bin/
```

Close the terminal window and re-open it, and type:

```
start_simulators
```

Gazebo and ArduPilot should now launch. Congrats! You're all setup.

## Sources

[http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html](http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

[https://github.com/SwiftGust/ardupilot\_gazebo](https://github.com/SwiftGust/ardupilot_gazebo)

[http://gazebosim.org/tutorials?tut=install\_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

