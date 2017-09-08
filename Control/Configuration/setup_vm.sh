printf "\nCreating directory structure.\n\n"
mkdir ~/Desktop/Development
cd ~/Desktop/Development

mkdir ~/Desktop/Development/software
cd ~/Desktop/Development/software

printf "\nDownloading VS Code.\n\n"
wget -q "https://go.microsoft.com/fwlink/?LinkID=760868"
mv index.html* vs_code.deb
sudo dpkg -i vs_code.deb

printf "\nDownloading APM Planner. \n\n"
wget -q http://firmware.ardupilot.org/Tools/APMPlanner/apm_planner_2.0.24_xenial64.deb
sudo dpkg -i apm_planner*.deb
sudo apt-get -f install -y #In case install fails.
sudo dpkg -i apm_planner*.deb

printf "\nDownloading QGroundControl.\n\n"
cd ~/Desktop
wget -q "https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage"
chmod +x QGroundControl.AppImage

printf "\nQGroundControl has been installed.\n\n"
cd ~/Desktop/Development

printf "\nCloning ArduPilot.\n\n"
git clone git://github.com/ArduPilot/ardupilot.git &> /dev/null
cd ardupilot
git checkout Copter-3.5

printf "\nInitializing git submodules.\n\n"
git submodule update --init --recursive &> /dev/null

printf "\nInstalling required packages.\n\n"
sleep 5s
yes | sudo aptdcon --safe-upgrade
yes | sudo aptdcon --install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python
yes | sudo aptdcon --install python-scipy python-opencv ccache gawk git python-pip python-pexpect
yes | sudo aptdcon --install curl cmake automake autoconf
yes | sudo aptdcon --remove --purge *modemmanager*

printf "\nInstalling required Python modules.\n\n"
sleep 5s
sudo pip install future pymavlink MAVProxy

printf "\nUpdating Python modules.\n\n"
sleep 5s
sudo pip install -U MAVProxy pymavlink

printf "\nDownloading Sentinel's parameters for ArduPilot.\n\n"
cd ~/Desktop/Development/ardupilot/ArduCopter
wget -q https://raw.githubusercontent.com/MST-MRR/IARC-2018/master/Control/Configuration/sentinel.parm

printf "\nArduPilot build complete. Setting up Gazebo directory structure... \n\n"
sleep 5s
cd ~/Desktop/Development/
mkdir gazebo_ws
cd gazebo_ws

printf "\nInstalling Gazebo...\n\n"
sleep 5s
curl -ssL http://get.gazebosim.org | sh
yes | sudo aptdcon --install libgazebo8-dev

printf "\nCreating startup script.\n\n"
echo "source ~/.bashrc
gazebo --verbose worlds/iris_arducopter_demo.world &> /dev/null &
cd ~/Desktop/Development/ardupilot/ArduCopter
sim_vehicle.py -f gazebo-iris -D
kill \$!
killall gzserver" >> ~/Desktop/start_simulators
chmod +x ~/Desktop/start_simulators
sudo mv ~/Desktop/start_simulators /usr/local/bin/

printf "\nInstalling custom Gazebo models...\n\n"
source ~/.bashrc
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo cp -a ~/Desktop/Development/gazebo_ws/ardupilot_gazebo/gazebo_worlds/. /usr/share/gazebo-8/worlds

printf "\nSetting path variables.\n\n"
echo "export PATH=/usr/lib/ccache:$PATH:$HOME/Desktop/Development/ardupilot/Tools/autotest" >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/Desktop/Development/gazebo_ws/ardupilot_gazebo/gazebo_models' >> ~/.bashrc

printf "\nReloading PATH...\n\n"
sleep 1s
source ~/.bashrc

printf "\nDownloading Sweep SDK...\n\n"
cd ~/Desktop/Development/software/
wget -q https://github.com/scanse/sweep-sdk/archive/v1.2.3.zip

unzip -q v1.2.3.zip
cd sweep-sdk*
cd libsweep

printf "\nBuilding Sweep C SDK...\n\n"
sleep 5s
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --build . --target install
sudo ldconfig

printf "\nBuilding Sweep Python SDK...\n\n"
sleep 5s
cd ../../sweeppy
sudo python2 setup.py install

sudo aptdcon clean
sudo apt-get autoremove -y
source ~/.bashrc
printf "\n\nDone.\n\n"
