#!/bin/bash -e
# Copyright by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-fmr2016

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

#
# Note: Ubuntu X-Windows Desktop and ROS indigo are pre-installed
# on the "shadowrobot/ros-indigo-desktop-trusty64" base box
#

echo "Start of install_deps.sh script!"
echo "input arguments: [SCRIPTUSER] [FORCE (-f)]"
echo "(note: order of [SCRIPTUSER] and -f argument can be swapped)"

# set defaults for input arguments
ROSVERSION=indigo
SCRIPTUSER=vagrant
FORCE=
# if we get an input parameter (username) then use it, else use default 'vagrant'
# get -f (force) if given
echo "ROS version is $ROSVERSION."
if [ $# -gt 0 ]; then # at least 1 (possibly more) arguments at commandline...
    if [ "$1" == "-f" ]; then
        echo "-f (force) commandline argument given."
        FORCE=$1
    else
        echo "Username given as commandline argument."
        SCRIPTUSER=$1
    fi
    if [ $# -gt 1 ]; then # at least 2 (possibly more) arguments at commandline...
        if [ "$2" == "-f" ]; then
            echo "-f (force) commandline argument given."
            FORCE=$2
        elif [ "$SCRIPTUSER" -eq "vagrant" ]; then
            echo "Username given as commandline argument."
            SCRIPTUSER=$2
        else
            echo "Username already set. Second argument ignored."
        fi
    fi
fi
echo "Will be using user $SCRIPTUSER and directories at and under /home/$SCRIPTUSER..."
if [ "$FORCE" -eq "-f" ]; then
    echo "Forcing install of all compiled-from-source components."
fi

#
# run installation + upgrades
#

# update all packages, because "gah!" otherwise, especially for 'rosdep' stuff later
sudo apt-get -y update
sudo apt-get -y upgrade

# install recommended software for python development (python-pip already installed above)
sudo apt-get -y install spyder geany python-dev build-essential dos2unix

#for fmrbenchmark 0.0.1:

# http://docs.fmrchallenge.org/en/v0.0.1/intro.html#support-for-platforms-and-programming-languanges

#
# install ROS indigo (for "ubuntu/trusty64" box)
# --> comment out for "shadowrobot/ros-indigo-desktop-trusty64" box (pre-installed)
#
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
#sudo apt-get -y update
#sudo apt-get -y install ros-$ROSVERSION-desktop-full # will not hurt anything if preinstalled
#sudo rosdep init
#su - $SCRIPTUSER -c "rosdep update;"
#sudo apt-get -y install python-rosinstall
#
#sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
#mkdir ~/initdeps
#cd ~/initdeps
#wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
#sudo apt-get -y update
#sudo apt-get -y install gazebo2 gazebo2-dbg ros-$ROSVERSION-gazebo-ros

sudo apt-get -y install ros-indigo-desktop-full # should already be installed w/Vagrantbox
#sudo apt-get -y install gazebo gazebo-dev ???

#
# http://docs.fmrchallenge.org/en/v0.0.1/integrator_chains.html
#

# Eigen
sudo apt-get -y install libeigen3-dev

# NumPy
sudo apt-get -y install python-numpy

# MatplotLib
sudo apt-get -y install python-matplotlib python-matplotlib-data python-matplotlib-doc

#pycddlib
sudo apt-get -y install python-pip
sudo apt-get -y install cython
sudo pip install pycddlib

# Python Control Systems Library: requires numpy, scipy, and matplotlib
sudo apt-get -y install python-scipy
# from https://github.com/jgoppert/Slycot.git
sudo pip install slycot
# from https://github.com/python-control/python-control.git
sudo pip install control

# then get/install fmrb locally:

# local user install via:
cd /home/$SCRIPTUSER
if [ "$FORCE" == "-f" ]; then
    rm -rf fmrbenchmark
fi
if [ ! -d fmrbenchmark ]; then
    sudo -u $SCRIPTUSER git clone https://github.com/fmrchallenge/fmrbenchmark.git
fi
cd fmrbenchmark
cd tools/fmrb-pkg
sudo -u $SCRIPTUSER pip install -e .
# or:
#pip install fmrb
# for global system-wide install:
sudo pip install fmrb

# set up catkin workspace
#cd /home/$SCRIPTUSER
#sudo -u $SCRIPTUSER mkdir -p catkin_ws/src/rss_work
#su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/catkin_ws/src; source /opt/ros/$ROSVERSION/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_init_workspace; cd ..; /opt/ros/$ROSVERSION/bin/catkin_make;"
#sudo -u $SCRIPTUSER echo "source /opt/ros/$ROSVERSION/setup.bash" >> /home/$SCRIPTUSER/.bashrc
#sudo -u $SCRIPTUSER echo "source /home/$SCRIPTUSER/catkin_ws/devel/setup.bash" >> /home/$SCRIPTUSER/.bashrc
##sudo chown -R $SCRIPTUSER:$SCRIPTUSER /home/$SCRIPTUSER/.bashrc

#
# http://docs.fmrchallenge.org/en/v0.0.1/dubins_traffic.html
#

# Eigen
sudo apt-get -y install libeigen3-dev

# kobuki_node
sudo apt-get -y install ros-indigo-kobuki-node

# kobuki_description
sudo apt-get -y install ros-indigo-kobuki-description

# gazebo
#sudo apt-get -y install gazebo gazebo-dev ???

sudo apt-get -y install ros-indigo-kobuki-gazebo

# kobuki_gazebo_plugins
sudo apt-get -y install ros-indigo-kobuki-gazebo-plugins


#--eof--
