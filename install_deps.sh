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
#wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
#sudo apt-get -y update
#sudo apt-get -y install gazebo2 gazebo2-dbg ros-$ROSVERSION-gazebo-ros

sudo apt-get -y install ros-indigo-desktop-full # should already be installed w/Vagrantbox
sudo apt-get -y install gazebo2 ros-$ROSVERSION-gazebo-ros-pkgs ros-$ROSVERSION-gazebo-ros-control

# auto-setup of ROS indigo as version of ROS to use:
sudo -u $SCRIPTUSER echo "source /opt/ros/$ROSVERSION/setup.bash" >> /home/$SCRIPTUSER/.bashrc

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
sudo apt-get -y install cython libgmp-dev
sudo pip install pycddlib

# Python Control Systems Library: requires numpy, scipy, matplotlib, slycot
sudo apt-get -y install python-numpy python-scipy python-matplotlib
# see: https://github.com/jgoppert/Slycot.git
sudo apt-get -y install libblas-dev liblapack-dev gfortran # Slycot requires python-scipy, BLAS/LAPACK, and a Fortran compiler, "sudo apt-get -y build-dep python-scipy" should also work
sudo pip install slycot
# see: https://github.com/python-control/python-control.git
sudo pip install control

# then get/install fmrb locally
# see: https://github.com/fmrchallenge/fmrbenchmark.git

FMRBENCHMARK=/home/$SCRIPTUSER/fmrbenchmark-0.0.1

# local user install via:
cd /home/$SCRIPTUSER
if [ "$FORCE" == "-f" ]; then
    rm -rf fmrbenchmark-0.0.1
    rm -rf v0.0.1.zip
fi
sudo apt-get -y install wget
if [ "$FORCE" == "-f" ] || [ ! -f v0.0.1.zip ]
then
    sudo -u $SCRIPTUSER wget https://github.com/fmrchallenge/fmrbenchmark/archive/v0.0.1.zip
fi
if [ ! -d $FMRBENCHMARK ]; then
    sudo -u $SCRIPTUSER unzip v0.0.1.zip
fi
# local install:
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd $FMRBENCHMARK; cd tools/fmrb-pkg; pip install --user -e . ;"
# or:
#pip install fmrb
# alt. for global system-wide install:
cd $FMRBENCHMARK/tools/fmrb-pkg && sudo pip install -e .
# alt:
#sudo pip install fmrb # for most up-to-date version
# or:
#wget https://pypi.python.org/packages/7b/1a/264fc6a4376550d74ce5e8d6ef2ab9afa477169034a77e5354bd6cde74ab/fmrb-0.0.1.tar.gz
#tar xvzf fmrb-0.0.1.tar.gz
#cd fmrb-0.0.1
#sudo python setup.py install

# Section 2.2.1: build standalone example via...
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd $FMRBENCHMARK/domains/integrator_chains/dynamaestro; mkdir build; cd build; cmake ..; make"

# Section 2.2.1: test genproblem via...
#cd $FMRBENCHMARK/domains/integrator_chains/
#dynamaestro/build/genproblem | analysis/plotp.py -

# Section 2.2.2: set up a catkin workspace
cd /home/$SCRIPTUSER
sudo -u $SCRIPTUSER mkdir -p fmrb_demo/src
if [ ! -f /home/$SCRIPTUSER/fmrb_demo/src/CMakeLists.txt ]; then
    su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/fmrb_demo/src; source /opt/ros/$ROSVERSION/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_init_workspace;"
fi
if [ ! -d /home/$SCRIPTUSER/fmrb_demo/src/integrator_chains_msgs ]; then
    sudo -u $SCRIPTUSER ln -s $FMRBENCHMARK/domains/integrator_chains/integrator_chains_msgs
fi
if [ ! -d /home/$SCRIPTUSER/fmrb_demo/src/dynamaestro ]; then
    sudo -u $SCRIPTUSER ln -s $FMRBENCHMARK/domains/integrator_chains/dynamaestro
    # in fmrbenchmark-0.0.1:
    # issue with genmsg not creating libraries ("integrator_chains_msgs/VectorStamped.h") before compiling dynamaestro
    # see: http://answers.ros.org/question/52744/how-to-specify-dependencies-with-foo_msgs-catkin-packages/
    # CMakeLists.txt requires additional dependencies for each executable:
    #  add_dependencies (log integrator_chains_msgs_generate_messages_cpp)
    #  add_dependencies (dm integrator_chains_msgs_generate_messages_cpp)
    # new (added below "catkin_package ()") lines 24-25 of /home/$SCRIPTUSER/fmrb_demo/src/dynamaestro/CMakeLists.txt
    sed -i.orig '23s/catkin_package ()/catkin_package ()\n  add_dependencies (log integrator_chains_msgs_generate_messages_cpp)\n  add_dependencies (dm integrator_chains_msgs_generate_messages_cpp)/' $FMRBENCHMARK/domains/integrator_chains/dynamaestro/CMakeLists.txt
fi
if [ ! -d /home/$SCRIPTUSER/fmrb_demo/src/sci_concrete_examples ]; then
    sudo -u $SCRIPTUSER ln -s $FMRBENCHMARK/examples/sci_concrete_examples
fi
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/fmrb_demo; source /opt/ros/$ROSVERSION/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_make install;"

# Section 2.2.2: various tests via...
#0$ python $FMRBENCHMARK/domains/integrator_chains/trial-runner.py -l -f mydata.json src/sci_concrete_examples/trialconf/mc-small-out3-order3.json
#1$ source /home/$SCRIPTUSER/fmrb_demo/install/setup.bash
#1$ roslaunch sci_concrete_examples lqr.launch
#2$ source /home/$SCRIPTUSER/fmrb_demo/install/setup.bash
#2$ rostopic echo state
#2$ rostopic echo input
# Section 2.2.2.: try this post-run:
#0$ $FMRBENCHMARK/domains/integrator_chains/analysis/tdstat.py -s mydata.json
#0$ $FMRBENCHMARK/domains/integrator_chains/analysis/tdstat.py -t 0 --wordmodrep mydata.json
#0$ tdstat.py -h

#
# http://docs.fmrchallenge.org/en/v0.0.1/dubins_traffic.html
#

# Eigen
sudo apt-get -y install libeigen3-dev

# kobuki_node
sudo apt-get -y install ros-indigo-kobuki-node

# kobuki_description
sudo apt-get -y install ros-indigo-kobuki-description

# gazebo -- see above

# kobuki_gazebo
sudo apt-get -y install ros-indigo-kobuki-gazebo

# kobuki_gazebo_plugins
sudo apt-get -y install ros-indigo-kobuki-gazebo-plugins

# download fmrbenchmark as above, then...

# Section 3.2.1: set up a catkin workspace
cd /home/$SCRIPTUSER
sudo -u $SCRIPTUSER mkdir -p dubsim_workspace/src
if [ ! -f /home/$SCRIPTUSER/dubsim_workspace/src/CMakeLists.txt ]; then
    su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/dubsim_workspace/src; source /opt/ros/$ROSVERSION/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_init_workspace;"
fi
if [ ! -d /home/$SCRIPTUSER/fmrb_demo/src/dub_sim ]; then
    sudo -u $SCRIPTUSER ln -s $FMRBENCHMARK/domains/dubins_traffic/dub_sim
fi
if [ ! -d /home/$SCRIPTUSER/fmrb_demo/src/wander ]; then
    sudo -u $SCRIPTUSER ln -s $FMRBENCHMARK/domains/dubins_traffic/e-agents/wander
fi
su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; cd /home/$SCRIPTUSER/dubsim_workspace; source /opt/ros/$ROSVERSION/setup.bash; /opt/ros/$ROSVERSION/bin/catkin_make install;"

# Section 3.2.1: test this via...
#su - $SCRIPTUSER -c "source /home/$SCRIPTUSER/.bashrc; source /home/$SCRIPTUSER/dubsim_workspace/install/setup.bash; roslaunch dub_sim straightroad.launch;"
# a.k.a:
#source /home/$SCRIPTUSER/dubsim_workspace/install/setup.bash
#roslaunch dub_sim straightroad.launch

#--eof--
