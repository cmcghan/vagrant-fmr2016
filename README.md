# vagrant-fmr2016
Vagrantfile for setting up a VM for the fmr2016 challenge (includes installation scripts) -- see fmrchallenge/fmrbenchmark on github

Installation
============

Install VirtualBox and Vagrant on your machine first before attempting to use the Vagrantfile(s) in this repository:
* VirtualBox: https://www.virtualbox.org/wiki/Downloads
* Vagrant: https://www.vagrantup.com/downloads.html

Requirements, Setup, Use
========================

See the individual Vagrantfile(s) for up-to-date information on requirements, setup, and use.

The virtual machine created by the (ROS indigo) Vagrantfile requires:
* 2 CPUs, 4GB RAM, 40GB space (dynamic)
So if you want to use this Vagrantfile, you will need:
* a virtual 4-core CPU or higher, >6GB RAM, >30GB of space free

Note that the directory  
    `./vagrant-fmr2016`  
is synced with  
    `/vagrant`  
in the VM.

Also note that these install scripts can be used separately on a native Ubuntu 14.04 install. To do so, try:
```
    sudo git clone https://github.com/cmcghan/vagrant-fmr2016.git /vagrant
    cd /vagrant
    sudo su
    ./install_deps.sh SCRIPTUSER
```
where SCRIPTUSER should be replaced with the name of your user account (e.g., vagrant).

Inside the Vagrantbox, these scripts can be rerun post-setup, after the initial `vagrant up` command, via:
```
    cd /vagrant
    sudo su
    ./install_deps.sh vagrant -f
```
where "-f" will force-reinstall the installed-from-source libraries.

For the ROS indigo Vagrantbox:
----------------------------

The intended usage is:
```
    git clone https://github.com/cmcghan/vagrant-fmr2016.git
    cd vagrant-fmr2016
    vagrant box add shadowrobot/ros-indigo-desktop-trusty64
    vagrant up
    vagrant ssh
    cd ~/catkin_ws/src/rss_work
```

License
=======

This is free software released under the terms of the BSD 3-Clause License. There is no warranty; not even for merchantability or fitness for a particular purpose. Consult LICENSE for copying conditions.

When code is modified or re-distributed, the LICENSE file should accompany the code or any subset of it, however small. As an alternative, the LICENSE text can be copied within files, if so desired.

Contact
=======

If you have any questions regarding the contents of this repository, please email Catharine McGhan at <cmcghan@cms.caltech.edu>.

-EOF-
