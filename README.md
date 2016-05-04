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

For the ROS indigo Vagrantbox:
----------------------------

The intended usage is:

    git clone https://github.com/cmcghan/vagrant-rss.git
    cd vagrant-rss
    vagrant box add shadowrobot/ros-indigo-desktop-trusty64
    vagrant up
    vagrant ssh
    cd ~/catkin_ws/src/rss_work


Shutting down the Vagrant session:
----------------------------------

When you are done using your Vagrantbox, run  
`vagrant halt`  
to shut down the VM session.

When you wish to connect to the Vagrantbox again, try:

    vagrant up
    vagrant ssh

to start the virtual machine up again.

Choice of access to VirtualBox:
-------------------------------

Note that use of `vagrant ssh` will log you in as user "vagrant", while the VirtualBox GUI will initially log you in as user "ros".

In order to work in the VirtualBox GUI as user "vagrant", first create the Vagrantbox as per the instructions above, then run

    vagrant halt

to stop the current session.

After this:
* Open your VirtualBox application.
* Select the "fmr2016_v0.0.1" machine in the left panel, and click on "Start".

The VM session should start, and you should be logged in automatically as user "ros".

Now, to work as user "vagrant":
* Left-click on the gear symbol in the upper-right corner, and select "Log Out..."
* Click on the right-hand icon ("Log Out").
* Wait for the user menu to come up, then select user "vagrant". Default password is the login name.

You should now be logged into the VirtualBox VM as user "vagrant".


Post-install and/or Non-Vagrantbox Use
======================================

Native Ubuntu 14.04 install:
----------------------------

Note that these install scripts can be used separately on a native Ubuntu 14.04 install. To do so, try:

    sudo git clone https://github.com/cmcghan/vagrant-fmr2016.git /vagrant
    cd /vagrant
    sudo su
    ./install_deps.sh SCRIPTUSER

where **SCRIPTUSER** should be replaced with the name of your user account (e.g., **vagrant**).

Post-install updates to Vagrant VM:
-----------------------------------

Note that the directory  
`./vagrant-fmr2016`  
is synced with  
`/vagrant`  
in the VM.

Inside the Vagrantbox, these scripts can be rerun post-setup, after the initial `vagrant up` command, via:

    cd /vagrant
    sudo su
    ./install_deps.sh vagrant -f

where **-f** will force-reinstall the installed-from-source libraries.

The latter is most useful when the installation files have been updated in the github repository; one can simply `git pull` an updated version of the installation scripts and rerun them to install the new dependencies.


License
=======

This is free software released under the terms of the BSD 3-Clause License. There is no warranty; not even for merchantability or fitness for a particular purpose. Consult LICENSE for copying conditions.

When code is modified or re-distributed, the LICENSE file should accompany the code or any subset of it, however small. As an alternative, the LICENSE text can be copied within files, if so desired.


Contact
=======

If you have any questions regarding the contents of this repository, please email Catharine McGhan at <cmcghan@cms.caltech.edu>.

-EOF-
