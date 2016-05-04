# Emacs, this is in -*- ruby -*-
# Copyright by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss
#
# ---- REQUIREMENTS ----
#
# The virtual machine created by this Vagrantfile requires: 2 CPUs, 4GB RAM, 40GB space
# Your computer should have: virtual 4-core CPU or higher, >6GB memory, >50GB space free
#
# Install VirtualBox and Vagrant on your machine first before attempting to use this file:
# * VirtualBox: https://www.virtualbox.org/wiki/Downloads
# * Vagrant: https://www.vagrantup.com/downloads.html
#
# ---- SETUP AND USE ----
#
# The directory this file resides within is synced with
#   /vagrant
# in the VM.
#
# The intended usage is:
#   git clone https://github.com/cmcghan/vagrant-fmr2016.git
#   cd vagrant-fmr2016
#   vagrant box add shadowrobot/ros-indigo-desktop-trusty64
#   vagrant up
#   vagrant ssh
#   cd ~/catkin_ws/src/rss_work
#
# We recommend you change your password after the first login (type 'passwd').
# For X-Window forwarding through ssh in MacOSX or Linux, use 'vagrant ssh -- -X' instead
# and uncomment the "config.ssh.forward_x11 = true" at line 80 below.
#
# When you are done, close ssh and type 'vagrant suspend' or 'vagrant halt' in the shell.
# --> Note that 'vagrant destroy' is NOT recommended, since the OMPL compilation can take
# around an hour or longer to finish the provisioning process on the initial 'vagrant up'!!
#
# Note that if you are using Windows, you will need to run these commands from either
# * the shell (Start->Run->cmd)
# =OR=
# * WindowsPowerShell (recommended).
#
# Also note that, under Windows, 'vagrant ssh' will likely not work, so either
# * uncomment 'vb.gui = true' below so that VirtualBox will show the usual interface
# =OR=
# * install PuTTY+Xming for ssh access with X-Windows support (127.0.0.1, port 2222)
#   and uncomment the "config.ssh.forward_x11 = true" at line 80 below.
#
# ----REFERENCES ----
#
# This file was modified from tulip-control'S Vagrantfile:
# https://github.com/tulip-control/tulip-control/blob/master/contrib/Vagrantfile
#
# Base box used:
# https://atlas.hashicorp.com/shadowrobot/boxes/ros-indigo-desktop-trusty64
#
# We also used a modified version of the 'increase swap memory' example from:
# https://jeqo.github.io/blog/devops/vagrant-quickstart/
#
# Other references:
# https://docs.vagrantup.com/v2/getting-started/boxes.html
# https://atlas.hashicorp.com/ubuntu/boxes/trusty64
# https://github.com/tulip-control/tulip-control/blob/master/contrib/nonroot-vagrant-instructions.md
# http://www.electrictoolbox.com/PuTTY-rsa-dsa-keys/
# http://unix.stackexchange.com/questions/32907/what-characters-do-i-need-to-escape-when-using-sed-in-a-sh-script
#

Vagrant.configure(2) do |config|
  #
  # general virtual machine parameters are defined here:
  #
  
  # default box:
  config.vm.box = "shadowrobot/ros-indigo-desktop-trusty64"
  # this is 2048MB memory and 64MB video RAM; 40GB drive; 2 processors; CD/DVD drive
  # login: /home/ros --> ros(/ros)
  # login: /home/vagrant --> vagrant
  # You can also use the RSA private key under:
  #   .vagrant/machines/default/virtualbox/private_key
  # On Windows, load this into PuTTYGen to create a 'private_key.ppk' that PuTTY can use;
  # login via PuTTY+XMing using Connection->SSH->Auth->Private key file for authentication.

  # to allow X11 forwarding over ssh, uncomment the following line:
  #config.ssh.forward_x11 = true
  # if this is set to 'true', with 'vb.gui = false' below, then on
  # Windows you can still use PuTTY+Xming to see GUI windows pop-up
  
  config.vm.provider :virtualbox do |vb|
    vb.customize ["modifyvm", :id, "--name"  , "fmr2016_v0.0.1"]
    vb.customize ["modifyvm", :id, "--memory", "4096"]
    vb.customize ["modifyvm", :id, "--cpus"  , 2]
    vb.customize ["modifyvm", :id, "--chipset", "ich9"]
    # Display the VirtualBox GUI when booting the machine
    vb.gui = true
  end
  #
  # The ros-indigo-desktop-trusty64 box already has a username 'ros' with default password;
  # you can avoid creating default username 'vagrant' by specifying 'ros' as the default:
  #config.ssh.username = "ros" # default password same as username
  
  # by default, the Vagrantfile directory mounts to the shared folder /vagrant
  #config.vm.synced_folder ".", "/vagrant"

  #
  # setup commands executed at the command-line as root during first-time run are defined below:
  #
  
  # create swap file (so OMPL compilation will succeed) and install dependencies for RSE:
  config.vm.provision "shell", inline: <<-SHELL
    #!/bin/sh -e

    # 
    # create swap file (so OMPL compilation will succeed)
    #

    # size of swapfile in megabytes
    swapsize=4096
    # grep -q will kick you out immediately (quit) if a match is found
    # without -q, the -e on bash or sh would kick you out of the script
    #
    # does the swap file already exist?
    grep -q "swapfile" /etc/fstab
    # if not then create it
    if [ $? -ne 0 ]; then
      echo 'swapfile not found. Adding swapfile.'
      fallocate -l ${swapsize}M /swapfile
      chmod 600 /swapfile
      mkswap /swapfile
      swapon /swapfile
      echo '/swapfile none swap defaults 0 0' >> /etc/fstab
    else
      echo 'swapfile found. No changes made.'
    fi
    
    #
    # Note: Ubuntu X-Windows Desktop and ROS indigo are pre-installed
    # on the "shadowrobot/ros-indigo-desktop-trusty64" base box
    #

    /vagrant/install_deps.sh vagrant
  SHELL
end
