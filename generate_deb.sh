#!/bin/bash

"""
To run this script
$ bash generate_deb.sh

Important: For this project, any folder that contains python scripts, must have the '__init__.py'
  file to be consider as a module.

***** IT IS REQUIRED TO SET 'webapp/settings.py/DEBUGGING_MODE' VARIABLE TO False 
  IN ORDER TO GENERATE THE CORRECT DEBIAN FILE ****
"""

#Sources:
#https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Debian-Package.html
#https://www.theconstructsim.com/how-to-build-a-local-debian-ros2-package/

#Get the building dependencies (only once)
# sudo apt update && sudo apt install python3-bloom python3-rosdep fakeroot dh-make dh-python

info() { #Function to print a message ">>>" in blue color
    echo -e "\n\e[34m>>>\e[0m ${@}"
}

substitute_chars() {
  # Use parameter expansion to substitute '_' with '-'
  local result="${1//_/-}"
  
  # Return the modified string
  echo "$result" 
}


#Package information setup
# pkg_dir='mau_functionalities' #repo name
pkg_name='navigation_server'
pkg_version='1.1.0' #info from package.xml
pkg_deb_name=$(substitute_chars "$pkg_name")

# info "package dir (repo name): ${pkg_dir}"
info "package name: ${pkg_name}"
info "deb name: ${pkg_deb_name}"
info "package version: ${pkg_version}"
sleep 2

echo "---------------------------!-----------------------------"
echo -e "\033[1;32m Generating debian package"
echo -e "\e[0m---------------------------!-----------------------------"

#Create a Debian installable package
cd ~/colcon_ws/src/$pkg_name
bloom-generate rosdebian #A summary of ros dependencies is display

#After successfully running the above command, you should get a directory named debian in the current directory, 
# which contains the compiled files and rules required to create the Debian file.
fakeroot debian/rules binary #create a Debian package

#After a while, the command should finish with no errors. The command should have created a .deb file in the directory that is one level above.
#The name of the ros2 debian file is formed by the package name, followed by the version, the Ubuntu version and the processor architecture, 
#like this ros-$ROS_DISTRO-$pkg_deb_name_$pkg_version-0jammy_amd64.deb

#Remove the temporal files
cd ~/colcon_ws/src/$pkg_name
rm -r debian/ .pybuild/ $pkg_name.egg-info/
info "Temporal files removed"

cd ~/colcon_ws/src

#Moving and renaming deb file
mv ros-$ROS_DISTRO-$pkg_deb_name'_'$pkg_version-0jammy_amd64.deb ros-$ROS_DISTRO-$pkg_deb_name.deb

#To show the deb package info
echo "---------------------------!-----------------------------"
echo -e "\033[1;32m Debian package information"
echo -e "\e[0m---------------------------!-----------------------------"
dpkg-deb --info ros-$ROS_DISTRO-$pkg_deb_name.deb

info "Debian package successfully created at ~/colcon_ws/src !!!\n"

#----------------------------------------------------------------
#To install the Debian package (including unpacking and configuring) onto the 
#file system of the hard disk, use:
# sudo dpkg --install ros-$ROS_DISTRO-$pkg_deb_name.deb
#The package is installed in the same location where the main ROS 2 installation is located (/opt/ros/{$ROS_DISTRO}/share/pkg_name)

#To find the deb pkg
# dpkg -l | grep -i ros-$ROS_DISTRO-$pkg_deb_name
#To uninstall this package
# sudo dpkg --remove ros-$ROS_DISTRO-$pkg_deb_name


