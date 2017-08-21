This is a test version of the plugin for introducing the AGX Dynamics library released by algoryx.

How to use AGXDynamicsPlugin:

1. Install AGX Dynamics

For Ubuntu Linux, AGX can be installed by using the dpkg command like:

 sudo dpkg -i agx-setup-2.19.0.0-x64-ubuntu_16.04-double.deb

2. Copy the license file to the AGX directory

AGX Dynamics is installed into /opt/Algoryx/AgX-2.19.0.0
Copy the license file to this directory.

3. Set the environment variables

The setup_env.bash script is in the AGX directory.
Add the command to execute the script to the .profile file like:
 
 cd ~
 echo "source /opt/Algoryx/AgX-2.19.0.0/setup_env.bash" >> .profile

You can immediately update the environment variables by executing:

 source ~/.profile

4. Compile AGXDynamicsPlugin

You can compile AGXDynamicsPlugin by enabling BUILD_AGX_DYNAMICS_PLUGIN In CMake.

5. Test AGX Dynamics

Create an AGXSimulator item from the menubar and locate it under a World item.
Select the item and execute the simulation, then the simulation is performed using AGX Dynamics.

Note:
Some libraries used in AGX Dynamics seem to conflict with those used in the Bullet physics library,
so AGXDynamicsPlugin and BulletPlugin cannot be used simultaneously.
Please disable BulletPlugin when you use AGXDynamicsPlugin.
