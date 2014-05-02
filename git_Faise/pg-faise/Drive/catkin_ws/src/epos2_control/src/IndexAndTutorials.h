/** \mainpage Epos2MotorController
 *
 * \section intro_sec Introduction
 *
 * There are two Tutorials: One for a existing and working setup of the EPOS2MotorController: \ref start and
 * one for \ref setup.
 *
 * \section hw_setup The Hardware Setup
 *
 * \subsection hw_power_switches The Power Switches
 *
 * 1 - (24V) Motors\n
 * 2 - (24V) Sensors (Laser-Scanner & Ultrasonic-array)\n
 * 3 - (12V) Processing (Control-PC, AD-Converter & Ethernet-Switch)\n
 * 4 - (5V) not used
 *
 */
 
 /** \page start Getting Started
  *
  \tableofcontents

  In the next steps you will get Introduction how to bring an existing roboter-setup to life.


  \section start_step1 Step1: Power on the Robot
  First you may \ref checkWires. To power on the robot afterwards, you had to press the green button (see \ref hw_setup).
  The switches 1 (EPOS2) and 3 (Control-PC) right of the power button (see /ref hw_power_switches) should be switched on.


  \section start_step2 Step2: Waiting until the robot is ready
  Some LEDs are switch on others are off. If the robot get not into this state within 5 minutes lock at \ref checkLEDs.


  \section start_step3 Step3: Drive the robot
  There are different driving modes by using the gamepad.
  When the robot doesn't move, you can switch between this driving modes by pressing the "RB"-Button of the Logitech F710.
  For other gamepads it's the button number 6.

  \note When the Logitech F710 wasn't used for a longer time it will be switched off automatically.
  It will wake up by pressing one of it's buttons (the sticks shouldn't work).
  Please remember, if you use the "RB"-Button for this, the driving mode will be changed instantly.

  \par driving mode: twoStick
  <b>Velocity and Turning with two sticks:</b>
  The right Stick (only up and down) set the forward velocity and the left stick (only left and right) set the angular velocity of the robot.

  \par driving mode: oneStick
  <b>Velocity and Turning with one stick:</b>
  Same as twoStick-mode but both directions (up - down & left - right) are set with one stick.


  \par driving mode: tankStick
  <b>One Stick for each side:</b>
  With the left stick you set the velocity for the left side of the robot and with the right stick for the right side of the robot.


  \section power_off Last Step: Power off the robot
  After driving you can power off the robot by press and hold the red power button until the Lights of the Power-panel are switched off.

*/

/** \page troubleshooting Troubleshooting
 *
 *   \tableofcontents

  \section checkWires Check the Wires
  The Controll-PC (see \ref hw_setup) should be connected to one of the EPOS2s by USB and to the power supply with a voltage of 12V.
  The "Nano Receiver" of the Controller (Logitech F710) should also Plugged into one USB-Port.


  \section checkLEDs The Meaning of the LEDs

  Here you can find some known LED-settings, the corresponding errors and their solution.

  \subsection no_connection LED: red is on
  \par Error
  No connection to a EPOS2.
  \par Solution
  Power off the robot (\ref power_off), \ref checkWires, and switch on again (\ref start_step1).

  \subsection epos2_leds EPOS2-LED: not green
  \par Error
  The LED of one or both EPOS2 is red or blinking
  \par Solution
  Wait until the LEDs of both EPOS2 is constantly green. This should happen within 2 minutes.
  Otherwise you should turn the robot off (\ref power_off) and on (press the green button) again.

*/


/** \page setup Setup a new Ubuntu with ros::groovy and the EPOS2MotorController
 *
 *   \tableofcontents
 *
  In the following tutorial you will get an step-wise introduction how to setup a x86-PC for driving a Volksbot with EPOS2-Motor-driver


  \section setup_step1 Step1: Install Xubuntu
  At first you should install an Xubuntu-OS (also a Lubuntu is possible - Kubuntu and Ubuntu should work also, but they need to much power for there Desktop-Interfaces).
  Please use the version 12.10 for 32bit-PCs (i386). You can get it for free on http://ftp.tu-chemnitz.de/pub/linux/ubuntu-cdimage/xubuntu/releases/12.10/release/.


  \section setup_step2 Step 2: Setup ROS::groovy
  You can find detailed Information on this page: http://ros.org/wiki/groovy/Installation/Ubuntu. The commands you had to type are listed here (one per line).

  \verbatim
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list'

  wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

  sudo apt-get update

  sudo apt-get install ros-groovy-desktop-full

  sudo apt-get install ros-groovy-joystick-drivers

  sudo rosdep init

  rosdep update

  echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
  \endverbatim


  \section setup_step3 Step 3: Copy the Source-Code to the Catkin-directory

  At First you should create a directory which includes later your catkin-workspace. In this folder also a "src" folder should be created.

  \verbatim
  mkdir ~/catkin_ws

  mkdir ~/catkin_ws/src
  \endverbatim

  Unpack/Copy the source-folder (epos2_control) into the directory "~/catkin_ws/src".

  Add the setup.bash from the development-path of your catkin-workspace to the .bashrc.

  \verbatim
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  \endverbatim

  \section setup_step4 Step 4: Include the EPOS- and FTDI-Library into your OS
  The EPOS- and FTDI-Library you need are included  in the source-package. The commands for copying them in the right directory
  and some additional necessary commands are listed below (one per line). More Information you can find in the Document
  "EPOS Library Integration into Ubuntu Eclipse Dokumentation.pdf", which is in the libraries-Folder of the Project-Directory
  (mostly it's "~/catkin_ws/src/epos2_control/libraries").

  Include the FTDI-Library:
  \verbatim
  sudo cp ~/catkin_ws/src/epos2_control/libraries/libftd2xx.so.1.1.12 /usr/local/lib/

  sudo ln -s /usr/local/lib/libftd2xx.so.1.1.12 /usr/local/lib/libftd2xx.so.1

  sudo ln -s /usr/local/lib/libftd2xx.so.1 /usr/local/lib/libftd2xx.so

  sudo ln -s /usr/local/lib/libftd2xx.so.1 /usr/lib/libftd2xx.so

  sudo cp ~/catkin_ws/src/epos2_control/libraries/99-ftdi.rules /etc/udev/rules.d/

  sudo gpasswd -a user dialout
  \endverbatim

  \warning Please substitute "user" in the last command (gpasswd) by your own username.

  Include the EPOS-Command-Library:
  \verbatim
  sudo mkdir /opt/lib

  sudo cp ~/catkin_ws/src/epos2_control/libraries/libEposCmd.so.4.9.1.0 /opt/lib/

  sudo ln -s /opt/lib/libEposCmd.so.4.9.1.0 /opt/lib/libEposCmd.so.4

  sudo ln -s /opt/lib/libEposCmd.so.4 /opt/lib/libEposCmd.so
  \endverbatim

   .
  You had to reload your ".bashrc" before you are able to continue with \ref setup_step5. Please type
  \verbatim source ~/.bashrc \endverbatim
  Alternatively you can just reboot your system, but this will be done in every case before starting with \ref setup_step6.

  \section setup_step5 Step 5: Compile the epos2_control-Package
  Now it should be possible to compile the package on your system. Please change into your catkin-directory and run make.

  \verbatim
  cd ~/catkin_ws/

  catkin_make
  \endverbatim

  If the "epos2_control"-package was compiled successfully, it's necessary to reboot your system before you can go on with \ref setup_step6.

  \section setup_step6 Step 6: Using the EPOS2MotorController
  For starting you need at least two Shells. One you need for the remote and the other is for the driver itself. You also should power on the robot
  (if necessary lock at \ref start_step1).

  Starting the driver:
  When you are starting the driver some parameter will be need from the parameter-server of ROS. These parameter are saved in a ROS-launch-file.
  There are two examples in the "launch"-folder of the project-directory. One was made for a 4-wheel-robot and the other for a 6-wheel-robot.
  For starting the driver with the parameters of the 4-wheel-robot type:
  \verbatim
  roslaunch epos2_control epos2_init_r4.launch
  \endverbatim

  Starting the remote:
  The remote also depend on some values from the parameter-server. But all needed values should be loaded by starting the driver.
  This means you need only one launch-file for starting the remote-node:
  \verbatim
  roslaunch epos2_control epos2_start_remote.launch
  \endverbatim

  You can send now velocity-messages to the driving node. If you want to use the keyboard, the shell with the remote-node should have the focus.
  Then you can use the keys W, S, A and D for driving. The other possibility is the usage of a gamepad or steering-wheel. You can find a list of available
  driving modes at \ref start_step3.

  \note The detection of the connect device will be done by the count of axes. So its possible that not all gamepads and steering-wheels will work.

  If you want to power off your robot lock at \ref power_off.

*/
