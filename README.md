# Alex_CG2111A_S2G2

In the Alex Challenge, we built a robot named Alex that can be controlled via the Internet. Equipped with various units, Alex maps the surrounding area and avoids collisions with walls and other objects. Our mission is to remotely explore and map a room using Alex, while also scanning for specific target objects.

This is repository contains the source code used in controlling Alex as well as the final project report. It does not contain the code used for the ROS, RPLidar and SLAM. Links to the repositories referenced can be found below.

## Dependencies
### Raspberry Pi
The operating system used on the RPi was a custom image created for the purposes of this module. However, any other operating system with support for ROS can be used, such as [Ubuntu Mate](https://ubuntu-mate.org/raspberry-pi/).
Others:
The OpenSSL library
```
sudo apt-get install openssl
```
TLS programming libraries
```
sudo apt-get install libssl-dev
```

### Laptop
The operating system used on the laptop was Ubuntu 20.04 on Windows Subsytem for Linux 2 (WSL2). Any other Linux distribution with support for ROS will suffice to follow this guide.
Likewise, OpenSSL and the TLS programming libraries must also be installed on your device.

## Using this repository
Running commands or editing files on the RPi can be done through ssh, VNC or connecting your RPi to a monitor (this is a noob move and not recommended unless you have not set up your RPi's internet).
1. clone this repository onto your RPi with
```
git clone https://github.com/lwenyi1/Alex_CG2111A_S2G2
```
all the RPi files needed for controlling are in the folder Alex_files.
2. clone this repository onto your laptop with the same command. All the laptop files needed for the controlling are in the folder Laptop_files.

### Setting up TLS and main control programs
1. Set up the required public and private keys, certs etc with OpenSSL. Because I am lazy, I won't be providing instructions here, but guides can be sourced easily [online](https://www.ibm.com/docs/en/rpa/21.0?topic=keys-generating-private-public-key-pair).
2. Ensure that the keys/certs for your RPi are in the folder titled Network and that the keys/certs for your laptop are the folder titled Laptop_files.
3. Change the common name, server name, client name and IP address to yours in tls-alex-server (in Alex_files/Network) and in tls-alex-client (in Laptop_files).
4. Once done, compile the programs in both folders using
_ Pi, tls-alex-server:
```
g++ tls-alex-server.cpp tls_server_lib.cpp tls_pthread.cpp make_tls_server.cpp tls_common_lib.cpp serial.cpp serialize.cpp -o tls-alex-server -pthread -lssl -lcrypto
```
_ Laptop, tls-alex-client:
```
g++ -g tls-alex-client.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -o tls-alex-client -pthread -lssl -lcrypto
```

### Setting up the Arduino Mega
1. Connect the Mega to any device with the Arduino IDE and upload the file Alex.ino (Alex_files/Alex) onto it.
2. Connect the Mega to the RPi.
3. Check that your Mega is connected to port /dev/ttyacm0. If not, update the port variable in tls-alex-server to the correct port (could be /dev/ttyusb1) and recompile the program.

### Running the main control programs
1. On the RPi, ensure you are in the Network folder, then run
```
./tls-alex-server
```
If all goes well, the server should start up, connect to the Mega and start listening for clients.
2. On the laptop, ensure you are in the Laptop_files folder, then run
```
./tls-alex-client <your IP address> 5001
```
If all goes well, the client will connect to the server successfully and you can begin sending commands over.

## ROS, RPLidar and SLAM
> The missile knows where it is, because it knows where it isn't.
Big thank you to the hardworking, actually competent individuals and organisations that made the following guides and repos. Your work is appreciated.

### Setting up ROS
The ROS version used for this project was ROS Noetic. Instructions to set it up can be found [here](https://wiki.ros.org/noetic/Installation/Ubuntu). 
Note that both your laptop and RPi must have ROS installed.

### Setting up RPLidar
We used the repository provided by Slamtec, the set up and repository can be found [here](https://github.com/Slamtec/rplidar_ros).

### Setting up SLAM
We used the repository provided by tu-darmstadt-ros-pkg, the set up and repository can be found [here](https://github.com/tu-darmstadt-ros-pkg/hector_slam).

### Running everything
1. On one terminal on the RPi, start the ROS master with:
```
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://<your IP address>:11311
roscore
```
2. On another terminal on the RPi, start the RPlidar with:
```
source ~/<name of your lidar workspace>/devel/setup.bash
roslaunch rplidar_ros rplidar.launch
```
You can verify that whether it is working by opening another terminal and running rostopic list. You should see a /scan topic.
3.  On your laptop, start the SLAM visualisation with
```
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://<your IP address>:11311
source ~/<your slam workspace>/devel/setup.bash 
roslaunch hector_slam_launch tutorial.launch
```
If all goes well, Rviz should start up and display your map. 

## Additional notes
This is just meant to be a very rough guide for lost souls interested in this project. The code here is by no means perfect (frankly, I don't think its very good, but it was good enough to complete the mod I guess) and has a lot of room for improvement. To the new CG2111A students looking for inspiration and have stumbled upon this, keep looking, I wouldn't trust my code.

For set up and troubleshooting, your best friends are YouTube and ChatGPT.
