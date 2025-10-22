In the docker_test (it may change) folder, use the following command to build the image

```
docker build -t ariel-image .
```
When the process is finished, Its time to run the docker (improve in the device group permission is needed!)

```
docker run -it --name ariel --user ros --network=host --ipc=host -v ~/volumes:/container_volume -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ~/.ssh:/home/ros/.ssh --env=DISPLAY -v /dev:/dev --device-cgroup-rule='c *:* rmw' ariel-image
```
Now, we have acess to the terminal.

Plug the UART connector in the serial port of your device (computer, NUC, raspberry) and turn on the drone.

Certify the serial port name by using the following command
```
ls -l /dev/serial/by-id
```
and you should observe something like that

```
total 0
lrwxrwxrwx 1 root root 13 Aug 20 19:51 usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0 -> ../../ttyUSB0
```

It indicates that your serial port is ttyUSB0 (make sure to read the device type, in this case is a USB_UART_A5028KBI, but it can change in the future).


Now, sourcing the New workspace (dji_ws)

```
source ~/dji_ws/devel/setup.bash
```

After building the Onboard-SDK-ROS you should configure the file sdk.launch. This is important to set the app_id and enc_key, a pair of credentials created in the DJI website to set the onboard computer able to send critical commands.

So, openning the launch file

```
nano ~/dji_ws/src/Onboard-SDK-ROS/dji_sdk/launch/sdk.launch
```

or if you prefer vim 

```
vim ~/dji_ws/src/Onboard-SDK-ROS/dji_sdk/launch/sdk.launch
```

Next, delete the current data in the file, and copy the following information inside this and change the serial_name value to the serial port seen before ("/dev/ttyUSB0"):

```
<launch>
<node pkg="dji_sdk" type="dji_sdk_node" name="dji_sdk" output="screen">
<!-- node parameters -->
<param name="serial_name" type="string" value="/dev/ttyAMA0"/>
<param name="baud_rate" type="int" value="921600"/>
<param name="app_id" type="int" value="1077139"/>
<param name="app_version" type="int" value="1"/>
<param name="align_time" type="bool" value="false"/>
<param name="enc_key" type="string" value="772d7f71ae1c4b42c767d568b3882c11ce0d63451cb0d4d
f71ca24f711e0c5dd"/>
<param name="use_broadcast" type="bool" value="false"/>
</node>
</launch>
```

You can now launch the file with the ros command, but there is still a problem (if you don't use RTK)

There is a topic trying to publish data from the RTK, but if there isnt a RTK, this node broke.

```
ERRORLOG/1 @ getValue, L315: Topic 0x26 value memory not initialized, return default[FATAL] [1755719577.268928509]: ASSERTION FAILED
	file = /opt/ros/noetic/include/ros/publisher.h
	line = 110
	cond = false
	message = 
[FATAL] [1755719577.268988182]: Call to publish() on an invalid Publisher
[FATAL] [1755719577.269010703]: 

[dji_sdk-2] process has died [pid 119, exit code -5, cmd /home/ros/dji_ws/devel/lib/dji_sdk/dji_sdk_node __name:=dji_sdk __log:=/home/ros/.ros/log/408684d6-7dff-11f0-a9e2-e8b0c596502d/dji_sdk-2.log].
log file: /home/ros/.ros/log/408684d6-7dff-11f0-a9e2-e8b0c596502d/dji_sdk-2*.log
```

So, if you don't use RTK, make sure to comment the line 259~309 in the following file

```
~/dji_ws/src/Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node_publisher.cpp 
```

Yo need to rebuild your project to make the update in the code

```

cd ~/dji_ws && rm -rf build/ devel/ && catkin_make
```

Now, you can launch sdk.launch

```
roslaunch dji_sdk sdk.launch 
```

To exit the container use

``` 
exit
```

To return to the previous created container

``` 
docker container start -i ariel
```
To open other terminals of this docker

``` 
docker exec -it ariel /bin/bash
```
