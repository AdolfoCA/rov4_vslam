# Blue Atlas Robotics - API

## Description

This repository contains instructions for interacting with the camera system developed by Blue Atlas Robotics. The system can be easily accessed through an ftp server running on the onboard computer, by using a ROS2 based command line interface for more configuration options, and by using a simple network interface using TCP sockets.

## Table of contents

- [Description](#description)
- [Dependencies](#dependencies)
- [How to use this repository](#how-to-use-this-repository)
- [Connecting the camera system to a PC](#connecting-the-camera-system-to-a-pc)
- [Connecting the camera system to a router](#connecting-the-camera-system-to-a-router)
- [Quick start guide](#quick-start-guide)
    - [Getting started with the ftp server](#getting-started-with-the-ftp-server)
    - [Getting started with the ROS2 command line interface](#getting-started-with-the-ros2-command-line-interface)
    - [Getting started with the socket-based interface](#getting-started-with-the-socket-based-interface)
- [System overview](#system-overview)
    - [Functionality](#functionality)
    - [System power-on](#system-power-on)
    - [Power-off and restart](#power-off-and-restart)
    - [Automatic updates](#automatic-updates)
    - [How to interact with the camera system](#how-to-interact-with-the-camera-system)
- [Using the ftp server](#using-the-ftp-server)
    - [How to connect to the ftp server](#how-to-connect-to-the-ftp-server)
    - [Camera system configuration parameters](#camera-system-configuration-parameters)
    - [Camera settings](#camera-settings)
- [Using the ROS2 command line interface](#using-the-ros2-command-line-interface)
    - [How to install the custom messages](#how-to-install-the-custom-messages)
    - [Before using any ROS2 command](#before-using-any-ros2-command)
    - [Service call examples](#service-calls-examples)
    - [ROS2 diagnostics topic](#ros2-diagnostics-topic)
- [Exposed services and topics definitions](#exposed-services-and-topics-definitions)
    - [Exposed services](#exposed-services)
    - [Exposed topics](#exposed-topics)
- [Logs](#logs)
- [Troubleshooting](#troubleshooting)

## Dependencies

* Operating system :
    * Linux, Windows if using only the ftp server through a tool with graphical user interface
    * Linux, if using the ROS2 based command line interface, tested on Ubuntu 20.04
* ROS2 [Tested on foxy, on Ubuntu 20.04]
    * ROS2 foxy installation instructions https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
* avahi : In order to use the self-discovery functionality of the camera system
* [Optional] : [Filezilla](https://filezilla-project.org/) or any other tool to access the data on the onboard computer through the ftp server 

## How to use this repository

* [Note] In order to open a terminal window, you can use *ctrl+alt+t*
* Open a terminal window, and paste the following command to clone this repository : ```git clone git@github.com:Blue-Atlas-Robotics/ba_api.git```
* In order to communicate with the camera system, the user can connect it to a [PC](#connecting-the-camera-system-to-a-pc) or to a [router](#connecting-the-camera-system-to-a-router)
* Read the [Quick start guide](#quick-start-guide) to start using the API immediately, and read the following sections for a better understading on how the system works

## Connecting the camera system to a PC

If the camera system is connected to a PC the user has to configure the network adapter on his/hers device appropriately. There are two possible configurations, either using a static IP address on the host computer, or running a DHCP server. The following guides can be used to setup the network adapter:

* [Setup a static ip address to the network adapter](https://linuxize.com/post/how-to-configure-static-ip-address-on-ubuntu-20-04/) the static IP address of the host computer should be **192.168.2.1**
    * In that case, the IP of the camera system will be *192.168.2.4*
    * Alternatively, you can use the *[Serial-Number].local* to access the camera system
* [Setup a DHCP server on the host computer](https://www.linuxfordevices.com/tutorials/ubuntu/dhcp-server-on-ubuntu)
    * In that case the camera system will be assigned an IP address from the DHCP server and you can use *[Serial-Number].local* in order to access it
        * [Note] Replace [Serial-Number] with your given serial number, i.e ba-cs-101.local

## Connecting the camera system to a router

The camera system can also be connected to a router. In that case the DHCP server running on the router will assign an IP address to the camera system. In order to access it you can use the name *[Serial-Number].local*, or alternatively find the IP address of the camera system through the router's page.
* [Note] Replace [Serial-Number] with your given serial number, i.e ba-cs-101.local

## Quick start guide

* This guide will show you how to start using the system by skiping all the boring stuff
* There are two possible ways to interact with the system
    * Using the [ftp server](#getting-started-with-the-ftp-server) 
    * Using the [ROS2 command line interface](#getting-started-with-the-ros2-command-line-interface)

### Getting started with the ftp server

* You can log into the ftp server using the terminal, or by using a tool with a graphical interface, i.e [Filezilla](https://filezilla-project.org/)
    * Use the credentials from section [How to connect to the ftp server](#how-to-connect-to-the-ftp-server)
* Through the ftp server you can:
    * Download the videos to your computer, from the directory */storage/logs/YYYY-MM-DD/flight_x*
    * Modify the camera system settings file, */storage/config/camera_system_parameters_user.yaml*, details on the parameters can be found in section [Camera system configuration parameters](#camera-system-configuration-parameters)
    * Modify the camera parameters file, */storage/config/camera_setting.yaml*

### Getting started with the ROS2 command line interface

* [Note] To open a terminal use *ctrl+alt+t*
* In order to use the ROS2 command line interface, the first step is to [install the custom messages](#how-to-install-the-custom-messages)
* Before you start typing the commands below, make sure to **source** the installation file
    * Open a terminal:
        * Go to the *ba_api* directory : ```cd /path/to/ba_api```
        * Got to the *ros2_ws* directory : ```cd ros2_ws```
        * Source the ROS2 installation file : ```source install/setup.bash```
* For more detailed information look at the [Service calls examples](#service-calls-examples) section
* However, an overview of the commands is shown below :
    * **Adjust the light brightness** : ```ros2 service call /ba/light_controller/set_brightness ba_msgs/srv/SetBrightness "{channel: 3, brightness: 20}"```
        * The value for brightness is in the range [0-100]
        * The channel values are : 1 ,2, 3, corresponding to light chains 1, 2, 3    
    * **Start the recording when the system boots** : Set the parameter **record_on_startup** on the file */storage/config/camera_system_parameters_user.yaml* to **true**
    * **Start the recording manually** : ```ros2 service call /ba/camera_feed_manager/start_recording ba_msgs/srv/StartRecording```
    * **Stop the recording**: ```ros2 service call /ba/camera_feed_manager/stop_recording ba_msgs/srv/StopRecording```
        * To check the **status of the recording and synchronizer module** use the */diagnostics* topic : ```ros2 topic echo /diagnostics```
    * **Add streaming consumer** (send camera feed to a topside computer) : 
        ```ros2 service call /ba/camera_feed_manager/attach_streaming_consumer ba_msgs/srv/AttachStreamingConsumer "{camera_device_path: '/dev/videoX', output_stream_path: 'rtp://IP:PORT'}"```
        * [NOTE] ```/dev/videoX``` : **X** corresponds to the camera index which is in range [0, 7] but it depends on the number of the connected cameras, i.e */dev/video1*
        * ```IP``` is the IP of a topside computer, i.e. *192.168.2.1*
        * ```PORT``` is the port used to stream that video, i.e. *5000*
        * [Note] To visualize the video stream use the command :
            ```gst-launch-1.0 udpsrc port=PORT ! application/x-rtp ! rtph265depay ! avdec_h265 ! videoconvert ! xvimagesink sync=false async=false```
            * [NOTE] Replace **PORT** with the port you're streaming the video to, i.e *5000*
    * **Stop camera from streaming**:
        ```ros2 service call /ba/camera_feed_manager/detach_streaming_consumer ba_msgs/srv/DetachConsumer "{camera_device_path: '/dev/videoX'}"```
            * [NOTE] ```/dev/videoX``` : **X** corresponds to the camera index which is in range [0, 7] but it depends on the number of the connected cameras, i.e */dev/video1*

### Getting started with the socket-based interface

* The socket-based interface offers a way to communicate with the camera system using TCP sockets
* [Note] To open a terminal use *ctrl+alt+t*
* Open a terminal :
    * To start recording : ```echo start_recording | socat - tcp:IP:8080```
    * To stop recording : ```echo stop_recording | socat - tcp:IP:8080```
    * To check if the system is recording : ```echo is_recording | socat - tcp:[Serial-Number].local:8080```
    * To restart the system : ```echo restart | socat - tcp:IP:8080```
    * To control the brightness of the lights : ```echo set_brightness channel brightness | socat - tcp:[Serial-Number].local:8080```
        * The value for brightness is in the range [0-100]
        * The channel values are : 1 ,2, 3, corresponding to light chains 1, 2, 3 
    * [NOTE] Relpace the `[Serial-Number]` attribute to the serial number of your camera system, i.e `ba-cs-101`

## System overview

* The camera system can record synchronized videos from up to 8 cameras, and stream up to 2 cameras to the topside computer
* The camera system is comprised of 
    * a main Tube containing the onboard computer, the DC-DC converter for power distribution, along with a battery monitor, and leak sensors
    * 1-8 full HD cameras with underwater enclosures 
    * a battery tube containing the battery, supplying power to the lights, and main tube
    * up to 4 light chains with up to 6 lights per each chain, for a total maximum of 24 lights connected to the system 
* [camera_system_drawing](/camera_system_drawing.pdf) contains the drawings/schematics of the camera system and its subcomponents

### Functionality

* The system can record synchronized full HD videos from up to 8 cameras, and store them on the onboard 2TB SSD
    * The available encodings are h264(.mp4 file) and h265(.mkv file)
    * The user can add/remove cameras to record in real time, using ROS2
    * Use the parameter file to enable automatic recording on boot
    * Each time a recording starts a new video file is created on the SSD, section [Logs](#logs) contains information regarding the video log directories
* The system can also stream up to 2 cameras using [gstreamer](https://gstreamer.freedesktop.org/)
    * The user can select which camera to stream and on which IP address and port
    * The streaming feature is only available when using the ROS2 command line tool
    * The streamed video feeds are not synchronized
* The user can adjust the camera settings in real time, e.g brightness or contrast using a config file located on the SSD
* The user can place the update files (.deb) in the */storage/updates* directory and the system will [automatically update](#automatic-updates) those packages the next time it starts-up 

### System power-on

* When the power is connected to the main tube the system will automatically power on
* It can take up to 2 minutes (depending on the number of connected cameras) from the moment the battery is connected, until the user can intereact with the cameras 
* The user can configure the system to automatically record once the camera initialization is finished, or start the recording manually using the command line interface
* The user can select which cameras should record on startup and which encoding they should use

### Power-off and restart

* In order to power-off the system the battery connector needs to be removed from the main tube
* In order to reboot the system the battery connector needs to be unpluged and pluged in to the main tube again

### Automatic updates

* Download the .deb files from Blue-Atlas-Robotics
* Place the .deb files on the */storage/updates* folder using the ftp server
* Reboot the system in order to automatically install the packages

### How to interact with the camera system

* There are two possible ways of interacting with the system
    * [Using the ftp server](#using-the-ftp-server) running on the onboard computer
    * [Using the ROS2 command line interface](#using-the-ros2-command-line-interface) from the topside computer

## Using the ftp server

* Using the ftp server is the simplest way to interact with the camera system
* Through the ftp server you can 
    * modify the system parameters and camera settings
    * Download the recorded videos from the SSD
    * Add updates to the SSD so that the system can automatically update those packages

### How to connect to the ftp server

* To connect to the ftp server you can use the credentials below :
    ```
    IP address : [system serial number].local
    username : user
    password : user
    ```
    * [NOTE] Those credential are the default ones, the user can alter the credentials upon delivery of the camera system
* Example of connecting to the ftp server using the terminal : 
    * [Note] To open a terminal you can use *ctrl+alt+t*
    * Open a terminal window and type the command : ```ssh USERNAME@ba-cs-101.local```
    * Replace **USERNAME** with your username, 
    * On the next promt type in your password
* Otherwise a tool with graphical interface can be used with the credentials above
    * In order to use filezilla to connect to the ftp server you can follow this [tutorial](https://filezillapro.com/docs/v3/basic-usage-instructions/connecting-to-a-server/)

### Camera system configuration parameters

* The parameters are located in */storage/config/camera_system_parameters_user.yaml*
* Parameter explanation:
    * camera_feed_manager:
        * The *camera_feed_manager* is the node controlling the cameras on the camera system, and it has the following attributes : 
            * `total_cameras` : Paths to all of the cameras connected to the Xavier
            * `default_recording_cameras` : The cameras that can record once the system boots
            * `default_encoding` : The default encoding used unless specified otherwise 
                * h265->.mkv
                * h264->.mp4
            * `timeout` : Max time to wait for all cameras to initialize, the more cameras the bigger the timeout should be
                * [Note] for all 6 cameras the default timeout is **50** meaning that the system will wait up to 50 seconds until all cameras are detected, if a camera is not detected before the timeout ellapses then that camera won't be available to the user, the available cameras are shown on the [diagnostics topic](#ros2-diagnostics-topic)
            * `record_on_startup` : **True** if the camera should start recording on startup using the *cameras_to_record*
* Example of the parameter file:
```
ba:
    camera_feed_manager:
        ros__parameters:
        total_cameras: [/dev/video0, /dev/video1, /dev/video2, /dev/video3, /dev/video4, /dev/video5]
        default_recording_cameras: [/dev/video0, /dev/video1, /dev/video2]
        default_encoding: h265
        timeout: 50.0
        record_on_startup: false
```

### Camera settings

* For the **Nile25** camera, the available settings are:
    ```
    User Controls

                        brightness 0x00980900 (int)     : min=-15 max=15 step=1 default=0 value=0 flags=slider
                        contrast 0x00980901 (int)       : min=0 max=30 step=1 default=10 value=10 flags=slider
                        saturation 0x00980902 (int)     : min=0 max=60 step=1 default=16 value=16 flags=slider
            white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                            gamma 0x00980910 (int)      : min=40 max=500 step=1 default=220 value=220 flags=slider
                            gain 0x00980913 (int)       : min=1 max=40 step=1 default=1 value=15
                    horizontal_flip 0x00980914 (bool)   : default=0 value=0
                    vertical_flip 0x00980915 (bool)     : default=0 value=0
        white_balance_temperature 0x0098091a (int)      : min=10 max=10000 step=10 default=4600 value=4600
                        sharpness 0x0098091b (int)      : min=0 max=127 step=1 default=16 value=16 flags=slider

    Camera Controls

                    exposure_auto 0x009a0901 (menu)     : min=0 max=2 default=0 value=1
            exposure_time_absolute 0x009a0902 (int)     : min=1 max=10000 step=1 default=312 value=5
                    roi_window_size 0x009a0924 (int)    : min=8 max=64 step=8 default=8 value=8 flags=slider
                    roi_exposure 0x009a0926 (int)       : min=0 max=65535 step=1 default=32896 value=32896 flags=slider
                        frame_sync 0x009a092a (menu)    : min=0 max=2 default=0 value=1
                            denoise 0x009a092d (int)    : min=0 max=15 step=1 default=8 value=8 flags=slider
            exposure_compensation 0x009a0931 (int)      : min=8000 max=1000000 step=1 default=16000 value=32000 flags=slider
                        bypass_mode 0x009a2064 (intmenu): min=0 max=1 default=0 value=0
                    override_enable 0x009a2065 (intmenu): min=0 max=1 default=0 value=0
                    height_align 0x009a2066 (int)       : min=1 max=16 step=1 default=1 value=1
                        size_align 0x009a2067 (intmenu) : min=0 max=2 default=0 value=0
                write_isp_format 0x009a2068 (int)       : min=1 max=1 step=1 default=1 value=1
        sensor_signal_properties 0x009a2069 (u32)       : min=0 max=4294967295 step=1 default=0 [30][18] flags=read-only, has-payload
            sensor_image_properties 0x009a206a (u32)    : min=0 max=4294967295 step=1 default=0 [30][16] flags=read-only, has-payload
        sensor_control_properties 0x009a206b (u32)      : min=0 max=4294967295 step=1 default=0 [30][36] flags=read-only, has-payload
                sensor_dv_timings 0x009a206c (u32)      : min=0 max=4294967295 step=1 default=0 [30][16] flags=read-only, has-payload
                low_latency_mode 0x009a206d (bool)      : default=0 value=1
                preferred_stride 0x009a206e (int)       : min=0 max=65535 step=1 default=0 value=0
                    sensor_modes 0x009a2082 (int)       : min=0 max=30 step=1 default=30 value=3 flags=read-only
    ```
* The file */storage/config/camera_setting.yaml* contains the settings that are applied to all the cameras
    * An example of those settings is :
        ```
        brightness:
            default: 0
            flags:
            - slider
            id: 9963776
            max: 15
            min: -15
            name: brightness
            step: 1
            type: int
            value: 0
        ```
* In order to change those settings, 
    * open the above file */storage/config/camera_setting.yaml* using the ftp server 
    * Choose one of the available settings, i.e brightness
    * Write your *desired setpoint* in the **value** attribute
    * [NOTE] The *desired setpoint* should be between the *min* and *max* values 
* Those settings are automatically applied when the system boots
* The user can change the settings when the system is running by editing this file, but needs to use the ROS2 command line interface to apply them without rebooting
* [WARNING] Modifying some settings may cause errors to the synchronizer module

## Using the ROS2 command line interface

* The ROS2 command line interface allows you to configure the camera system in real time
* In order to run the commands in the following sections, you need to [install the custom messages](#how-to-install-the-custom-messages), and [source](#before-using-any-ros2-command) the workspace before use
* use the [ROS2 services](#service-calls-examples) in order to configure the system, i.e start/stop recording, add/remove streaming camera
* use the [ROS2 diagnostics topic](#ros2-diagnostics-topic) in order to get the status of the system

### How to install the custom messages

* [Note] To open a terminal you can use *ctrl+alt+t*
* Open a terminal:
    * Go to the *ba_api* directory : ```cd /path/to/ba_api```
    * Got to the *ros2_ws* directory : ```cd ros2_ws```
    * Source the ROS2 installation file : ```source install/setup.bash```

### Before using any ROS2 command

* [Note] To open a terminal you can use the key combination *ctrl+alt+t*
* Open a terminal window, and paste the commands below
```
    cd /path/to/ba_api/ros2_ws
    source install/setup.bash
```
* [Note] The *source* command will let ROS2 know about the definitions of the custom *ba_msgs*, so that you can use them, therefore **run** the source command before running the commands on the next section. This has to be done inside of every new terminal window.

### Service calls examples

* Adjust the light brightness : ```ros2 service call /ba/light_controller/set_brightness ba_msgs/srv/SetBrightness "{channel: 3, brightness: 20}"```
    * The value for brightness is in the range [0-100]
    * The channel values are :
        * **1** : for light chain 1
        * **2** : for light chain 2
        * **3** : for light chain 3
* Control the video recording:
    * **Add camera to the recording devices list** : ```ros2 service call /ba/camera_feed_manager/attach_recording_consumer ba_msgs/srv/AttachRecordingConsumer "{camera_device_path: '/dev/videoX', encoding: 'h265'}"```
        * [Argument] `camera_device_path` : The full path of the camera device, i.e */dev/video0*
        * [Argument] `encoding` : The endoding for the added video stream
            * It is an optional argument, the default value can be set in the file */storage/config/camera_system_parameters_user.yaml*
        * [NOTE] ```/dev/videoX``` : **X** corresponds to the camera index which is in range [0, 7] but it depends on the number of the connected cameras, i.e */dev/video1*
        * [Note] : Available encodings are :
            * **h264** which is stored as an **.mp4** file
            * **h265** which is stored as an **.mkv** file
        * [Note] : The recorded videos are synchronized
        * [WARNING] : You can add a new camera to the recording's list, **but** if the system is already recording, you need to stop and start recording again in order to store the video from the newly added camera
    * **Remove camera from the recording devices list** : ```ros2 service call /ba/camera_feed_manager/detach_recording_consumer ba_msgs/srv/DetachConsumer "{camera_device_path: '/dev/videoX'}"```
        * [Argument] `camera_device_path` : The full path of the camera device, i.e */dev/video0*
        * [NOTE] ```/dev/videoX``` : **X** corresponds to the camera index which is in range [0, 7] but it depends on the number of the connected cameras, i.e */dev/video1*
        * [Note] : if the camera is removed while the system is recording, the recording of that camera will stop immediately
    * **Start the recording when the system boots** : Set the parameter **record_on_startup** on the file */storage/config/camera_system_parameters_user.yaml* to **true**
    * **Start the recording manually** : ```ros2 service call /ba/camera_feed_manager/start_recording ba_msgs/srv/StartRecording```
    * **Stop the recording** : ```ros2 service call /ba/camera_feed_manager/stop_recording ba_msgs/srv/StopRecording```
    * To **check the status of the recording and synchronizer** module use the */diagnostics* topic : ```ros2 topic echo /diagnostics```
* Control the video streaming:
    * **Add streaming consumer** (send camera feed to a topside computer) : 
        ```ros2 service call /ba/camera_feed_manager/attach_streaming_consumer ba_msgs/srv/AttachStreamingConsumer "{camera_device_path: '/dev/video0', output_stream_path: 'rtp://IP:PORT'}"```
        * [Argument] `camera_device_path` : The full path of the camera device, i.e */dev/video0*
        * [Argument] `output_stream_path` : The full path of the output stream, i.e *rtp://192.168.2.1:5000*
            * ```IP``` is the IP of a topside computer, i.e. *192.168.2.1*
            * ```PORT``` is the port used to stream that video, i.e. *5000*
                * [WARNING] If you attach multiple streams to the same IP address the image will be distorted, remember to assign **different ports**
        * [NOTE] ```/dev/videoX``` : **X** corresponds to the camera index which is in range [0, 7] but it depends on the number of the connected cameras, i.e */dev/video1*
        * [Note] The streaming cameras are not synchronized
        * [WARNING] Only one *output_stream_path* can be attached per camera, attaching a new one will replace the old one
        * [Note] To **visualize the video stream** use the command :
            ```gst-launch-1.0 udpsrc port=5000 ! application/x-rtp ! rtph265depay ! avdec_h265 ! videoconvert ! xvimagesink sync=false async=false```
            * Adjust the port, i.e **5000**, to match the port used on the service call command, i.e output_stream_path: 'rtp://192.168.2.1:**5000**'
    * **Stop camera from streaming**:
        ```ros2 service call /ba/camera_feed_manager/detach_streaming_consumer ba_msgs/srv/DetachConsumer "{camera_device_path: '/dev/videoX'}"```
        * [Argument] `camera_device_path` : The full path of the camera device, i.e */dev/video0*
    * **Stop all streaming cameras** : ```ros2 service call /ba/camera_feed_manager/stop_streaming ba_msgs/srv/StopStreaming```
* **Update camera parameters** :  ```ros2 service call /ba/camera_feed_manager/update_camera_parameters std_srvs/srv/Empty```
    * [NOTE] The user can [manually change the camera settings](#camera-settings), i.e. brightness, located in */storage/config/camera_settings.yml*, and in order to apply the new settings without reboot use the above command  
* **Get the current log_dir**: ```ros2 service call /ba/get_log_dir ba_msgs/srv/GetLogDir```

### ROS2 diagnostics topic

* The diagnostics topic contains diagnostic messages from most of the nodes running on the system
* To get the diagnostic messages use the command: ```ros2 topic echo /diagnostics```
* Contents:
    * Xavier status
        * Available space on the SSD
        * CPU Temperature
        * Running time
        * IPv4 address for eth0
        * IPv4 address for eth1
    * Camera system status
        * Available devices
        * Cameras that can record video
        * Current recording duration
        * Synchronizer status
        * Synchronized video fps
        * max_t_diff_us : The max difference in usec between the timestamps of each video
    * Light controller
        * Status -> Connected/disconnected
    * Battery sensor 
        * Status -> Connected/disconnected
        * Battery voltage
    * Leak detector
        * Leak detected or not

## Exposed services and topics definitions

The subsections below contain information regarding the exposed topics and services that can be used to interact with the system

### Exposed services

* `/ba/camera_feed_manager/attach_streaming_consumer` : Adds a streaming consumer to a non-synchronized video stream. Attaching a new consumer on the same device will overwrite the previous one. Currently the only available options are `rtp://...`, `rtmp://...`.
* `/ba/camera_feed_manager/attach_recording_consumer` : Adds a recording consumer to a synchronized video stream. Attaching a new consumer on the same device will ovewrite the previous one. If the system is already recording then the new video stream will be included once the current recording stops. Currently the only available encodings are `h265`, `h264`.
* `/ba/camera_feed_manager/detach_streaming_consumer` : Detach a streaming consumer from a non-synchronized video stream. 
* `/ba/camera_feed_manager/detach_recording_consumer` : Detach a recording consumer from a synchronized video stream, it can be done even if the system is recording, the recording of that camera will stop immediately when the service is called
* `/ba/camera_feed_manager/start_recording` : Start the recording using the predifined cameras
* `/ba/camera_feed_manager/stop_recording` : Stop the recording using the predifined cameras
* `/ba/camera_feed_manager/stop_streaming` : If there are active streaming cameras, stop the streaming

### Exposed topics

* `/ba/get_log_dir` : Publishes the current log directory

## Logs

* The logs are saved in the directory /storage/logs/YYYY-MM-DD/flightxxxx/
* The ros logs are saved in /storage/fishbot/logs/YYYY-MM-DD/.ros
* The kernel logs are saved periodically in the latest flight
* The systemd status msgs are stored in the latest flight

## Troubleshooting

* Video feed on the topside is pixalated or has poor quality,
    * [Possible cause] : You're streaming multiple cameras to the same port
        * [Solution] : Stop all cameras from streaming, and attach them again
* *Failed to load entry point 'call': librcl_interfaces__rosidl_typesupport_c.so: cannot open shared object file: No such file or directory*
    * [Possible cause] The custom *ba_msgs* weren't build in the system
        * [Solution] Follow the instructions in the section [How to install the custom messages](#how-to-install-the-custom-messages)
    * [Possible cause] Forgot to source the *ba_msgs* workspace
        * [Solution] Follow the instructions in the section [Before using any ROS2 command](#before-using-any-ros2-command)