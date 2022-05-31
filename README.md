# ublox_utils
Repository with ROS tools for the u-blox RTK receiver.

## Table of Contents
1. [Installation](#installation)
2. [Hardware Setup](#hardware-setup)  
     1. [Electronics](#electronics)
     2. [Part List](#part-list)
     3. [Firmware](#firmware)
4. [ROS Sensor Launch](#ros-sensor-launch)  
     1. [Single Receiver](#single-receiver)
     2. [Dual Receivers](#dual-receivers)
     3. [NTRIP Corrections](#ntrip-corrections)
     4. [Dual Receivers with NTRIP](#dual-receivers-with-ntrip)

## Installation
```
source /opt/ros/noetic/setup.bash
export ROS_VERSION=`rosversion -d`

sudo apt install -y python3-catkin-tools \
  ros-$ROS_VERSION-ublox \
  ros-$ROS_VERSION-ntrip-client \
  ros-$ROS_VERSION-nmea-msgs \
  ros-$ROS_VERSION-mavros-msgs \
  ros-$ROS_VERSION-rtcm-msgs
  
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/ublox_utils.git
catkin build
```

## Hardware Setup
This section describes the hardware setup with one or two receivers on the rover.
The following considerations should be taken into account:
- Optimal **sky coverage** of the antennas.
- Antennas **far distanced** from other electronics to avoid interference (especially LiDAR and USB 3.0 devices).
- **Omnidirectional helical antennas** preferable over patch antennas as they do not require ground plane and have a better attitude coverage (see [ZED-FP9 Moving base applications, p.8](https://content.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_%28UBX-19009093%29.pdf)). 
- **Multi-band antennas** that support all receiver frequencies with well-defined phase center and small phase center variation (see for example [HC882 Dual-Band Helical Antenna + L-Band](https://www.tallysman.com/app/uploads/2018/03/Tallysman%C2%AE-HC882-Datasheet_March-2022.pdf)). 
- **Known phase center position** with respect to rover body frame (see r_BP and r_BM in Figure below).
- Rover antenna **at least 20cm** distanced from moving base antenna (see [ZED-FP9 Moving base applications, p.7](https://content.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_%28UBX-19009093%29.pdf)).

![Hardware GNSS setup.](https://user-images.githubusercontent.com/11293852/169337168-dd9f23a8-5c68-41e9-bf57-185111bd45fb.png)

### Electronics
The NTRIP setup does not require a base station.
Instead the corrections are provided by an NTRIP caster online.
The wiring is shown in the Figure below.
In case of a moving baseline setup, corrections from the moving base receiver (Receiver 1) will be sent to the rover (Receiver 2) via UART2.

![Electronic wiring of dual RTK setup.](https://user-images.githubusercontent.com/11293852/169337161-7a531299-0cdd-4294-901e-e4295f50c316.png)

### Part List
This is the part list for the standard moving baseline setup above.
The prices are regular retail prices.
For some, e.g., NTRIP corrections or Tallysman antennas research discounts may apply.

| No. | Pc. No. | Description                                  | Source Object   | Supplier  | Supplier Number     | Cost per Unit   | Cost Total      | 
|:---:|:-------:|:---------------------------------------------|:----------------|:----------|:--------------------|:----------------|:----------------|
|  1  |    2    | Sparkfun u-blox ZED-F9P GPS-RTK-SMA receiver | GPS-16481       | digikey   | 1568-GPS-16481-ND   | CHF 260.-       | CHF 520.-       |
|  2  |    2    | Tallysman HC882 dual-band helical antenna    | 33-HC882-28     | digikey   | 1526-33-HC882-28-ND | CHF 250.-       | CHF 500.-       |
|  3  |    1    | Swipos GIS/GEO NTRIP license                 | swipos GIS/GEO  | swisstopo | swipos-GIS/GEO      | CHF 2000.- p.a. | CHF 2000.- p.a. |
|  4  |    2    | USB-C connector cable (300mm)                | CAB-15426       | digikey   | 1568-CAB-15426-ND   | CHF 4.-         | CHF 8.-         |
|  5  |    2    | Coaxial SMA to SMA (500mm)                   | 415-0031-MM500  | digikey   | J10300-ND           | CHF 16.-        | CHF 32.-        |

### Firmware
The latest tested firmware was **1.32** on ZED-F9P receiver. 
To update the firmware follow the [sparkfun tutorial](https://learn.sparkfun.com/tutorials/how-to-upgrade-firmware-of-a-u-blox-gnss-receiver).

When using a dual receiver setup update the configuration of the [moving base receiver](config/moving_base.txt) and [rover receiver](config/rover.txt).
The configuration is elaborated in [ZED-FP9 Moving base applications, p.16](https://content.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_%28UBX-19009093%29.pdf).
To flash the configuration go to u-center and select `View->Generation 9 Configuration View->Advanced Configuration->Load from file...->Send config changes`

**Note**: If you are only using a single rover, the default config is fine.

To revert to the default config select `View->Messages view->UBX->CFG->CFG->Revert to default configuration->Send`.
## ROS Sensor Launch
[ublox.launch](./launch/ublox.launch) provides the launch file to start the u-blox receivers ROS interface.

### Single Receiver
The default startup with a single receiver is
```
roslaunch ublox_utils ublox.launch device_position_receiver:=/dev/ttyACM0
```
The relevant ROS topics are
```
/ublox_position_receiver/fix
/ublox_moving_baseline_receiver/fix_velocity
```

Additionally, you should monitor the fix status.
```
/ublox_position_receiver/navstatus
```

### Dual Receivers
At first use, setup the [firmware](#firmware).

To startup two receivers where the second receiver estimates the moving baseline run
```
roslaunch ublox_utils ublox.launch device_position_receiver:=/dev/ttyACM0 use_moving_baseline:=true device_moving_baseline_receiver:=/dev/ttyACM1
```
The relevant ROS topic is
```
/ublox_moving_baseline_receiver/navheading
```
Additionally, you should monitor a valid fix flag as well as the distance between the antennas in
```
/ublox_moving_baseline_receiver/navrelposned
```

**Note**: You may want to find a smart way to allocate the device ID.

### NTRIP Corrections
We use the [swipos-GIS/GEO](https://www.swisstopo.admin.ch/de/geodata/geoservices/swipos/swipos-dienste/swipos-gisgeo.html) caster in conjunction with the [ROS ntrip client](http://wiki.ros.org/ntrip_client).
Our [ublox2nmea](src/ublox2nmea.cc) node makes sure the caster receives the current VRS location.
It receives the [NavPVT](http://docs.ros.org/en/noetic/api/ublox_msgs/html/msg/NavPVT.html) from the u-blox receiver and outputs it as [NMEA $GPGGA sentence](http://docs.ros.org/en/api/nmea_msgs/html/msg/Sentence.html).
```
roslaunch ublox_utils ublox.launch device_position_receiver:=/dev/ttyACM0 use_ntrip:=true ntrip_username:=YOUR_USER ntrip_password:=YOUR_PASSWORD
```

The rosgraph should look like this:
![Rosgaph of NTRIP connection.](https://user-images.githubusercontent.com/11293852/169337693-09c338d6-1e9d-416b-b12d-9ae0bfa735db.png)

### Dual Receivers with NTRIP
```
roslaunch ublox_utils ublox.launch device_position_receiver:=/dev/ttyACM0 use_moving_baseline:=true device_moving_baseline_receiver:=/dev/ttyACM1 use_ntrip:=true ntrip_username:=YOUR_USER ntrip_password:=YOUR_PASSWORD
```
