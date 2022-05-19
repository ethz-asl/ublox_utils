# ublox_utils
Repository with ROS tools for the u-blox RTK receiver.

## Table of Contents
1. [Hardware Setup](#Hardware Setup)
2. [ROS Sensor Launch](#ROS Sensor Launch)
3. [Moving Baseline Firmware Setup](#Moving Baseline Setup)
4. [NTRIP Corrections](#NTRIP Corrections)

5. This includes:
- hardware and firmware setup instructions for single and moving baseline receivers
- ROS node [fix2nmea](./ublox_utils/launch/fix2nmea.launch) to create $GPGGA message from ublox status and active NTRIP communication
- ROS [launch file](./ublox_utils/launch/ublox.launch) to start single and moving baseline receiver

## Hardware Setup
This section describes the hardware setup with one or two receivers on the rover.
The following considerations should be taken into account:
- Optimal **sky coverage** of the antennas.
- Antennas **far distanced** from other electronics to avoid interference (especially LiDAR and USB-C devices).
- **Omnidirectional helical antennas** preferable over patch antennas as they do not require ground plane and have a better attitude coverage (see [ZED-FP9 Moving base applications, p.8](https://content.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_%28UBX-19009093%29.pdf)).. 
- **Multi-band antennas** that support all receiver frequencies with well-defined phase center and small phase center variation (see for example [HC882 Dual-Band Helical Antenna + L-Band](https://www.tallysman.com/app/uploads/2018/03/Tallysman%C2%AE-HC882-Datasheet_March-2022.pdf)). 
- **Known phase center position** with respect to rover body frame (see r_BP and r_BM in Figure below).
- Moving baseline antenna **at least 20cm** distanced from moving base antenna (see [ZED-FP9 Moving base applications, p.7](https://content.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_%28UBX-19009093%29.pdf)).

![Hardware GNSS setup.]()

### Electronics
The NTRIP setup does not require a base station.
Instead the corrections are provided by an NTRIP caster online.
The wiring is shown in the Figure below.
In case of a moving baseline setup correction from the moving base receiver to the moving baseline receiver will be sent via UART2.

![Electronic wiring of dual RTK setup.]()

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

## ROS Sensor Launch
## Moving Baseline Setup
## NTRIP Corrections
We use the [swipos-GIS/GEO](https://www.swisstopo.admin.ch/de/geodata/geoservices/swipos/swipos-dienste/swipos-gisgeo.html) caster in conjunction with the [ROS ntrip client](http://wiki.ros.org/ntrip_client).
The following sequence will start the connection.
```

```

# fix2nmea
This ROS node receives the [NavHPPOSLLH](http://docs.ros.org/en/noetic/api/ublox_msgs/html/msg/NavHPPOSLLH.html) from the ublox receiver and outputs it as [NMEA $GPGGA sentence](  ros::Subscriber navsatfix_sub_;
).
This sentence can be used in conjunction with [ROS ntrip client](http://wiki.ros.org/ntrip_client) to pipe the [RTCM correction messages](https://github.com/tilk/rtcm_msgs/blob/master/msg/Message.msg) to the [ublox ROS driver](https://github.com/KumarRobotics/ublox).
