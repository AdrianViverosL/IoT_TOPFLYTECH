# Embedded System: TTGO-800 ESP32 with TOPFLYTech's TSTH1-B  & TSR1 devices. Internet of Things.

This project implements a system based on the Internet of Things with the purpose of monitoring conditions of an area/space, such as temperature, humidity, and scene. 

The architecture of this system is based on the ESP32 chip from Espressif Systems, which is popular hardware used for this kind of application. 

As the project was proposed to operate in a truck on the way, the board selected to develop the application is TTGO SIM800 from LILYGO company. This board has the unique feature to has embedded a SIM slot for 2G networks. However, at the moment only the WiFi connection is working. 

Another important piece of this system is the sensor device that connects to the board using BLE technology on Beacon mode. This device is a temperature, humidity sensor, called TSTH1-B from TOPFLYtech company. 

All the data is uploaded to a Blynk service, which is a cloud platform for IoT projects. It provides an API token to establish a connection between the service and the project. 

This project was developed using PlatformIO environment for VSCode, which has a lot of features to make development easier.

