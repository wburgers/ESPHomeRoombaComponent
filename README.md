# Roomba Component for ESPHome

The inspiration is [Mannkind ESPHomeRoombaComponent](https://github.com/mannkind/ESPHomeRoombaComponent), that became deprecated due to dependency updates.
In addition I did not want the Roomba device to communicate over MQTT, it instead registers a service call for commands, and expose sensors through `roomba.yaml`.

## Hardware

## Wiring Guide
Using the brc pin we get the following:
![brc wiring](https://github.com/Ceiku/ESPHomeRoombaComponent/blob/master/esp_roomba_brc.PNG)

Alternatively using the IR led:
![IR wiring](https://github.com/Ceiku/ESPHomeRoombaComponent/blob/master/esp_roomba_IR.PNG)

## Special Notes

*Depending on your Roomba model, you might be unlucky as me and have a bug that does not allow the device to wake from brc pin when it goes into passive mode during trickle charging. To circumvent this I copied my code from a previous general controller I had stationed in the kitchen for 433mhz and IR controlling that sends the start code with and IR led to the roomba, it need a clear sight of the roomba IR sensor over a short distance.

If you need this, uncomment the remote transmitter switch section in the `roomba.yaml` and use the switch to wake and clean. 

## Placement

The Wemos D1 mini is small enough to [fit into the compartment by one of the wheels](https://community-home-assistant-assets.s3.dualstack.us-west-2.amazonaws.com/optimized/2X/a/a258c7253f8bd3fe76ad9e7aa1202b60bd113d74_2_496x600.jpg). 
But using a esp-01 or esp-12 series and some single braid wire I have managed to fit it all under the top lid without bulging or deformeties, the only visual defect is my led that is pointing at the IR reciever.


## Software Setup/Use

Copy the contents of the `ESPRoombaHomeComponent.h` and `example/roomba.yaml` into your esphome config folder, change accordingly.

Flash onto new device and connect circuit.


## Motivation

Besides the obvious, connecting your vacuum to your automation system, it also does so completly inside a private and controlled ecosystem. As I was about to replace this unit due to the "hacky feel" of using the IR led, as the unit was shipped bugged and there's not a high availability of create cables for a patch, but all other solutions had me locked into a proprietary system such as mi home, or tuya with smart life; sure there are roundabout ways of both interfacing with them through Home assistant etc, and do so locally. They all seemingly lacked the future proofing and control I wanted. With this setup there is nothing really stopping you from a full on LiDar setup and room control!
