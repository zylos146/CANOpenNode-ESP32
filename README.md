## Code Loaded
This is an updated [CANOpenNode](https://github.com/CANopenNode/CANopenNode) implementation for ESP32 using Latest master code from Oct 13, 2021

This was built by copying over all relevant h/c files from 30x / extra folders, then adding a driver heavily based on Alexander's Implementation. It can be updated in the future with a similar method.

[Exact commit loaded](https://github.com/CANopenNode/CANopenNode/commit/8c7d852902b2d307e8b91a43332c14e366641e00)

This is meant to be used with [CANFuck](https://github.com/zylos146/CANFuck) which has examples of the Object Dictionary and usage/initialization

## Editor
CANOpenEditor v4.0-51-g2d9b1ad was used to build Object Dictionaries for this project

Although some manual editing afterwards was needed to fixup some missing `.attribute = ` entries, 
where the default value was not set to 0 due to a missing ReadOnly flag on RPDO OD Parameters.

This can be fixed either by 
- In OD.C, manually replacing `.attribute = ` with `.attribute = ODA_SDO_R,`
- Editing the device.xdd and removing all instances of ` access="noAccess"`, then re-generating the OD.h/c files

Seems to be some bug around how no SDO access is generated. Results in a bad OD.h/c file

## Based on
Based on work from 
- [nathanRamaNoodles - ESP32 Repo](https://github.com/nathanRamaNoodles/CANopen-ESP32-nodes)
- [Alexander Miller's implementation] https://github.com/xXAM22Xx/CANopenESP32/blob/master/ESP32_CANopen/lib/CANopen/CO_driver.c

