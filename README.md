# RespirHo' Project

This repository contains all the necessary files to program the firmware of RespirHo' boards and acquire the respiratory signal.

Repo structure:
* Folder "ANT_PROVA-SDK_PC.3.5_CONSALVATAGGIO" contains the old ANT interface.
* Folder "ANT_STE" contains the renewed ANT interface with new functions like sensors calibration, acquisition stop and resume.
* Folder "File HEX" contains the binary files to upload on boards. Upload "Device_x.hex" (with x: 1,2,3) on PCB version, "Device_x_(millefori).hex" (with x: 1,2,3) for prototype.
* Folder "nRF5_SDK_14.2.0_17b948a" contains the software development kit for the nRF52832 microcontroller. 
* Folder "OTA_Updater_v1.4" contains an app that allows to upload new firmware to boards over the air (OTA).

Please refer to the [developer guide](https://drive.google.com/file/d/1UFlXrOAoPOUmqghLcS6q3u2aQWWckizt/view?usp=sharing) for further information about usage.
