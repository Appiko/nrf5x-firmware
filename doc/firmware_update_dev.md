Steps for Modifying and Updating the Firmware (for Developers)      {#firmware_update_dev}
=======================
For modifying and updating the Firmware of a SensePi unit one must have the [repository setup](install_instruction.html) on their computer.
\n Then follow these steps:
- Go to the folder "nrf5x-firmware > application" and make changes to the files you want to modify.
- Use command 'make release' in that directory, for example, if you made changes in the SensePir, use 
    
      user:~/../nrf5x-firmware/application/SensePir$ make release
  
  to create the .zip folder containing all the .hex files for the firmware update of SensePir. 
  \n It will also create a .hex file which can be used for directly flashing the firmware to the SensePi unit using a Pogo Connector, or to any other board using it's respective connector.
- You will find these files in the "nrf5x-firmware > release" folder.
- Transfer this .zip file to your mobile device.


Steps for using nRF Connect App
------

- Download the "nRF Connect for Mobile" App (by Nordic Semiconductor ASA) ([Android](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en_IN)|[iOS](https://itunes.apple.com/in/app/nrf-connect/id1054362403?mt=8)) for flashing the firmware.
- You will have to go to the Bootloader mode. If you are using the SensePi unit, open it up and disconnect the power connector.
- Now keep the switch pressed and connect the power connector. You will see the LED blinking in a Green-Red pattern. 
  \n You are in Bootloader mode now.
- Open the app. Keep your phone's bluetooth On.
- Use "Scan" option in the app and connect to your device.
- You will see a small, circular "DFU" icon on the top right corner. Press that.
- Now select the "Distribution packet (ZIP)" option.
- Browse to the location where you have the .zip file. Select the file and wait till the firmware is updated.
- And that's it, you have updated your device's firmware.
- If you are using a SensePi unit, it will now go to "Sensing Mode" after this. \n You can put back the cover and use it now.
