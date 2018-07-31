Steps for Updating the Firmware      {#firmware_update}
=======================
For updating the Firmware of Sense_Pi unit one must have the [repository setup](install_instruction.html) on their computer.
\n Then follow these steps:
- Go to the folder "nrf5x-firmware > applications > sense_pir " and open the terminal there.
- Use command 'make release' to create the .zip folder containing all the .hex files for the firmware update. 
  \n It will also create a .hex file which can be used for directly flashing the firmware to the Sense_pi unit using a Pogo Connector.
- Transfer this .zip file to your mobile device.
- Download the "nRF Connect for Mobile" App (by Nordic Semiconductor ASA) for flashing the firmware to the Sense_Pi unit.


Steps for using nRF Connect App
------

- You will have to go to the Bootloader mode. Open the Sense_Pi unit and disconnect the power connector.
  \n Now keep the switch pressed and connect the power connector. You will see the LED blinking in a Green-Red pattern. 
  \n You are in Bootloader mode now.
- Open the app. Keep your phone's bluetooth On.
- Use "Scan" option in the app and connect to the SensiPi unit shown as "SensePi Update". 
- You will see a small, circular "DFU" icon on the top right corner. Press that.
- Now select the "Distribution packet (ZIP)" option.
- Browse to the location where you have the "sense_pir_xxx_yyy.zip" file. Select the file and wait till the firmware is updated.
- The Sense_Pi unit will now go to "Sensing Mode" after this. \n You can put back the cover and use it now.

