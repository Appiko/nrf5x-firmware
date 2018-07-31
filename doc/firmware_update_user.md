Steps for Updating the Firmware (for anyone else)      {#firmware_update_user}
=======================
For updating the Firmware of a SensePi unit you must have:
- The updated Firmware file (.zip file).
- The "nRF Connect for Mobile" App (by Nordic Semiconductor ASA). You can download it from here [Android](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en_IN) or [iOS](https://itunes.apple.com/in/app/nrf-connect/id1054362403?mt=8).

Steps for using nRF Connect App
------

- Transfer the .zip file provided to your mobile device.
- You will have to go to the Bootloader mode. Open the SensePi unit and disconnect the power connector.
  \n Now keep the switch pressed and connect the power connector. You will see the LED blinking in a Green-Red pattern. 
  \n You are in Bootloader mode now.
- Open the app. Keep your phone's bluetooth On.
- Use "Scan" option in the app and connect to the SensiPi unit shown as "SensePi Update". 
- You will see a small, circular "DFU" icon on the top right corner. Press that.
- Now select the "Distribution packet (ZIP)" option.
- Browse to the location where you have the "SensePir_xxx_yyy.zip" file. Select the file and wait till the firmware is updated.
- The SensePi unit will now go to "Sensing Mode" after this. \n You can put back the cover and use it now.
