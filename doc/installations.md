Steps for setting up the repository       {#install_instruction}
=======================
These instructions are valid for a Linux based environment.

- Install ARM GCC compiler https://launchpad.net/~team-gcc-arm-embedded/+archive/ubuntu/ppa
- Install srecord by `sudo apt-get install srecord`. Srecord's srec_cat is used to merge hex files before flashing to the target device.
- Clone the repository from https://github.com/Appiko/nrf52-firmware
- Download the nRF 12.1.0 SDK and extract the contents of the components directory to SDK_components directory. The SDK is available at http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v12.x.x/nRF5_SDK_12.1.0_0d23e2a.zip
- Download and extract the nrfjprog in utils directory from nRF5x-Command-Line-Tools-Linux64 available at https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832#Downloads
- Perform the changes mentioned in [SDK Changelog page](SDK_changelog.md)
- Install Doxygen for documentation by `sudo apt-get install doxygen`
