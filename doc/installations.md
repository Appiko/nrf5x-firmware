Steps for setting up the repository       {#install_instruction}
=======================
These instructions are valid for a x64 Linux based environment.

- Install ARM GCC compiler https://launchpad.net/~team-gcc-arm-embedded/+archive/ubuntu/ppa
- Install srecord by `sudo apt-get install srecord`. Srecord's srec_cat is used to merge hex files before flashing to the target device.
- Clone the repository from https://github.com/Appiko/nrf5x-firmware
- Get the components part of [nRF5x SDK](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v12.x.x/nRF5_SDK_12.1.0_0d23e2a.zip). Its not in the repository as it does not have an open license. 
    - Download the zip of the nRF 12.1.0 SDK and extract it.
    - Copy the components folder of the SDK to the nrf5x-firmare repository.
    - Rename the folder from components to SDK_components. 
- Download the SEGGER JLink software pack from https://www.segger.com/downloads/jlink The latest version at the time of writing for a x64 Debian based environment is 6.12i and can be found at https://www.segger.com/downloads/jlink/JLink_Linux_V612i_x86_64.deb  
- Get the nrfjprog work with the JLink. Its not in the repository as it does not have an open license.
    - Download the nRF5x-Command-Line-Tools-Linux64 available at https://www.nordicsemi.com/eng/nordic/download_resource/51388/20/11220429
    - Extract the nrfjprog folder in the utils folder 
- Perform the changes mentioned in [SDK Changelog page](sdk_changelog.html)
- Install Doxygen for documentation by `sudo apt-get install doxygen`
