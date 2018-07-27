Steps for setting up the repository       {#install_instruction}
=======================
These instructions are valid for a x64 Linux system with a Debian based environment.
\n First you need to setup the working environment and the repositories for editing and compiling the Firmware files.

Steps for compiling code and generating the hex files
------

- Install ARM GCC Compiler
	- You will have to install the following version of ARM GCC Compiler:[GNU Arm Embedded Toolchain: 6-2017-q2-update June 28, 2017] (https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2?revision=2cc92fb5-3e0e-402d-9197-bdfc8224d8a5?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,6-2017-q2-update)
	- Extract the above to  location "/usr/local".
- Install srecord using:

       sudo apt-get install srecord

 \n Srecord's "srec_cat" command is used to merge hex files before flashing to the target device.
- Clone the Github repository from https://github.com/Appiko/nrf5x-firmware
- Clone the GitLab repository to the same directory as "nrf5x-firmware", from https://gitlab.com/appiko/dfu_nrf_sdk15.git \n Its not available in the public repository as it does not have an open license.

Documentation with Doxygen
------
We use Doxygen for all our documentation. You can install Doxygen using: 

       sudo apt-get install doxygen

Using a Debugger/Programmer
------

This repository supports two debuggers/programmers, namely [Black Magic Probe (BMP)](https://github.com/blacksphere/blackmagic/wiki) and Segger JLink.

#### For utilizing BMP ####
For getting started with the Black Magic Probe follow this [guide](https://github.com/blacksphere/blackmagic/wiki/Getting-Started).
\n On some Linux installations you may get permissions errors. 
	
      /dev/ttyACM0: Permission denied

This can be fixed by adding your user to the dialout group as follows (you may need to log out and back in for this to take effect):

      $ sudo adduser <username> dialout

\n Then, as mentioned [here](https://github.com/blacksphere/blackmagic/wiki/Frequently-Asked-Questions#its-annoying-to-look-up-an-always-changing-device-name-on-linux) create a file named `/etc/udev/rules.d/99-blackmagic.rules` with the following contents:

      # Black Magic Probe
      # there are two connections, one for GDB and one for uart debugging
      SUBSYSTEM=="tty", ATTRS{interface}=="Black Magic GDB Server", SYMLINK+="ttyBmpGdb"
      SUBSYSTEM=="tty", ATTRS{interface}=="Black Magic UART Port", SYMLINK+="ttyBmpTarg"

Then unplug / replug the probe, or restart the computer.

#### For utilizing Segger JLink ####

- Download the SEGGER JLink software pack from [here](https://www.segger.com/downloads/jlink) 
\n The latest version at the time of writing for a x64 Debian based environment is V6.32i and can be found [here](https://www.segger.com/downloads/jlink/JLink_Linux_V632i_x86_64.deb)
- Get the nrfjprog to work with the JLink. Its not in the repository as it does not have an open license.
    - Download the nRF5x-Command-Line-Tools-Linux64 available at https://www.nordicsemi.com/eng/nordic/download_resource/51388/20/11220429
    - Extract the "nrfjprog" folder in the "nrf5x-firmware > utils" folder in the main repository that you have cloned. 

Using the proprietary nRF5x SDK
------
Get the components part of nRF5x SDK. Its not in the repository as it does not have an open license.
- Download the zip folder of the [nRF 15.0.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/) SDK and extract it.
- Copy the "components" folder of the SDK to the "nrf5x-firmare" repository and rename it as "SDK_components"
- Edit the Makefile of the application you want to compile to set the `INCLUDEDIRS`, `C_SRC_DIRS` and `C_SRC` to the appropriate files and folders in the SDK.

Installing nrfutil
------
nrfutil is a Python package and command-line utility that supports Device Firmware Updates (DFU) and cryptographic functionality.
\n Follow these [instructions](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrfutil%2Fnrfutil_installing.html&cp=5_5_1) to install this utility.


