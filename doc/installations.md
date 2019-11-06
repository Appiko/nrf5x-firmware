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

 \n Srecord's `srec_cat` command is used to merge hex files before flashing to the target device.
- Clone the Github repository from https://github.com/Appiko/nrf5x-firmware
- Clone the GitLab repository to the same directory as "nrf5x-firmware", from https://gitlab.com/appiko/dfu_nrf_sdk15.git \n Its not available in the public repository as it does not have an open license.

Documentation with Doxygen
------
We use Doxygen for all our documentation. You can install Doxygen from your package manager or manually from from the official website.

http://www.doxygen.nl/download.html


To generate documentation Navigate to `nrf5x-firmware/doc` and run 

`$ doxygen Appiko.doxyfile`

The `html` documentation is stored under `./build/html/`
You can start a python server locally to see the genrated documentation

`$  cd build/html; python -m http.server 5000`

Navigate to `localhost:5000` on your browser to see the generated documentation.

Using a Debugger/Programmer
------

This repository supports two debuggers/programmers, namely [Black Magic Probe (BMP)](https://github.com/blacksphere/blackmagic/wiki) and Segger JLink.

#### BMP ####
For getting started with the Black Magic Probe follow this [guide](https://github.com/blacksphere/blackmagic/wiki/Getting-Started).
\n On some Linux installations you may get permissions errors. 
	
      /dev/ttyACM0: Permission denied

This can be fixed by adding your user to the accessible group as follows (you may need to log out and back in for this to take effect):


    $ stat /dev/ttyACM0 | grep Gid
    Access: (0660/crw-rw----)  Uid: (    0/    root)   Gid: (  987/    dialout)



   Add user to the group, in our case `dailout`



    $ sudo adduser $USER dialout
    
    
\n Then, as mentioned [here](https://github.com/blacksphere/blackmagic/wiki/Frequently-Asked-Questions#its-annoying-to-look-up-an-always-changing-device-name-on-linux) create a file named `/etc/udev/rules.d/99-blackmagic.rules` with the following contents:

      # Black Magic Probe
      # there are two connections, one for GDB and one for uart debugging
      SUBSYSTEM=="tty", ATTRS{interface}=="Black Magic GDB Server", SYMLINK+="ttyBmpGdb"
      SUBSYSTEM=="tty", ATTRS{interface}=="Black Magic UART Port", SYMLINK+="ttyBmpTarg"

Then unplug / replug the probe, or restart the computer.

#### Segger JLink ####

- Download the SEGGER JLink software pack from [here](https://www.segger.com/downloads/jlink) 
\n The latest version at the time of writing for a x64 Debian based environment is V6.32i and can be found [here](https://www.segger.com/downloads/jlink/JLink_Linux_V632i_x86_64.deb)
- Get the nrfjprog to work with the JLink. Its not in the repository as it does not have an open license.
    - Download the nRF5x-Command-Line-Tools-Linux64 available at https://www.nordicsemi.com/eng/nordic/download_resource/51388/20/11220429
    - Extract the "nrfjprog" folder in the "nrf5x-firmware > utils" folder in the main repository that you have cloned. 

Using the proprietary nRF5x SDK
------
Get the components part of nRF5x SDK. Its not in the repository as it does not have an open license.
- Download the [nRF5_SDK_15.0.0_a53641a.zip](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.0.0_a53641a.zip) and extract it.
- Copy the "components" folder of the SDK to the "nrf5x-firmare" repository and rename it as "SDK_components"
- You might have to edit the Makefile of the application you want to compile, to set the `INCLUDEDIRS`, `C_SRC_DIRS` and `C_SRC` to the appropriate files and folders in the SDK.

Installing nrfutil
------
nrfutil is a Python package and command-line utility that supports Device Firmware Updates (DFU) and cryptographic functionality.

Follow these [instructions](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fug_nrfutil%2FUG%2Fnrfutil%2Fnrfutil_installing.html) to install this utility.

Setting up IDE
------
IDE(Integrated Development Environment) is useful for fast development of project. User can setup any IDE for project development.
\n User can also choose not to use any IDE at all. Two major IDEs which are used widely are:

- NetBeans IDE
- Eclipse IDE

\n It is advised to use the versions which comes with C/C++ plugins by default.
\n Follow the instructions given in following link to setup your IDE

- NetBeans IDE : 

    + [NetBeans Download](https://netbeans.org/downloads/) Select the one for C/C++.

    + [ARM ToolChain installation in NetBeans](http://www.dalbert.net/?p=239)

    + To get all the intellisense working perfectly without creating a mess, Download the [.netbeans.zip](https://github.com/Appiko/nrf5x-firmware/raw/development/doc/.netbeans.zip) file and import it using the import option under `tools > options`

    + Create a new Netbeans project and make sure the tool collection is `ARM(GNU)`, and select configuration mode is set to `Automatic`

    + On creating a successful project with intellisense, Netbeans would give a pop up saying *Code assistance successful*

- Eclipse
    + [Eclipse Download](https://www.eclipse.org/cdt/)
    + [ARM ToolChain installation in Eclipse](https://gnu-mcu-eclipse.github.io/toolchain/arm/install/)


> NOTE : Use IDE only for development of application. While compiling and uploading of program use "make " utilities in terminal.

