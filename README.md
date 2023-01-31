# Marvelmind API using example #

### About  ###

This software implements examples of using all functions from Marvelmind API (library dashapi.dll in Windows, libdashapi.so in Linux). 
See chapter 10.5 of the interfaces documnet for more details: [https://marvelmind.com/pics/marvelmind_interfaces.pdf](https://marvelmind.com/pics/marvelmind_interfaces.pdf)

* Microsoft Windows
* GNU/Linux (including Raspberry Pi)

### Building the example ###

To build the example on GNU/Linux or another *nix-OS you need to have installed GCC. Then unpack the archive, change directory to unpacked library and run make in console. Then you can execute ./mmm_api_example to start the example. 

If you want to build a project for MS Windows, you may use integrated development environment (such a MS Visual Studio, Code::Blocks etc.): create empty console project and add source files (all *.c and *.h files) into the project and run build. You may need to change the project settings to successfully build it.
<br />

Prebuilt executable API example for Windows is included into the software package:
[https://marvelmind.com/pics/marvelmind_SW.zip](https://marvelmind.com/pics/marvelmind_SW.zip)


### Command line options of the example ###

 ./mm_api_example   - starts the the example and trying to connect to any Marvelmind device via any virtual COM port over USB. <br />
 ./mm_api_example udp 192.168.1.100 49300    - starts the the example and trying to connect via UDP to the Super-Modem on IP address 192.168.1.100 with UDP port 49300

or for Microsoft Windows:

 mm_api_example.exe <br />
 mm_api_example.exe udp 192.168.1.100 49300
 



