SocketCAN driver for Toucan adapter
====

Building kernel module
----

Install build tools:

    sudo apt install build-essential

If you use with Raspberry Pi install Linux Kernel Headers:

    sudo apt install raspberrypi-kernel-headers
    
Download TouCAN SocketCAN driver source code from github:    

    git clone --branch v1.0 https://github.com/rusoku/TouCAN-SocketCAN

Make sure you have kernel source and C toolchain packages installed.
To compile run:

    sudo make

To install module to system run:

    sudo make install


Create a list of module dependencies.
  
    sudo depmod

Reboot your Raspberry Pi. Double check if TouCAN driver module is loaded
after booting.

    pi@raspberrypi:~ $ lsmod | grep toucan
    toucan 20480 0
    can_dev 28672 1 toucan
    

Set up interface
----
    ip link set can0 up type can bitrate 125000


Shut down interface
----

    ip link set can0 down
    rmmod toucan



