CAN driver for Toucan adapter
====

Building kernel module
----

Clone source code:

git clone --branch v1.0 https://github.com/rusoku/TouCAN-SocketCAN

Make sure you have kernel source and C toolchain packages installed.
To compile run:

    make

To install module to system run:

    sudo make install


Set up interface
----

    insmod toucan.ko
    ip link set can0 up type can bitrate 125000


Shut down interface
----

    ip link set can0 down
    rmmod toucan



