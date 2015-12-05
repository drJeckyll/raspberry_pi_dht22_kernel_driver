raspberry_pi_dht22_kernel_driver
================================

Raspberry PI DHT22 kernel driver

Comment first two lines in Makefile if you build this on Pi itself. 
They are only for cross compilation. Also adjust paths accoring to your environment

How to crosscompile:

git clone https://github.com/raspberrypi/linux.git
git clone https://github.com/raspberrypi/tools.git

Set the environment variable CCPREFIX:
export CCPREFIX=/mnt/storage/pi/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-

Set the environment variable KERNEL_SRC:
export KERNEL_SRC=/mnt/storage/pi/linux

In KERNEL_SRC: execute "make mrproper" to ensure you have a clean kernel source tree.

Pull /proc/config.gz from your running Raspberry Pi and extract it into KERNEL_SRC on your Ubuntu machine.

Prime kernel with the old config:
make ARCH=arm CROSS_COMPILE=${CCPREFIX} oldconfig

Build the kernel:
make ARCH=arm CROSS_COMPILE=${CCPREFIX}
