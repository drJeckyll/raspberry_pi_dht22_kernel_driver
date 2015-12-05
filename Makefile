CCPREFIX=/mnt/storage/pi/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
KERNEL_SRC=/mnt/storage/pi/linux-3.10.24

obj-m += dht11km.o

all:
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C $(KERNEL_SRC) M=$(PWD) modules 

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean
