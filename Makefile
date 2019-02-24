obj-m := toucan.o

kernel_ver := $(shell uname -r)

all:
	make -C /lib/modules/$(kernel_ver)/build M=$(shell pwd) modules

install:
	make -C /lib/modules/$(kernel_ver)/build M=$(shell pwd) modules_install

clean:
	make -C /lib/modules/$(kernel_ver)/build M=$(shell pwd) clean

