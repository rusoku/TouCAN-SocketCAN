obj-m := toucan.o

kernel_ver := $(shell uname -r)

#KDIR ?= $(HOME)/work/WSL2-Linux-Kernel
KDIR ?= /lib/modules/$(kernel_ver)/build

all:
#	$(MAKE) -j$(JOBS) -C $(KDIR) M=$(CURDIR) C=2 W=1 CHECK=sparse KCFLAGS="-Werror" WERROR=1 modules
	$(MAKE) -j$(JOBS) -C $(KDIR) M=$(CURDIR) C=2 WERROR=1 modules

install:
	$(MAKE) -j$(JOBS) -C $(KDIR) M=$(CURDIR) modules_install

clean:
	$(MAKE) -j$(JOBS) -C $(KDIR) M=$(CURDIR) clean

