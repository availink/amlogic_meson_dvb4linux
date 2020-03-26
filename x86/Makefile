TOP?=top

obj-m := avl68x2.o

avl68x2-y := \
dvb-avl/dvb-frontends-availink/availink/common/avl_bsp_linux.o \
dvb-avl/dvb-frontends-availink/availink/avl68x2/avl68x2.o \
dvb-avl/dvb-frontends-availink/availink/common/avl_lib.o \
dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src/AVL_Demod_DVBC.o \
dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src/AVL_Demod_CommonInternal.o \
dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src/AVL_Demod_ISDBT.o \
dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src/AVL_Demod_DVBSx.o \
dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src/AVL_Demod_DVBTx.o \
dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src/AVL_Demod.o


#src?=$(shell pwd)

vpath %.c $(src)/dvb-avl/dvb-frontends-availink/availink/common
vpath %.c $(src)/dvb-avl/dvb-frontends-availink/availink/avl68x2
vpath %.c $(src)/dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src

ccflags-y := -I$(src) -I$(src)/dvb-avl/dvb-frontends-availink/availink/common \
-I$(src)/dvb-avl/dvb-frontends-availink/availink/avl68x2 \
-I$(src)/dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src


KERNEL?=/lib/modules/$(shell uname -r)/build
#KERNEL?=/home/ubuntu/adam/kernels/linux-5.5
#KERNEL?=/home/ubuntu/adam/kernels/linux-4.19


all: dvb-avl
	echo $(src)
	make -C $(KERNEL) M=$(PWD) TOP=no modules

.PHONY: dvb-avl
dvb-avl:
ifeq ($(TOP),top)
	cp -ar $(PWD)/../dvb-avl .
endif

clean:
	rm -rf dvb-avl
	rm -f *.ko
	rm -f *.mod.*
	rm -f *.o
	rm -f modules.order
	rm -f Module.symvers

