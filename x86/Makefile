TOP?=top

obj-m := avl68x2.o

avl68x2-y := \
dvb-avl/lib/avl_bsp_linux.o \
dvb-avl/lib/avl_lib.o \
dvb-avl/demod/avl68x2/sdk_src/AVL_Demod_DVBSx.o \
dvb-avl/demod/avl68x2/sdk_src/AVL_Demod_Config.o \
dvb-avl/demod/avl68x2/sdk_src/AVL_Demod_DVBTx.o \
dvb-avl/demod/avl68x2/sdk_src/AVL_Demod_CommonInternal.o \
dvb-avl/demod/avl68x2/sdk_src/AVL_Demod.o \
dvb-avl/demod/avl68x2/sdk_src/AVL_Demod_ISDBT.o \
dvb-avl/demod/avl68x2/sdk_src/AVL_Demod_DVBC.o

vpath %.c $(src)/dvb-avl/lib
vpath %.c $(src)/dvb-avl/demod/avl68x2
vpath %.c $(src)/dvb-avl/demod/avl68x2/sdk_src

ccflags-y := -I$(src) -I$(src)/dvb-avl/lib \
-I$(src)/dvb-avl/demod/avl68x2 \
-I$(src)/dvb-avl/demod/avl68x2/sdk_src

#KERNEL?=/lib/modules/$(shell uname -r)/build
#KERNEL?=/home/ubuntu/adam/kernels/linux-5.5
KERNEL?=/home/ubuntu/adam/kernels/linux-4.19


all: dvb-avl
	make -C $(KERNEL) M=$(PWD) TOP=no modules

dvb-avl:
ifeq ($(TOP),top)
	cp -r $(PWD)/../dvb-avl .
endif

clean:
	rm -rf dvb-avl
	rm -f *.ko
	rm -f *.mod.*
	rm -f *.o
	rm -f modules.order
	rm -f Module.symvers

