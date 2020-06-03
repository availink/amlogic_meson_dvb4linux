## amlogic_meson_dvb4linux
This repository is our prototype for distributing Linux DVB drivers for our demodulator products.  Here, we have integrated our AVL62x1 S/S2/S2X demodulator driver and the Airoha AV201x tuner driver and our AVL68x2 multistandard demod driver and the R848 tuner driver into a DVB frontend devices for the Amlogic Meson SoC.  We package the drivers into the CoreELEC distro as a proof of concept.

### Directory Structure
```
\--dvb-avl/ : build area for CoreELEC
|       |
|       \--aml/ : Amlogic Meson SoC DVB driver source
|       |
|       \--aml_fe_avl62x1_av201x.* : DVB frontend for Amlogic Meson SoC using AVL62x1 and AV201x
|       |
|       \--aml_fe_avl68x2_r848.* : DVB frontend for Amlogic Meson SoC using AVL68x2 and R848
|       |
|       \--dvb-frontends-availink : git submodule containing demod drivers
|
\--x86 : build area for x86
```

### How to use with CoreELEC
The following will build CoreELEC with our drivers built into the "CrazyCat" driver set
<pre>
cd
git clone https://github.com/availink/CoreELEC_avl.git CoreELEC
cd CoreELEC
git checkout HACKING_9.2.1
cd
git clone --recurse-submodules https://github.com/availink/amlogic_meson_dvb4linux.git
cd CoreELEC
PROJECT=Amlogic DEVICES=S905 ARCH=arm make image
</pre>
* CoreELEC image
  * The following relevant kernel modules will be built:
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/avl_bsp.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/avl62x1.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/avl68x2.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/aml_fe_avl62x1_av201x.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/aml_fe_avl68x2_r848a.ko
  * The following demod firmware blobs will be build:
    * /lib/firmware/availink/dvb-fe-avl62x1.fw
    * /lib/firmware/availink/dvb-fe-avl68x2-dvbc.fw
    * /lib/firmware/availink/dvb-fe-avl68x2-dvbsx.fw
    * /lib/firmware/availink/dvb-fe-avl68x2-dvbtx.fw
    * /lib/firmware/availink/dvb-fe-avl68x2-isdbt.fw
### Notes

