## amlogic_meson_dvb4linux
This repository is our prototype for distributing Linux DVB drivers for our demodulator products.  Here, we have integrated our AVL62x1 S/S2/S2X demodulator driver and the Airoha AV201x tuner driver into a DVB frontend device for the Amlogic Meson SoC.  We package the drivers into the CoreELEC distro as a proof of concept.

### Directory Structure
```
\--dvb-avl/
|       |
|       \--aml/ : Amlogic Meson SoC DVB driver source
|       |
|       \--aml_fe_avl6261_av201x.* : DVB frontend for Amlogic Meson SoC using AVL6261 and AV201x
|       |
|       \--dvb-frontends-availink : git submodule containing demod drivers
|
\--linux-firmware-availink : git submodule containing demod firmware
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
cd amlogic_meson_dvb4linux
git checkout development
cd
cd CoreELEC
PROJECT=Amlogic DEVICES=S905 ARCH=arm make image
</pre>
* CoreELEC image
  * The following relevant kernel modules will be built:
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/avl_bsp.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/avl62x1.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/aml_fe_avl6261_av201x.ko
  * The demod firmware blob will exist in /lib/firmware/availink/avl62x1.patch
### Notes

