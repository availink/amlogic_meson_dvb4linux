## amlogic_meson_dvb4linux
This repository is our prototype for distributing Linux DVB drivers for our demodulator products.  Here, we have integrated our AVL6261 S/S2/S2X demodulator driver and our version of the Airoha AV201x tuner driver into a DVB frontend device for the Amlogic Meson SoC.  We package the drivers into the CoreELEC distro as a proof of concept.

### Directory Structure
```
dvb-avl/
|
\------aml/ : Amlogic Meson SoC DVB driver source
|
\------aml_fe_avl6261_av201x.* : DVB frontend for Amlogic Meson SoC using AVL6261 and AV201x
|
\------avl_bsp.* : Board support layer for AVL drivers (implements I2C, wait, and semaphores for AVL SDK)
|
\------demod/
|      |
|      \----avl6261/ : Linux DVB driver source for AVL6261 demod; written atop AVL SDK
|      |    |
|      |    \------sdk_src/ : AVL SDK for avl6261 driver;  written atop AVL BSP
|      \----avlXXXX/ : future demod driver source
|
|------tuner/
|      |
|      \----AV201X/ : Availink version of Airoha AV201x tuner driver; written atop AVL BSP
|      |
|      \----tunerABC/ : future tuner driver source
|
avl6261.patch : firmware blob for AVL6261 demodulator
```

### How to use with CoreELEC
For now, these are a hackfest.
* replace the dvb-avl directory in build.<target>/media_tree_cc_aml-<hash>/drivers/media/platform/meson/ with the one from this repository.
* patch the packages/linux-driver-addons/dvb/crazycat_aml/package.mk file as follows:
<pre>
diff --git a/packages/linux-driver-addons/dvb/crazycat_aml/package.mk b/packages/linux-driver-addons/dvb/crazycat_aml/package.mk
index c8c2755f09..63bdb22ff0 100644
--- a/packages/linux-driver-addons/dvb/crazycat_aml/package.mk
+++ b/packages/linux-driver-addons/dvb/crazycat_aml/package.mk
@@ -34,15 +34,11 @@ pre_make_target() {
 
 make_target() {
   cp -RP $(get_build_dir media_tree_cc_aml)/* $PKG_BUILD/linux
-  rm  -rf $PKG_BUILD/linux/drivers/media/platform/meson/wetek
-  rm  -rf $PKG_BUILD/linux/drivers/media/platform/meson/dvb-avl
-  cp -Lr $(get_build_dir media_tree_aml)/* $PKG_BUILD/linux
 
   # compile modules
   echo "obj-y += video_dev/" >> "$PKG_BUILD/linux/drivers/media/platform/meson/Makefile"
-  echo "obj-y += dvb/" >> "$PKG_BUILD/linux/drivers/media/platform/meson/Makefile"
-  echo 'source "drivers/media/platform/meson/dvb/Kconfig"' >>  "$PKG_BUILD/linux/drivers/media/platform/Kconfig"
-  sed -e 's/ && RC_CORE//g' -i $PKG_BUILD/linux/drivers/media/usb/dvb-usb/Kconfig
+  echo "obj-y += wetek/" >> "$PKG_BUILD/linux/drivers/media/platform/meson/Makefile"
+  echo "obj-y += dvb-avl/" >> "$PKG_BUILD/linux/drivers/media/platform/meson/Makefile"
 
   # make config all
   kernel_make VER=$KERNEL_VER SRCDIR=$(kernel_path) allyesconfig
</pre>
* patch the packages/linux-firmware/dvb-firmware/package.mk file as follows
<pre>
diff --git a/packages/linux-firmware/dvb-firmware/package.mk b/packages/linux-firmware/dvb-firmware/package.mk
index da4447d180..77f1d2e5d4 100644
--- a/packages/linux-firmware/dvb-firmware/package.mk
+++ b/packages/linux-firmware/dvb-firmware/package.mk
@@ -17,4 +17,5 @@ makeinstall_target() {
   PKG_FW_DIR="$INSTALL/$(get_kernel_overlay_dir)/lib/firmware"
   mkdir -p "$PKG_FW_DIR"
   cp -a "$PKG_BUILD/firmware/"* "$PKG_FW_DIR"
+  cp -a ${amlogic_meson_dvb4linux}/avl6261.patch "$PKG_FW_DIR"
 }
</pre>
* build CoreELEC image
  * The following relevant kernel modules will be built:
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/avl_bsp.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/avl6261.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/av201x_avl.ko
    * /lib/modules/3.14.29/updates/driver.dvb.crazycat_aml/aml_fe_avl6261_av201x
  * The demod firmware blob will exist in /lib/firmware/avl6261.patch
### Notes
* You may use any other tuner in the DVB tree.  Simply modify aml_fe_avl6261_av201x.c.
  * Availink will be releasing customized versions of tuner drivers that have been optimized and tested extensively for use with our products.
