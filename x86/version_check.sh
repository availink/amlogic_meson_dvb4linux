#!/bin/bash

let fw_min_hex="0x`dd if=../dvb-avl/dvb-frontends-availink/firmware/availink/dvb-fe-avl62x1.fw bs=1 count=1 skip=25 status=none | xxd | awk '{print $2}'`"
let drv_min_dec=`grep "define AVL62X1_VER_MINOR" ../dvb-avl/dvb-frontends-availink/availink/avl62x1/sdk_src/avl62x1_lib.h | awk '{print $3}'`

let drv_min_hex=$(printf "0x%x" $drv_min_dec)

if [ "$fw_min_hex" != "$drv_min_hex" ]; then
	printf "ERROR: dvb-fe-avl62x1.fw mismatching minor versions. FW: %d, Driver: %d\n" $fw_min_hex $drv_min_hex
	exit 1
else
	printf "dvb-fe-avl62x1.fw Minor versions match. FW: %d, Driver: %d\n" $fw_min_hex $drv_min_hex
fi


fws=("dvb-fe-avl68x2-dvbc.fw" "dvb-fe-avl68x2-dvbsx.fw" "dvb-fe-avl68x2-dvbtx.fw" "dvb-fe-avl68x2-isdbt.fw")

for fw in ${fws[@]}; do
	let fw_min_hex="0x`dd if=../dvb-avl/dvb-frontends-availink/firmware/availink/$fw bs=1 count=1 skip=25 status=none | xxd | awk '{print $2}'`"
	let drv_min_dec=`grep "define AVL68X2_VER_MINOR" ../dvb-avl/dvb-frontends-availink/availink/avl68x2/sdk_src/avl68x2_internal.h | awk '{print $3}'`
	let drv_min_hex=$(printf "0x%x" $drv_min_dec)

	if [ "$fw_min_hex" != "$drv_min_hex" ]; then
		printf "ERROR: $fw mismatching minor versions. FW: %d, Driver: %d\n" $fw_min_hex $drv_min_hex
		exit 1
	else
		printf "$fw Minor versions match. FW: %d, Driver: %d\n" $fw_min_hex $drv_min_hex
	fi
done
