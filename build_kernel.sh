# Android kernel 3.4.5 Build Script for M065 board
# You make sure that the PATH of CROSS COMPILER.
# Android sdk prebuilt cross compiler 4.6 should be used: http://developer.android.com/sdk/index.html

PHONE_SERIES=mx2
BOARD_REVISON=
COMPILE_LOG=./compile.log
CROSS_COMPILE_PREFIX=arm-eabi-
INTERACTIVE=0

CPU_JOB_NUM=$(grep processor /proc/cpuinfo | awk '{field=$NF};END{print field+1}')

if [ $# -lt 1 ]
then
		echo "========================================"
		echo "Usage: ./build_kernel.sh <CONFIG_TYPE> <CONFIG_METHOD>"
		echo "ex) ./build_kernel.sh eng|user|dev|test|overseas|unicom|recovery|pm|board|old|menu def|old|menu"
		echo "========================================"
		exit 0
fi

CONFIG_TYPE=$1
DEFCONFIG=1
CONFIG_METHOD="def"
BOARD=0
PM=0
if [[ $CONFIG_TYPE == "board" ]]; then
	CONFIG_TYPE=eng
	BOARD=1
elif [[ $CONFIG_TYPE == "pm" ]]; then
	CONFIG_TYPE=user
	PM=1
	BOARD=1
elif [[ $CONFIG_TYPE == "old" || $CONFIG_TYPE == "menu" ]]; then
	DEFCONFIG=0
	CONFIG_METHOD="$CONFIG_TYPE"
fi

if [ -n "$2" ]; then
	CONFIG_METHOD=$2
	if [[ "$CONFIG_METHOD" != "menu" && "$CONFIG_METHOD" != "def" && "$CONFIG_METHOD" != "old" ]]; then
		echo "ERROR: Configure methods must be def, old or menu." && exit 5
	fi
	if [[ "$CONFIG_METHOD" == "menu" ]]; then
		INTERACTIVE=1
	fi
fi

echo
echo "[[[[[[[ Build android kernel 3.4.5 for M065 $CONFIG_TYPE ]]]]]]]"
echo

START_TIME=`date +%s`

[ -z "$CROSS_COMPILE" ] && export CROSS_COMPILE=$CROSS_COMPILE_PREFIX
which ${CROSS_COMPILE}gcc
[ $? -ne 0 ] && echo "ERROR: Cross compiler ${CROSS_COMPILE}gcc not exist." && exit 1

echo "LOG: Use Cross Compiler: ${CROSS_COMPILE}gcc"
echo

if [ $DEFCONFIG -eq 1 ]; then
	if [ -n "${BOARD_REVISON}" ]; then
		defconfig=${PHONE_SERIES}_${BOARD_REVISON}_${CONFIG_TYPE}"_defconfig"
	else
		defconfig=${PHONE_SERIES}_${CONFIG_TYPE}"_defconfig"
	fi
	defconfig_file=arch/arm/configs/$defconfig
	[ ! -f $defconfig_file ] && echo "ERROR: Configure file: $defconfig_file not exist." && exit 2

	echo "LOG: Configure for $CONFIG_TYPE with $defconfig_file"
	echo
	make ARCH=arm $defconfig
else
	make ARCH=arm ${CONFIG_METHOD}config
fi

[ $? -ne 0 ] && echo "ERROR: Configure error, Exit." && exit 3

[ $INTERACTIVE -eq 1 ] && make ARCH=arm menuconfig

# Hack some configs here
[ $BOARD -eq 1 ] && sed -i -e "s/# CONFIG_MEIZU_M6X_V3_BOARD is not set/CONFIG_MEIZU_M6X_V3_BOARD=y/g" .config

if [ $PM -eq 1 ]; then
	sed -i -e "s/CONFIG_BATTERY_MAX17047_DEBUG=y/# CONFIG_BATTERY_MAX17047_DEBUG is not set/g" .config
	sed -i -e "s/# CONFIG_LEDS_GPIO is not set/CONFIG_LEDS_GPIO=y/g" .config
	sed -i -e "s/# CONFIG_LEDS_TRIGGERS is not set/CONFIG_LEDS_TRIGGERS=y/g" .config
fi
	
sed -i -e "s/# CONFIG_DEBUG_SECTION_MISMATCH is not set/CONFIG_DEBUG_SECTION_MISMATCH=y/g" .config

echo
echo "LOG: Compile with $CPU_JOB_NUM threads: make -j$CPU_JOB_NUM"
echo

rm -f $COMPILE_LOG
script -e -c "make -j$CPU_JOB_NUM" -a $COMPILE_LOG
[ $? -ne 0 ] && echo "ERROR: Compile error, Exit." && exit 4

END_TIME=`date +%s`

let "ELAPSED_TIME=$END_TIME-$START_TIME"
echo
echo "Total compile time is $ELAPSED_TIME seconds, compile log saved in $COMPILE_LOG"
