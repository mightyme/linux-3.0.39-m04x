#!/bin/bash

# Some tools requires an english envrionment
export LANG="en_US.UTF-8"
export LANGUAGE="en_US:en"

CPU_JOB_NUM=$(grep processor /proc/cpuinfo | awk '{field=$NF};END{print field+1}')

svn up

make distclean -j$CPU_JOB_NUM

mkdir -p output

function gen_all_kernel_image()
{
	function check_kernel_build_result() {
		if [ $? != 0 ];then 
			echo "$1 build error!!"
			exit -1
		fi
	}

	make mx_user_defconfig 
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx_user_defconfig 
	mv arch/arm/boot/zImage output/zImage

	make mx_overseas_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx_overseas_defconfig
	mv arch/arm/boot/zImage output/zImage-overseas

	make mx_release_recovery_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx_release_recovery_defconfig
	mv arch/arm/boot/zImage output/zImage-release-recovery

	make mx_recovery_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx_recovery_defconfig
	mv arch/arm/boot/zImage output/zImage-recovery

	make mx_eng_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx_eng_defconfig
	mv arch/arm/boot/zImage output/zImage-dev
	
	unset check_kernel_build_result

}

gen_all_kernel_image

unset gen_all_kernel_image

