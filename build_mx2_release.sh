#!/bin/bash

function get_svn_ver()
{
	if [ -d .svn ];then
		if rev=`svn info 2>/dev/null` ; then
			if [ $(locale |grep LANGUAGE|grep zh_CN) ];then
				rev=`echo "${rev}" | grep '^最后修改的版本' | awk '{print $NF}'`
			else
				rev=`echo "${rev}" | grep '^Last Changed Rev' | awk '{print $NF}'`
			fi
			printf -- '%s' $rev
		fi 
	else
		printf '0' 
	fi
}

function save_image()
{
	local config=$(echo ${1#*_})

	config=$(echo ${config%_*})
	cp vmlinux r$1.$config.vmlinux
	cp System.map r$1.$config.System.map
}

function save_all_image()
{
	if [ -d ~/dev/kernelimage ];then
		cp output ~/dev/kernelimage/r$1 -r
	fi
}

function gen_all_kernel_image()
{
	# Some tools requires an english envrionment
	export LANG="en_US.UTF-8"
	export LANGUAGE="en_US:en"
	export CROSS_COMPILE=arm-none-linux-gnueabi-
	CPU_JOB_NUM=$(grep processor /proc/cpuinfo | awk '{field=$NF};END{print field+1}')

	mkdir -p output
	svn up
	make distclean -j$CPU_JOB_NUM

	local svn_rev=$(get_svn_ver)

	function check_kernel_build_result() {
		if [ $? != 0 ];then 
			echo "$1 build error!!"
			exit -1
		fi
	}

	make mx2_recovery_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_recovery_defconfig
	mv arch/arm/boot/zImage output/zImage-recovery
	save_image mx2_recovery_defconfig

	make mx2_user_defconfig 
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_user_defconfig 
	mv arch/arm/boot/zImage output/zImage
	save_image mx2_user_defconfig

	make mx2_overseas_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_overseas_defconfig
	mv arch/arm/boot/zImage output/zImage-overseas
	save_image mx2_overseas_defconfig

	make mx2_eng_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_eng_defconfig
	mv arch/arm/boot/zImage output/zImage-dev
	save_image mx2_eng_defconfig

	save_all_image $svn_rev

	unset check_kernel_build_result
}

if [ $# -gt 0 -a "x$1" == "xmain" ];then
	gen_all_kernel_image
	unset gen_all_kernel_image
	unset get_svn_ver
	unset save_image
	unset save_all_image
fi

