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
	local config=$(echo ${2#*_})

	config=$(echo ${config%_*})
	cp vmlinux output/r$1.$config.vmlinux
	cp System.map output/r$1.$config.System.map
}

function save_all_image()
{
	if [ -d ~/dev/kernelimage ];then
		if [ -d ~/dev/kernelimage/r$1 ];then
			/bin/rm ~/dev/kernelimage/r$1 -rf
		fi
		cp output ~/dev/kernelimage/r$1 -rf
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
	local svn_rev=$(get_svn_ver)
	make distclean -j$CPU_JOB_NUM

	function check_kernel_build_result() {
		if [ $? != 0 ];then 
			echo "$1 build error!!"
			exit -1
		fi
	}

	make mx2_eng_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_eng_defconfig
	mv arch/arm/boot/zImage output/zImage-dev
	save_image $svn_rev mx2_eng_defconfig

	make mx2_user_defconfig 
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_user_defconfig 
	mv arch/arm/boot/zImage output/zImage
	save_image $svn_rev mx2_user_defconfig

	make mx2_unicom_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_unicom_defconfig
	mv arch/arm/boot/zImage output/zImage-unicom
	save_image $svn_rev mx2_unicom_defconfig

    make mx2_cmcc_defconfig
    make all -j$CPU_JOB_NUM
    check_kernel_build_result mx2_cmcc_defconfig
    mv arch/arm/boot/zImage output/zImage-cmcc
    save_image $svn_rev mx2_cmcc_defconfig

	make mx2_recovery_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_recovery_defconfig
	mv arch/arm/boot/zImage output/zImage-recovery
	save_image $svn_rev mx2_recovery_defconfig

	make mx2_release_recovery_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_release_recovery_defconfig
	mv arch/arm/boot/zImage output/zImage-release-recovery
	save_image $svn_rev mx2_release_recovery_defconfig

	make mx2_overseas_defconfig
	make all -j$CPU_JOB_NUM
	check_kernel_build_result mx2_overseas_defconfig
	mv arch/arm/boot/zImage output/zImage-overseas
	save_image $svn_rev mx2_overseas_defconfig

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

