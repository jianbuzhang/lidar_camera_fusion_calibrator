#!/bin/bash

ROOTDIR=`pwd`

VERSION="0.1.0"

cmakebuild(){
	if [ ! -d build ];then
		mkdir build
	fi
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make
}

linuxdeployqt_package(){
	if [ ! -d deploy_package ];then
		mkdir deploy_package
	fi
	cp CalibratorTool deploy_package
	cd deploy_package
	linuxdeployqt CalibratorTool -appimage -no-strip
	cd ..
}

deb_package(){
	if [ ! -d deb_package ];then
		mkdir deb_package
	fi
	cd deb_package
	if [ ! -d source ];then
		mkdir source
	fi
	if [ ! -d source/opt ];then
		mkdir -p source/opt
	fi
	if [ ! -d source/opt/CalibratorTool ];then
		mkdir -p source/opt/CalibratorTool
	fi
	if [ ! -d source/DEBIAN ];then
		mkdir -p source/DEBIAN
	fi
	cd ..
	cp -rp deploy_package/* deb_package/source/opt/CalibratorTool
	cp ${ROOTDIR}/debfiles/CalibratorTool.desktop deb_package/source/opt/CalibratorTool
	cp ${ROOTDIR}/icon/icon.png deb_package/source/opt/CalibratorTool
	cp ${ROOTDIR}/config/initparam.yaml deb_package/source/opt/CalibratorTool
	cp ${ROOTDIR}/debfiles/control deb_package/source/DEBIAN
	cp ${ROOTDIR}/debfiles/postinst deb_package/source/DEBIAN
	cp ${ROOTDIR}/debfiles/postrm deb_package/source/DEBIAN
	cd deb_package/source
	dpkg -b . CalibratorTool_amd64_${VERSION}.deb

	if [ ! -d ${ROOTDIR}/debout ];then
		mkdir -p ${ROOTDIR}/debout
	fi

	cp CalibratorTool_amd64_${VERSION}.deb ${ROOTDIR}/debout
	cd ${ROOTDIR}
	rm -rf build
}

cmakebuild
linuxdeployqt_package
deb_package