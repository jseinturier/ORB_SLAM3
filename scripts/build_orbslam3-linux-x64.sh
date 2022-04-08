#!/bin/bash

# Clone and build Eigen C++ library from official Git repository.
# 
# This installation script needs that Git binaries are presents within PATH
# Git can be downloaded from https://git-scm.com/downloads
# 
# CMake is also needed and its binaries have to be pointed by PATH variable.
# CMake can be downloaded from https://cmake.org
#

echo "JSE/Orbslam3 build"

# Check if Git is available
if ! [ -x "$(command -v git)" ]; then
  echo 'Git command not found, please install git package'
  exit 1
fi

# Check if CMake is available
if ! [ -x "$(command -v cmake)" ]; then
  echo 'CMake command not found, please install CMake package'
  exit 1
fi

script_directory="$(cd "$( echo "${BASH_SOURCE[0]%/*}" )" && pwd )"

logfile=${script_directory}/build.log

gittag=master
gitproject=ORB_SLAM3

rootdir=${script_directory}/${gittag}

tmpdir=${script_directory}/tmp

gitdir=${tmpdir}/git
builddir=${tmpdir}/build
installdir=${tmpdir}/install

if [ -d "${rootdir}" ]; then
  rm -rf "${rootdir}"
fi
mkdir -p ${rootdir}

if [ -d "${tmpdir}" ]; then
  rm -rf "${tmpdir}"
fi
mkdir -p ${tmpdir}

echo "  Using repository ${gitdir}"
echo "  Installing into ${rootdir}"

echo
echo "Getting JSE/ORBSLAM3 from git repository"
if [ ! -d "${gitdir}" ]; then
  mkdir ${gitdir}
  pushd ${gitdir} > /dev/null
  git clone -b ${gittag} --single-branch --depth 1 "https://github.com/jseinturier/ORB_SLAM3.git"
  popd > /dev/null
else
  echo "${gitdir} already exists, using these sources."
  echo "Remove the directory to perform a clean download."
fi

if [ -d "${builddir}" ]; then
  rm -rf ${builddir}
fi

mkdir -p ${builddir}
mkdir -p ${builddir}/${gitproject}

if [ -d "${installdir}" ]; then
  rm -rf "${installdir}"
fi

mkdir -p "${installdir}/${gitproject}"

echo
echo "CMAKE processing"

echo "  Running CMake (see ${logfile} for details)"

pushd ${builddir}/${gitproject} > /dev/null

CMAKE_CONFIG_GENERATOR="Unix Makefiles"

CMAKE_OPTIONS=""

echo "    CMake options:"
echo "      General: ${CMAKE_OPTIONS}"

cmake -G"${CMAKE_CONFIG_GENERATOR}" ${CMAKE_OPTIONS} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${installdir}/${gitproject} ${gitdir}/${gitproject} 1>>${logfile} 2>>${logfile}

echo "  Building"
make -j$(nproc) 1>>${logfile} 2>>${logfile}
#make install 1>>${logfile} 2>>${logfile}

popd

echo "  Installing to ${rootdir}"
cp -r ${gitdir}/${gitproject}/lib ${script_directory}/${gittag}/lib
cp -r ${gitdir}/${gitproject}/include ${script_directory}/${gittag}/include
cp -r ${gitdir}/${gitproject}/Thirdparty/g2o/g2o ${script_directory}/${gittag}/include
cp -r ${gitdir}/${gitproject}/Thirdparty/DBoW2/DBoW2 ${script_directory}/${gittag}/include
cp -r ${gitdir}/${gitproject}/Thirdparty/DBoW2/DUtils ${script_directory}/${gittag}/include

echo
echo "Cleaning temporary files"
rm -rf ${tmpdir}
mv build.log ${rootdir}/build.log

echo
echo "Build done"

echo
echo "Please update your environment variables as follows:"
echo
echo "  ORBSLAM3_DIR=${rootdir}"
echo "  ORBSLAM3_LIBRARY_DIR=${rootdir}/lib"
echo "  ORBSLAM3_INCLUDE_DIR=${rootdir}/include"
echo
echo "see ${rootdir}/orbslam3_environment.sh"



