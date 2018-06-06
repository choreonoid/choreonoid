@echo off
@rem ================================================================
@rem Build batch for Choreonoid Merge Module
@rem
@rem @author 
@rem            Copyright (C) 2018 AIST All Rights Reserved
@rem
@rem output msm files under MergeModules directory:
@rem  ex. choreonoid170_base_x86_64_vc*.msm
@rem      choreonoid170_openrtm_x86_64_vc*.msm
@rem      choreonoid170_python2_x86_64_vc*.msm
@rem ================================================================

if not defined CHOREONOID_VERSION set CHOREONOID_VERSION=1.7.0
if not defined ARCH       set ARCH=x86_64
if not defined VC_VERSION set VC_VERSION=14
if not defined PYTHON_DIR set PYTHON_DIR=c:\Python27
if not defined TARGET_SCRIPT set TARGET_SCRIPT=c:\Jenkins-job-scripts\scripts

@set CNOID_SHORT_VER=%CHOREONOID_VERSION:.=%
@set INSTALL_VERSION=%CHOREONOID_VERSION%

@rem ------------------------------------------------------------
@rem preparing choreonoid binary
set BASE_BINARY=..\Choreonoid_CMAKE_OP_01_%ARCH%
set OPENRTM_BINARY=..\Choreonoid_CMAKE_OP_02_%ARCH%
set PYTHON2_BINARY=..\Choreonoid_CMAKE_OP_03_%ARCH%

@rem ------------------------------------------------------------
@rem set GUID file path
@set PWD=%CD%
@set GUIDS_FILE=%PWD%\Choreonoid_guids.txt

@rem ------------------------------------------------------------
@rem  copy common scripts 
xcopy /s/i/y/q %TARGET_SCRIPT% scripts

@rem ------------------------------------------------------------
@rem copy choreonoid files to specific directories
@rem  (choreonoid_base, choreonoid_openrtm, choreonoid_python2, ...)
call divide-choreonoid-binary.bat

@rem ------------------------------------------------------------
@rem call msm build script
call mk_msm.bat base %VC_VERSION% %GUIDS_FILE%
call mk_msm.bat openrtm %VC_VERSION% %GUIDS_FILE%
call mk_msm.bat python2 %VC_VERSION% %GUIDS_FILE%

if exist ..\MergeModules rmdir /s/q ..\MergeModules
mkdir ..\MergeModules
@move /Y *.msm ..\MergeModules
@del *.wixobj *.wixpdb
@del *_files.wxs *~

@echo on