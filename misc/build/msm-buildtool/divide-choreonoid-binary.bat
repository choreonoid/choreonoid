@echo off
rem ============================================================
rem Choreonoid for Windows binary copy batch
rem
rem @author 
rem                Copyright (C) 2018 AIST All Rights Reserved
rem
rem This is part of build_msm.bat.
rem
rem INSTALL_VERSION  (= 1.7.0)
rem ARCH (= x86, x86_64)
rem VC_VERSION  (= 14)
rem
rem out : choreonoid_base
rem out : choreonoid_openrtm
rem out : choreonoid_python2
rem ============================================================
set INSTALL_DIR=%INSTALL_VERSION%

echo INSTALL_DIR  %INSTALL_DIR%
echo ARCH  %ARCH%
echo VC_VERSION   %VC_VERSION%
echo CNOID_SHORT_VER  %CNOID_SHORT_VER%

set BASE=choreonoid_base
set RTM=choreonoid_openrtm
set PY2=choreonoid_python2

set BASE_WXSCTRL=WxsCtrl\choreonoid_base_wxsctrl

if exist %BASE% rmdir /s/q %BASE%
if exist %RTM% rmdir /s/q %RTM%
if exist %PY2% rmdir /s/q %PY2%

rem ------------------------------------------
rem bin 
rem ------------------------------------------
set SUBDIR=bin
set TARGET_BASE=%BASE%\%INSTALL_DIR%\%SUBDIR%
set TARGET_RTM=%RTM%\%INSTALL_DIR%\%SUBDIR%
set TARGET_PY2=%PY2%\%INSTALL_DIR%\%SUBDIR%
mkdir %TARGET_BASE%
mkdir %TARGET_RTM%
mkdir %TARGET_PY2%

echo ---- bin copy
xcopy /s/i/y/q %BASE_BINARY%\%SUBDIR% %TARGET_BASE%
xcopy /f %BASE_WXSCTRL%\*.wxsctrl %TARGET_BASE%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\cnoid-nameserver.exe %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\CnoidBodyIoRTC.dll %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\CnoidCorba.dll %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\coil*.dll %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\omni*.dll %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\RTC*.dll %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\rtm*.dll %TARGET_RTM%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\CnoidPy*.dll %TARGET_PY2%

rem ------------------------------------------
rem include 
rem ------------------------------------------
set SUBDIR=include
set TARGET_BASE=%BASE%\%INSTALL_DIR%\%SUBDIR%
set TARGET_RTM=%RTM%\%INSTALL_DIR%\%SUBDIR%
set TARGET_PY2=%PY2%\%INSTALL_DIR%\%SUBDIR%
mkdir %TARGET_BASE%
mkdir %TARGET_RTM%
mkdir %TARGET_PY2%

echo ---- include copy
xcopy /s/i/y/q %BASE_BINARY%\%SUBDIR% %TARGET_BASE%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\cnoid\src\Corba %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\cnoid\src\CorbaPlugin %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\cnoid\src\OpenRTM %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\cnoid\src\OpenRTM %TARGET_RTM%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\cnoid\src\Base\pybind11 %TARGET_PY2%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\cnoid\src\PythonPlugin %TARGET_PY2%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\cnoid\src\Util\pybind11 %TARGET_PY2%

rem ------------------------------------------
rem lib
rem ------------------------------------------
set SUBDIR=lib
set TARGET_BASE=%BASE%\%INSTALL_DIR%\%SUBDIR%
set TARGET_RTM=%RTM%\%INSTALL_DIR%\%SUBDIR%
set TARGET_PY2=%PY2%\%INSTALL_DIR%\%SUBDIR%
mkdir %TARGET_BASE%
mkdir %TARGET_RTM%
mkdir %TARGET_PY2%

echo ---- lib copy
xcopy /s/i/y/q %BASE_BINARY%\%SUBDIR% %TARGET_BASE%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\*Corba* %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\*OpenRTM* %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\choreonoid-1.7\*Corba* %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\choreonoid-1.7\*OpenRTM* %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\choreonoid-1.7\rtc %TARGET_RTM%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\*Python* %TARGET_PY2%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\PyUtil* %TARGET_PY2%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\choreonoid-1.7\*Python* %TARGET_PY2%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\choreonoid-1.7\python %TARGET_PY2%

rem ------------------------------------------
rem share
rem ------------------------------------------
set SUBDIR=share
set TARGET_BASE=%BASE%\%INSTALL_DIR%\%SUBDIR%
set TARGET_RTM=%RTM%\%INSTALL_DIR%\%SUBDIR%
set TARGET_PY2=%PY2%\%INSTALL_DIR%\%SUBDIR%
mkdir %TARGET_BASE%
mkdir %TARGET_RTM%
mkdir %TARGET_PY2%

echo ---- share copy
xcopy /s/i/y/q %BASE_BINARY%\%SUBDIR% %TARGET_BASE%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\locale\ja\LC_MESSAGES\*Corba* %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\locale\ja\LC_MESSAGES\*OpenRTM* %TARGET_RTM%
xcopy /s/i/y/q %OPENRTM_BINARY%\%SUBDIR%\project\OpenRTM* %TARGET_RTM%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\locale\ja\LC_MESSAGES\*Python* %TARGET_PY2%
xcopy /s/i/y/q %PYTHON2_BINARY%\%SUBDIR%\project\OpenRTM* %TARGET_PY2%

:END
