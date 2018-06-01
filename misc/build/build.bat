@echo off
@rem ============================================================
@rem Choreonoid for Windows build batch
@rem
@rem Environment vairables required
@rem
@rem ARCH: x86_64
@rem VC_VERSION: 14(=VC2015) 
@rem PYTHON_DIR: c:\Python27
@rem BUILD_OPTION: 01(Base)
@rem               02(OpenRTM)
@rem               03(Python2)
@rem
@rem Please copy this bat file directly under the choreonoid and 
@rem execute it.
@rem   Example:
@rem   cd choreonoid
@rem   copy misd\build\build.bat .
@rem   call build.bat 01
@rem ============================================================

@if "%1" == "" (
  echo error: Argument [01-03] is missing.
  exit /b
)
@set BUILD_OPTION=CMAKE_OP_%1

set PATH_ORG=%PATH%
set TOP_DIR=%CD%
set PATH=%PATH%;C:\cygwin\bin;C:\cygwin64\bin

if not DEFINED ARCH       set ARCH=x86_64
if not DEFINED VC_VERSION set VC_VERSION=14
if not DEFINED PYTHON_DIR set PYTHON_DIR=c:\python27_x64
if not DEFINED BOOST_ROOT set BOOST_ROOT=C:\local\boost_1_64_0
if not DEFINED EIGEN_DIR  set EIGEN_DIR=c:\local\eigen-3.3.4
if not DEFINED QT_CMAKR_DIR set QT_CMAKR_DIR=c:\Qt\5.9.1\msvc2015_64\lib\cmake
if not DEFINED ASSIMP_DIR   set ASSIMP_DIR=c:\local\Assimp-vc2015
if not DEFINED ODE_DIR      set ODE_DIR=C:\local\ode-0.12

set PYTHONPATH=%PYTHON_DIR%\Lib
set TARGET=Choreonoid_%BUILD_OPTION%_%ARCH%
set INSTALL_DIR=%TOP_DIR%\%TARGET%

if %VC_VERSION% == 141 (
  set VS_VERSION=15
) else (
  set VS_VERSION=%VC_VERSION%
)

@rem ------------------------------------------------------------
@rem Printing env variables
echo ARCH       : %ARCH%
echo VC_VERSION : %VC_VERSION%
echo PYTHON_DIR : %PYTHON_DIR%

set PATH_ORG=%PATH%
set PATH=%PYTHON_DIR%;%PATH%;

if %ARCH% == x86_64    set DLL_ARCH=_x64

@rem ============================================================
@rem make build dir 
@rem ============================================================
if exist build_%BUILD_OPTION%_%ARCH% rmdir /s/q build_%BUILD_OPTION%_%ARCH%
mkdir build_%BUILD_OPTION%_%ARCH%
cd build_%BUILD_OPTION%_%ARCH%

@rem ============================================================
@rem CMake common option
@rem ============================================================
set Qt5Widgets_PATH=%QT_CMAKR_DIR%/Qt5Widgets
set Qt5OpenGL_PATH=%QT_CMAKR_DIR%/Qt5OpenGL
set Qt5Network_PATH=%QT_CMAKR_DIR%/Qt5Network
set Qt5Core_PATH=%QT_CMAKR_DIR%/Qt5Core

set COMMON_OPT=-D EIGEN_DIR:PATH=%EIGEN_DIR% ^
               -D Qt5Core_DIR:PATH=%Qt5Core_PATH% ^
               -D Qt5Widgets_DIR:PATH=%Qt5Widgets_PATH% ^
               -D Qt5OpenGL_DIR:PATH=%Qt5OpenGL_PATH% ^
               -D Qt5Network_DIR:PATH=%Qt5Network_PATH% ^
               -D CMAKE_INSTALL_PREFIX:PATH=%INSTALL_DIR% ^
               -D INSTALL_SDK:BOOL=ON
               
set DEFALUT_OPT_OFF=-D BUILD_BALANCER_PLUGIN:BOOL=OFF ^
                    -D BUILD_POSE_SEQ_PLUGIN:BOOL=OFF ^
                    -D BUILD_SIMPLE_CONTROLLER_SAMPLES:BOOL=OFF

@rem ============================================================
@rem start to cmake 64bit 
@rem ============================================================
:cmake_x86_64
set BUILD_DIR=%CD%
echo %BUILD_DIR%
set VC_NAME="Visual Studio %VS_VERSION% Win64"
call :%BUILD_OPTION%

@rem ============================================================
@rem  Compiling 64bit binaries
@rem ============================================================
:x86_64
echo Compiling 64bit binaries
if /i %VC_VERSION% == 14 (
   call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64
   set VCTOOLSET=14.0
   set PLATFORMTOOL=/p:PlatformToolset=v140
   goto MSBUILDx64
   )
if /i %VC_VERSION% == 141 (
   call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64
   set VCTOOLSET=15.0
   set PLATFORMTOOL=/p:PlatformToolset=v141
   goto MSBUILDx64
   )

@rem ------------------------------------------------------------
@rem Build (VC2015- x64)
@rem ------------------------------------------------------------
:MSBUILDx64
set OPT=/M:4 /toolsversion:%VCTOOLSET% %PLATFORMTOOL% /p:platform=x64

:BUILD_EXECUTE
set SLN=%BUILD_DIR%\Choreonoid.sln
set PRJ=%BUILD_DIR%\INSTALL.vcxproj

msbuild /t:rebuild /p:configuration=release %OPT% %SLN%
set BUILD_STATUS=%ERRORLEVEL%
if not %BUILD_STATUS%==0 (
  set RETVAL=ERROR
  goto END
)

echo Compiling INSTALL.vcxproj
msbuild /t:build /p:configuration=release %OPT% %PRJ%
set BUILD_STATUS=%ERRORLEVEL%
if not %BUILD_STATUS%==0 (
  set RETVAL=ERROR
  goto END
)

cd %TOP_DIR%
set ZIP_FILE=%TARGET%.zip
if exist %ZIP_FILE% del %ZIP_FILE%
zip -r %ZIP_FILE% %TARGET%
set RETVAL=SUCCESS

:END
cd %TOP_DIR%
set PATH=%PATH_ORG%

exit /b

@rem ============================================================
@rem cmake build option
@rem default : BUILD_BALANCER_PLUGIN=ON
@rem           BUILD_POSE_SEQ_PLUGIN=ON
@rem           BUILD_SIMPLE_CONTROLLER_SAMPLES=ON
@rem ============================================================
:CMAKE_OP_01
echo BUILD_OPTION *-*-* Base
cmake %DEFALUT_OPT_OFF% ^
      %COMMON_OPT% ^
      -D BUILD_ASSIMP_PLUGIN:BOOL=ON ^
      -D BUILD_BALANCER_PLUGIN:BOOL=ON ^
      -D BUILD_POSE_SEQ_PLUGIN:BOOL=ON ^
      -D ASSIMP_DIR:PATH=%ASSIMP_DIR% ^
      -D BUILD_MEDIA_PLUGIN:BOOL=ON ^
      -D BUILD_ODE_PLUGIN:BOOL=ON ^
      -D ODE_DIR:PATH=%ODE_DIR% ^
      -D BUILD_SCENE_EFFECTS_PLUGIN:BOOL=ON ^
      ../../.. -G %VC_NAME%
exit /b

:CMAKE_OP_02
echo BUILD_OPTION *-*-* OpenRTMPlugin
cmake %DEFALUT_OPT_OFF% ^
      %COMMON_OPT% ^
      -D BUILD_OPENRTM_PLUGIN:BOOL=ON ^
      -D BUILD_OPENRTM_SAMPLES:BOOL=ON ^
      -D BUILD_CORBA_PLUGIN:BOOL=ON ^
      -D ENABLE_CORBA:BOOL=ON ^
      ../../.. -G %VC_NAME%
exit /b

:CMAKE_OP_03
echo BUILD_OPTION *-*-* PythonPlugin(Python27)
cmake %DEFALUT_OPT_OFF% ^
      %COMMON_OPT% ^
      -D BUILD_PYTHON_PLUGIN:BOOL=ON ^
      -D BUILD_PYTHON_SIM_SCRIPT_PLUGIN:BOOL=ON ^
      -D ENABLE_PYTHON:BOOL=ON ^
      -D USE_PYBIND11:BOOL=ON ^
      -D USE_PYTHON3:BOOL=OFF ^
      ../../.. -G %VC_NAME%
exit /b

:CMAKE_OP_04


