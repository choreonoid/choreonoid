@echo off

echo ---- 01(Base) build
call build.bat 01
echo %RETVAL%

echo ---- 02(OpenRTM) build
call build.bat 02
echo %RETVAL%

echo ---- 03(Python2) build
call build.bat 03
echo %RETVAL%

