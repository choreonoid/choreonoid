@echo off

echo ---- 01(Base) build
call build.bat 01 > ..\01.log 2>&1
echo %RETVAL%

echo ---- 02(OpenRTM) build
call build.bat 02 > ..\02.log 2>&1
echo %RETVAL%

echo ---- 03(Python2) build
call build.bat 03 > ..\03.log 2>&1
echo %RETVAL%

