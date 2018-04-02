@echo off
@rem ================================================================
@rem OpenRTM-aist installer for Windows build batch
@rem
@rem Installer has been created using the following merge module file
@rem     OpenRTM-aist release version
@rem     OpenRTM-aist old version runtime
@rem     omniORB
@rem     OpenCV
@rem     OpenCV Components
@rem     JRE (OpenJDK)
@rem     OpenRTP, RTSystemEditorRCP
@rem     OpenRTM-aist-Python release version
@rem     OpenRTM-aist-Java release version
@rem     RTShell
@rem These msm file is placed in MSM_URL
@rem ================================================================

@if "%1" == "" exit/b
@if "%2" == "" exit/b
@set SVN_USER=%1
@set SVN_USER_PASSWD=%2

if not defined CHOREONOID_VERSION set CHOREONOID_VERSION=1.7.0

@rem if not defined MSM_URL set MSM_URL=http://openrtm.org/pub/Windows/OpenRTM-aist/msi-buildtools/%OPENRTM_VERSION%/

@set INSTALL_VERSION=%CHOREONOID_VERSION%
@set SHELL=C:\cygwin64\bin

if exist Msi rmdir /s /q Msi
if exist *.msi del *.msi
@rem ------------------------------------------------------------
@rem Supported languages
@rem   supported languages have to be specified
@rem ------------------------------------------------------------
@set LANGUAGES=(ja-jp)

@rem ------------------------------------------------------------
@rem set GUID file path 
@rem ------------------------------------------------------------
@set PWD=%CD%
@set GUIDS_FILE=%PWD%\Choreonoid_msi_guids.txt

@rem ------------------------------------------------------------
@rem downloading common scripts 
@rem ------------------------------------------------------------
@set URL=http://openrtm.org/svn/msi-buildtool/trunk/scripts
@%SHELL%\svn co --username %SVN_USER% --password %SVN_USER_PASSWD% %URL% scripts
@set SCRIPT_DIR=scripts\

rem ------------------------------------------------------------
rem msm download
rem ------------------------------------------------------------
rem if exist MergeModules rmdir /s /q MergeModules
rem mkdir MergeModules
rem %SHELL%\wget -A msm -nd -r -l 1 -P MergeModules %MSM_URL%

@rem ------------------------------------------------------------
@rem Import Language-Country, Language codes, Codepages
@rem from langs.txt
@rem http://www.tramontana.co.hu/wix/lesson2.php#2.4
@rem ------------------------------------------------------------
for /F "tokens=1,2,3,4 delims=, " %%i in (langs.txt) do (
    set LC[%%j]=%%j
    set LANG[%%j]=%%k
    set CODE[%%j]=%%l
)

rem ------------------------------------------------------------
rem msi build (32bit)
rem ------------------------------------------------------------
@set ARCH=x86
@set ARCH_NAME=win32

@rem @call build_msi.bat

rem ------------------------------------------------------------
rem msi build (64bit)
rem ------------------------------------------------------------
@set ARCH=x86_64
@set ARCH_NAME=win64

@call build_msi.bat

@echo on

