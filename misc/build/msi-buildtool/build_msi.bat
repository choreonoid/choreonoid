@echo off
@rem ================================================================
@rem Choreonoid installer for Windows build batch
@rem
@rem Installer has been created using the following merge module file
@rem     choreonoid170_base_x86_64_Vc14.msm
@rem     choreonoid170_openrtm_x86_64_Vc14.msm
@rem     choreonoid170_python2_x86_64_Vc14.msm
@rem ================================================================

if not defined ARCH       set ARCH=x86_64
if not defined CHOREONOID_VERSION set CHOREONOID_VERSION=1.7.0
if not defined TARGET_SCRIPT set TARGET_SCRIPT=c:\Jenkins-job-scripts\scripts

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
@rem copy common scripts 
@rem ------------------------------------------------------------
xcopy /s/i/y/q %TARGET_SCRIPT% scripts

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

@rem ------------------------------------------------------------
@rem Variable Settings
@rem   usually only %TARGET% might be changed
@rem ------------------------------------------------------------
set TARGET=Choreonoid
set TARGET_WXS=%TARGET%.wxs
set TARGET_WIXOBJ=%TARGET%.wixobj
set TARGET_FULL=%TARGET%-%CHOREONOID_VERSION%_%ARCH%

@rem ------------------------------------------------------------
@rem Licenses Settings
@rem   usually only %WIXUI% might be changed
@rem ------------------------------------------------------------
set LICENSE=CustomLicense
set LICENSE_WXS=%LICENSE%.wxs
set LICENSE_WIXOBJ=%LICENSE%.wixobj

@rem ------------------------------------------------------------
@rem WixUI Customization Settings
@rem   usually only %WIXUI% might be changed
@rem ------------------------------------------------------------
set WIXUI=WixUI_Mondo_CNOID
set WIXUI_WXS=%WIXUI%.wxs
set WIXUI_WIXOBJ=%WIXUI%.wixobj

set CNOID_SHORT_VER=%CHOREONOID_VERSION:.=%
set WiX_BIN_DIR="%WIX%bin\"

@rem ------------------------------------------------------------
@rem msi
@rem ------------------------------------------------------------
set OS_ARCH=64-bit OS
set PROGRAM_FOLDER=ProgramFiles64Folder
set REGISTRY_WIN64=yes

echo -------- ARCH : %ARCH%
echo -------- CNOID_SHORT_VER : %CNOID_SHORT_VER%

@set CNOID_MSI_VER=%TARGET%-%CHOREONOID_VERSION% (%OS_ARCH%)

@call scripts\getGuid.bat choreonoid%CNOID_SHORT_VER%_%ARCH%_upgrade %GUIDS_FILE%
@set UPGRADE_GUID=%GUID%
@call scripts\getGuid.bat choreonoid%CNOID_SHORT_VER%_%ARCH%_msi %GUIDS_FILE%
@echo off
@rem ============================================================
@rem compile wxs file and link msi
@rem ============================================================
set IDS=1033

%WiX_BIN_DIR%candle.exe -arch x64 %TARGET_WXS% %LICENSE_WXS% %WIXUI_WXS% ^
        -dlanguage=%IDS% -dcodepage=1252

%WiX_BIN_DIR%light.exe -sw1076 -ext WixUtilExtension -ext WixUIExtension  -loc WixUI_en-us.wxl ^
        -o %TARGET_FULL%.msi %TARGET_WIXOBJ% %LICENSE_WIXOBJ% %WIXUI_WIXOBJ%

setlocal ENABLEDELAYEDEXPANSION
for %%i in %LANGUAGES% do (

    @rem ------------------------------------------------------------
    @rem language ID list
    @rem
    set IDS=!IDS!,!LANG[%%i]!

    @rem ------------------------------------------------------------
    @rem compile wxs file and link msi
    @rem
    %WiX_BIN_DIR%candle.exe -arch x64 %TARGET_WXS% %LICENSE_WXS% %WIXUI_WXS% ^
            -dlanguage=!LANG[%%i]! -dcodepage=!CODE[%%i]!

    if exist WixUI_!LC[%%i]!.wxl (
        %WiX_BIN_DIR%light.exe -sw1076 -ext WixUtilExtension -ext WixUIExtension  -loc WixUI_!LC[%%i]!.wxl ^
                -o %TARGET_FULL%_!LC[%%i]!.msi ^
                %TARGET_WIXOBJ% %LICENSE_WIXOBJ% %WIXUI_WIXOBJ%
    )
    if not exist WixUI_!LC[%%i]!.wxl (
        %WiX_BIN_DIR%light.exe -sw1076 -ext WixUtilExtension -ext WixUIExtension  -cultures:!LC[%%i]! ^
                -o %TARGET_FULL%_!LC[%%i]!.msi ^
                %TARGET_WIXOBJ% %LICENSE_WIXOBJ% %WIXUI_WIXOBJ%
    )

    @rem ------------------------------------------------------------
    @rem creating transformation files
    @rem
    %WiX_BIN_DIR%torch.exe -p -t language %TARGET_FULL%.msi ^
            %TARGET_FULL%_!LC[%%i]!.msi -out !LC[%%i]!.mst

    @rem ------------------------------------------------------------
    @rem embed transformation files
    @rem
    cscript wisubstg.vbs %TARGET_FULL%.msi !LC[%%i]!.mst !LANG[%%i]!
)

@rem ------------------------------------------------------------
@rem Update the summary information stream to list all
@rem supported languages of this package
@rem ------------------------------------------------------------
cscript WiLangId.vbs %TARGET_FULL%.msi Package %IDS%

if not exist Msi mkdir Msi
@move /Y *_%ARCH%.msi Msi
@del *.wixobj *.wixpdb

@echo on

