@echo off

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

if "%ARCH%" == "x86" (
  set OS_ARCH=32-bit OS
  set PROGRAM_FOLDER=ProgramFilesFolder
  set REGISTRY_WIN64=no
)
if "%ARCH%" == "x86_64" (
  set OS_ARCH=64-bit OS
  set PROGRAM_FOLDER=ProgramFiles64Folder
  set REGISTRY_WIN64=yes
)
echo -------- ARCH : %ARCH%
echo -------- CNOID_SHORT_VER : %CNOID_SHORT_VER%

@set CNOID_MSI_VER=%TARGET%-%CHOREONOID_VERSION% (%OS_ARCH%)

@call %SCRIPT_DIR%getGuid.bat choreonoid%CNOID_SHORT_VER%_%ARCH%_upgrade %GUIDS_FILE%
@set UPGRADE_GUID=%GUID%
@call %SCRIPT_DIR%getGuid.bat choreonoid%CNOID_SHORT_VER%_%ARCH%_msi %GUIDS_FILE%
@echo off
@rem ============================================================
@rem compile wxs file and link msi
@rem ============================================================
set IDS=1033

if "%ARCH%" == "x86" (
  %WiX_BIN_DIR%candle.exe %TARGET_WXS% %LICENSE_WXS% %WIXUI_WXS% ^
        -dlanguage=%IDS% -dcodepage=1252
) else (
  %WiX_BIN_DIR%candle.exe -arch x64 %TARGET_WXS% %LICENSE_WXS% %WIXUI_WXS% ^
        -dlanguage=%IDS% -dcodepage=1252
)
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
    if "%ARCH%" == "x86" (
      %WiX_BIN_DIR%candle.exe %TARGET_WXS% %LICENSE_WXS% %WIXUI_WXS% ^
            -dlanguage=!LANG[%%i]! -dcodepage=!CODE[%%i]!
    ) else (
      %WiX_BIN_DIR%candle.exe -arch x64 %TARGET_WXS% %LICENSE_WXS% %WIXUI_WXS% ^
            -dlanguage=!LANG[%%i]! -dcodepage=!CODE[%%i]!
    )

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

