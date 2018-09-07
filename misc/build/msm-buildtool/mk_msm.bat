@echo off
@rem ================================================================
@rem Build batch for OpenRTM-aist Merge Module
@rem 
@rem set the variables required for common scripts.
@rem   ARCH: x86 or x86_64
@rem   PKG_NAME: runtime, develop, examples,..
@rem   VC_VERSION: 9(=VC2008), 10(=VC2010), 11(=VC2012), 12(=VC2013),..
@rem   GUIDS_FILE: *_guids.txt file full path
@rem   TARGET_NAME: msm file name
@rem      openrtm112_runtime_x86_vc*.msm ÅÀ TARGET_NAME=openrtm112
@rem   TARGET_DIR:  specify a target directory to be searched
@rem
@rem set the variables required for msm_template.wxs.
@rem   TARGET_VERSION: OpenRTM-aist version
@rem ================================================================

set PKG_NAME=%1
set VC_VERSION=%2
set GUIDS_FILE=%3

set TARGET_NAME=choreonoid%CNOID_SHORT_VER%
set TARGET_VERSION=%CHOREONOID_VERSION%

set TARGET_DIR=choreonoid_%PKG_NAME%\%INSTALL_VERSION%

echo %TARGET_DIR%

@rem ------------------------------------------------------------
@rem call msm build common script
call scripts\make_msm.bat

@set MSM_FILE_NAME=
@echo on
