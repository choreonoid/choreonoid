@echo off
@rem 
@rem @brief Preparing choreonoid binary
@rem @author 
@rem          Copyright 2018 (C) AIST All Right Reserved
@rem 

set BASE_BINARY=Choreonoid_CMAKE_OP_01_%ARCH%
set OPENRTM_BINARY=Choreonoid_CMAKE_OP_02_%ARCH%
set PYTHON2_BINARY=Choreonoid_CMAKE_OP_03_%ARCH%

if exist %BASE_BINARY% rmdir /s/q %BASE_BINARY%
if exist %OPENRTM_BINARY% rmdir /s/q %OPENRTM_BINARY%
if exist %PYTHON2_BINARY% rmdir /s/q %PYTHON2_BINARY%
@rem del /q 	%BASE_BINARY%.zip
@rem del /q 	%OPENRTM_BINARY%.zip
@rem del /q 	%PYTHON2_BINARY%.zip

@rem C:\Cygwin64\bin\wget http://choreonoid.org/staging/%BASE_BINARY%.zip
@rem C:\Cygwin64\bin\wget http://choreonoid.org/staging/%OPENRTM_BINARY%.zip
@rem C:\Cygwin64\bin\wget http://choreonoid.org/staging/%PYTHON2_BINARY%.zip

C:\Cygwin64\bin\unzip.exe %BASE_BINARY%.zip
C:\Cygwin64\bin\unzip.exe %OPENRTM_BINARY%.zip
C:\Cygwin64\bin\unzip.exe %PYTHON2_BINARY%.zip
