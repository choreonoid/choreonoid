・PhysX-3.3.0_LINUX_SDK_Core.zipをダウンロードして、展開する。
・ccmake で、BUILD_PhysX_PLUGIN を ON
             PhysX_DIR に展開したディレクトリを指定する。
・ubuntu 32bit OS の場合は、ADDITIONAL_CXX_FLAGS_RELEASE に -malign-double を指定する。

