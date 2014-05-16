*Windows
bullet-2.82-r2704.zipをダウンロードして展開。
cmakeを起動、USE_DOUBLE_PRECISION　にチェックを入れる。
USE_MSVC_RUNTIME_LIBRARY_DLL に　チェックを入れる。
VC++のソリューションファイルを起動する。
ビルドする。

*linux
bullet-2.82-r2704.zipをダウンロードして展開。
cmakeを起動、
cmake .. -DINSTALL_LIBS=ON -DBUILD_SHARED_LIBS=ON
USE_DOUBLE_PRECISION　にチェックを入れる。
INSTALL_EXTRA_LIBSをONにする。
make
sudo make install

