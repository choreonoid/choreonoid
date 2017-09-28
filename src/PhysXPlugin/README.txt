・PhysX-3.3.0_LINUX_SDK_Core.zipをダウンロードして、展開する。
・ccmake で、BUILD_PhysX_PLUGIN を ON
             PhysX_DIR に展開したディレクトリを指定する。
・ubuntu 32bit OS の場合は、ADDITIONAL_CXX_FLAGS_RELEASE に -malign-double を指定する。

***** 2016.10.18 追記 
NVIDIAのアカウント必要
https://developer.nvidia.com/content/apply-access-nvidia-physx-source-code
ここからGitHubのアカウント入力
メールが届くのでそこからアクセスしてソースを取得

PhysXSDK/Source/compiler/linux64に移動して
make release
すると、Lib以下にライブラリが作成される。

/home/hattori/physX/PhysX-3.3/PhysXSDK/Source/SimulationController/src
のScScene.cpp３４２２行でエラーが出るので、　&を&&に書き換える。

