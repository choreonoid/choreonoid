[![Build Status](https://travis-ci.org/fkanehiro/choreonoid-sdfloader-plugin.svg?branch=master)](https://travis-ci.org/fkanehiro/choreonoid-sdfloader-plugin)


# choreonoid-sdfloader-plugin


概要
----

Choreonoid (https://github.com/s-nakaoka/choreonoid) で SDF 形式および URDF 形式のモデルを読み込み可能とするプラグインです。


使用方法
--------

本プラグインは chorenoid_ros_pkg (https://github.com/fkanehiro/choreonoid_ros_pkg) を依存パッケージとしている為、事前に choreonoid_ros_pkg を導入しておいてください。

ビルド方法

1. choreonoid_ros_pkg の導入時に作成した catkin workspace へ移動します。

   ```
   $ cd ~/catkin_ws
   ```

1. 本プラグインの取得、依存パッケージの導入、本パッケージのビルドを行います。

   ```
   $ cd ~/catkin_ws/src
   $ wstool set choreonoid_sdfloader_plugin https://github.com/fkanehiro/choreonoid-sdfloader-plugin.git --git
   $ wstool update
   $ cd ~/catkin_ws
   $ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
   $ catkin build choreonoid_sdfloader_plugin
   ```

実行方法

1. Choreonoid を起動します。

   以下のコマンドを実行します。

   ```
   $ ~/catkin_ws/devel/bin/choreonoid
   ```

1. Chorenoid のメッセージビューに以下の出力がある事を確認します。

   ```
   SDFLoaderプラグインが読み込まれました。
   ```

1. Choreonoid のメニュー「ファイル」「読み込み」「Gazebo Model File」を選択し、読み込むモデルを選択します。


注記
----

- モデル読み込み時、Chorenoid のメッセージビューに以下のような警告メッセージが出力される場合、読み込みを試みたモデルへのパスを環境変数 ROS_PACKAGE_PATH もしくは GAZEBO_MODEL_PATH に含めるか、環境変数に含まれるパスへモデルを移動する事を試みてください。

  ```
  Warning: model://... not found
  ```

- 現時点では、正常に読み込めないモデルが多く存在しますが、順次改善してゆく予定です。

