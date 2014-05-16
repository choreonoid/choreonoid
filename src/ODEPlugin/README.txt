**ODE ソースからコンパイルしてstaticライブラリを作成する**

1.ソースの取得
apt-get source libode-dev

2.コンパイル
cd ode-0.11.1/

CXXFLAGS="-fPIC -fvisibility=hidden" ./configure --enable-static --enable-release --enable-double-precision --with-trimesh=opcode --disable-demos 

make
sudo make install

インストール先(/usr/local/lib)に libode.a ができる。


**Gazebo_ode　ソースからコンパイルしてstaticライブラリを作成する**

http://gazebosim.org/wiki/3.0/installに従って、依存ライブラリをインストール。

Build And Install GazeboのConfigure Gazebo (choose either method a or b below)の前に

/gazebo/CMakeLists.txtの中の
set (BUILD_GAZEBO ON CACHE INTERNAL "Build Gazebo" FORCE)
を
set (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
に書き換える。

/gazebo/cmake/GazeboUtils.cmakeファイルに

macro (gz_add_static_library _name)
  add_library(${_name} STATIC ${ARGN})
  get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
  if(NOT compile_flags)
      set(compile_flags "")
  endif()
  set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fPIC -fvisibility=hidden")
  target_link_libraries (${_name} ${general_libraries})
endmacro ()

を追加する。

gazebo/deps/opende/CMakeLists.txt
gazebo/deps/opende/OPCODE/CMakeLists.txt
gazebo/deps/opende/GIMPACT/CMakeLists.txt
gazebo/deps/opende/ou/CMakeLists.txt
の中に書かれている
gz_add_library(gazebo_*** ${sources})
の行を
gz_add_static_library(gazebo_*** ${sources})
に書き換える。

cmake ..
make
sudo make install

