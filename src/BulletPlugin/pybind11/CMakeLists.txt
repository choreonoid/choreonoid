set(target PyBulletPlugin)
choreonoid_add_python_module(${target} PyBulletPlugin.cpp)
target_link_libraries(${target} CnoidBulletPlugin CnoidPyBase)
