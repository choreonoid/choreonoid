This plugin is not complete yet.

Dependency:
 libsdformat4-dev  (4.0.0)
 libassimp-dev   (3.2)
 libogre-1.9-dev (1.9)

Issues:
- How to specify the default model file path
 Currently, the path is specified with environmental variable ROS_PACKAGE_PATH or GAZEBO_MODEL_PATH.
- Geometry types such as plane are not supported
- Parameters such as pose static and self_collision are not supported
- Sensors have not been debugged
- The implementation of loading tgazebo_color of Material should be improved
- Close link loop is not supported
- Transparency becomes inverse in some model files
- Nesting models have not been debugged
- Putting error messages has not been implemented






