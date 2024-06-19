/*
  This file is part of Choreonoid, an extensible graphical robotics application suite.
  Copyright (c) 2019-2024 Choreonoid Inc.
  Copyright (c) 2007-2019 National Institute of Advanced Industrial Science and Technology (AIST)
  Released under the MIT license. See 'LICENSE' for more information.
*/

#include <cnoid/App>

int main(int argc, char *argv[])
{
    cnoid::App app(argc, argv, "Choreonoid", "Choreonoid");
    app.setBuiltinProject(":/Base/project/layout.cnoid");
    return app.exec();
}
