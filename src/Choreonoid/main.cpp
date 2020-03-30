/*
  This file is part of Choreonoid, an extensible graphical robotics application suite.
  Copyright (c) 2019-2020 Choreonoid Inc.
  Copyright (c) 2007-2019 National Institute of Advanced Industrial Science and Technology (AIST)
  Released under the MIT license. See 'LICENSE' for more information.
*/

#include <cnoid/App>

int execute(cnoid::App& app);

#ifdef _WIN32
#include <windows.h>

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    cnoid::App app(hInstance, hPrevInstance, lpCmdLine, nCmdShow);
    return execute(app);
}
#endif

int main(int argc, char *argv[])
{
    cnoid::App app(argc, argv);
    return execute(app);
}

int execute(cnoid::App& app)
{
    app.initialize("Choreonoid", "Choreonoid");
    app.exec();
    return 0;
}
