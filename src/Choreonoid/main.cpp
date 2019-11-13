/*
  This file is part of Choreonoid, an extensible graphical robotics application suit.
  Copyright (c) 2007-2014 National Institute of Advanced Industrial Science and Technology (AIST)
  Released under the MIT license. See accompanying file 'LICENSE' for more information.
*/

#include <cnoid/Config>
#include <cnoid/App>
#include <cstdlib>

#ifdef _WIN32
#include <windows.h>
#include <vector>
#include <boost/program_options.hpp>
#endif

using namespace std;
using namespace cnoid;

int main(int argc, char *argv[])
{
    cnoid::App app(argc, argv);

    app.initialize("Choreonoid", "Choreonoid", getenv("CNOID_PLUGIN_PATH"));
    app.exec();
    
    return 0;
}


#ifdef _WIN32

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    vector<char*> argv;
    char appname[] = "application";
    argv.push_back(appname);
    
#ifndef UNICODE
    vector<string> args = boost::program_options::split_winmain(lpCmdLine);
    for(size_t i=0; i < args.size(); ++i){
        char* arg = new char[args[i].size() + 1];
        strcpy(arg, args[i].c_str());
        argv.push_back(arg);
    }

#else
    vector<wstring> wargs = boost::program_options::split_winmain(lpCmdLine);
    vector<char> buf;
    vector<string> args(wargs.size());
    int codepage = _getmbcp();
    for(size_t i=0; i < args.size(); ++i){
        const int size = WideCharToMultiByte(codepage, 0, &wargs[i][0], wargs[i].size(), NULL, 0, NULL, NULL);
        char* arg;
        if(size > 0){
            arg = new char[size + 1];
            WideCharToMultiByte(codepage, 0, &wargs[i][0], wargs[i].size(), arg, size + 1, NULL, NULL);
        } else {
            arg = new char[1];
            arg[0] = '\0';
        }
        argv.push_back(arg);
    }
#endif
  
    return main(argv.size(), &argv[0]);
}
#endif
