#include <cnoid/App>

int main(int argc, char *argv[])
{
    cnoid::App app(argc, argv, "Choreonoid", "Choreonoid");
    app.setBuiltinProject(":/Base/project/layout.cnoid");
    return app.exec();
}
