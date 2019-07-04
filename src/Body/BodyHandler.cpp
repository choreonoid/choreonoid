#include "BodyHandler.h"
#include <fmt/format.h>
#include <ostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;


bool BodyHandler::checkVersion(const char* name, int version, int internalVersion, std::ostream& os)
{
    int version2 = cnoid::getVersion();

    int major1 = version / 100000;
    int major2 = version2 / 100000;
    int minor1 = (version % 100000) / 100;
    int minor2 = (version2 % 100000) / 100;
    int patch1 = version % 100;
    int patch2 = version2 % 100;

    if(major1 != major2 || minor1 != minor2){
        os << fmt::format(_("Body handler {0} built with Choreonoid {1}.{2} cannot be used on Choreonoid {3}.{4}"),
                          name, major1, minor1, major2, minor2) << endl;
        return false;
    }

    if(internalVersion != cnoid::getInternalVersion()){
        os << fmt::format(_("Body handler {1} built with Choreonoid {1}.{2}.{3} cannot be used on Choreonoid {4}.{5}.{6}"),
                          name, major1, minor1, patch1, major2, minor2, patch2) << endl;
        return false;
    }

    return true;
}
