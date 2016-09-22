/**
   @author Shin'ichiro Nakaoka
*/

#include "../MessageView.h"
#include <sol.hpp>
#include <cmath>

using namespace std;
using namespace cnoid;

namespace {

}  

extern "C" int luaopen_cnoid_Base(lua_State* L)
{
    sol::state_view lua(L);

    sol::table base = lua.create_table();
    
    return 1;
}
