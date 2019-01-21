/*!
  @author Shin'ichiro Nakaoka
*/

#include "LuaUtil.h"
#include "../EigenUtil.h"
#include <fmt/format.h>

using namespace std;
using fmt::format;
using namespace cnoid;

namespace cnoid {

void exportLuaEigenTypes(sol::table& module)
{
    module.new_usertype<Vector3>(
        "Vector3",
        "new", sol::factories(
            []() { return make_shared_aligned<Vector3>(); },
            [](double x, double y, double z) { return make_shared_aligned<Vector3>(x, y, z); }),
        sol::call_constructor, sol::factories(
            [](sol::table self){ return make_shared_aligned<Vector3>(); },
            [](sol::table self, double x, double y, double z){ return make_shared_aligned<Vector3>(x, y, z); }),
        sol::meta_function::index, [](Vector3& self, int index) { return self[index]; },
        sol::meta_function::new_index, [](Vector3& self, int index, double value) { self[index] = value; },
        sol::meta_function::unary_minus, [](Vector3& self) { return make_shared_aligned<Vector3>(-self); },
        sol::meta_function::addition, [](Vector3& self, Vector3& other) { return make_shared_aligned<Vector3>(self + other); },
        sol::meta_function::subtraction, [](Vector3& self, Vector3& other) { return make_shared_aligned<Vector3>(self - other); },
        sol::meta_function::multiplication, [](Vector3& self, Vector3& other) { return self.dot(other); },
        "dot", [](Vector3& self, Vector3& other) { return self.dot(other); },
        "cross", [](Vector3& self, Vector3& other) { return make_shared_aligned<Vector3>(self.cross(other)); },
        "toString", [](Vector3& self) { return format("{ {0} {1} {2} }", self[0], self[1], self[2]); }
        );
}

}
