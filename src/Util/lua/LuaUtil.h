/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYUTIL_H
#define CNOID_UTIL_PYUTIL_H

#include "../Referenced.h"
#include <sol.hpp>
#include <iosfwd>
#include "exportdecl.h"

namespace sol {

template <typename T>
struct unique_usertype_traits<cnoid::ref_ptr<T>> {
    typedef T type;
    typedef cnoid::ref_ptr<T> actual_type;
    static const bool value = true;
    static bool is_null(const actual_type& value) {
        return value == nullptr;
    }
    static type* get (const actual_type& p) {
        return p.get();
    }
};

}

namespace cnoid {

CNOID_EXPORT void stackDump(lua_State* L, std::ostream& os);

}

#endif
