/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LUA_BASE_H
#define CNOID_BASE_LUA_BASE_H

#include <cnoid/LuaUtil>

namespace cnoid {

template<typename ItemType>
void defineLuaItemFunctions(const char* itemTypeName, sol::table& module)
{
    static const std::string extract("extract");

    module[extract + itemTypeName + "s"] = [](sol::table items) {
        sol::state_view lua(items.lua_state());
        sol::table extracted = lua.create_table();
        const int n = items.size();
        int index = 1;
        for(int i=0; i < n; ++i){
            sol::object obj = items[i];
            if(obj.is<Item*>()){
                Item* item = obj.as<Item*>();
                ItemType* casted = dynamic_cast<ItemType*>(item);
                if(casted){
                    extracted[index++] = casted;
                }
            }
        }
        return extracted;
    };
}

}

#endif
