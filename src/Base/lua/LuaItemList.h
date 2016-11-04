/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LUA_ITEM_LIST_H
#define CNOID_BASE_LUA_ITEM_LIST_H

#include "../ItemList.h"
#include <cnoid/LuaUtil>

namespace sol {

template <>
struct lua_type_of<cnoid::ItemList<cnoid::Item>> : std::integral_constant<type, type::table> {};

namespace stack {

template <>
struct pusher<cnoid::ItemList<cnoid::Item>> {
    static int push(lua_State* L, const cnoid::ItemList<cnoid::Item>& items) {
        lua_createtable(L, items.size(), 0);
        sol::table luaItems(L);
        for(int i=0; i < items.size(); ++i){
            luaItems[i+1] = items[i];
        }
        return 1;
    }
};

template <>
struct getter<cnoid::ItemList<cnoid::Item>> {
    static cnoid::ItemList<cnoid::Item> get(lua_State* L, int index) {
        sol::table luaItems(L, index);
        const size_t n = luaItems.size();
        cnoid::ItemList<cnoid::Item> items;
        items.reserve(n);
        for(int i=1; i <= n; ++i){
            sol::object obj = luaItems[i];
            if(obj.is<cnoid::Item*>()){
                items.push_back(obj.as<cnoid::Item*>());
            }
        }
        return items;
    }
};

}

}

namespace cnoid {

template<typename ItemType>
void LuaItemList(const char* itemTypeName, sol::table& module)
{
    static const std::string extract("extract");

    module[extract + itemTypeName + "s"] = [](sol::table items) {
        sol::state_view lua(items.lua_state());
        sol::table extracted = lua.create_table();
        const int n = items.size();
        int index = 1;
        for(int i=1; i <= n; ++i){
            sol::object obj = items[i];
            if(obj.is<Item*>()){
                ItemType* casted = dynamic_cast<ItemType*>(obj.as<Item*>());
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
