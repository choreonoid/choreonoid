/*!
  @author Shin'ichiro Nakaoka
*/

#include "LuaUtil.h"
#include "../ValueTree.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

void exportLuaValueTree(lua_State* L, sol::table& module)
{
    module["StringStyle"] = 
        sol::table::create_with(
            L,
            "PLAIN_STRING",   PLAIN_STRING,
            "SINGLE_QUOTED",  SINGLE_QUOTED,
            "DOUBLE_QUOTED",  DOUBLE_QUOTED,
            "LITERAL_STRING", LITERAL_STRING,
            "FOLDED_STRING",  FOLDED_STRING);

    module.new_usertype<ValueNode>(
        "ValueNode",
        "new", sol::no_constructor,
        "isValid", &ValueNode::isValid,
        "nodeType", &ValueNode::nodeType,
        "isScalar", &ValueNode::isScalar,
        "isString", &ValueNode::isString,
        "isMapping", &ValueNode::isMapping,
        "isListing", &ValueNode::isListing,
        "toInt", &ValueNode::toInt,
        "toNumber", &ValueNode::toDouble,
        "toBool", &ValueNode::toBool,
        "toString", &ValueNode::toString,
        "toMapping", [](ValueNode* self) -> MappingPtr { return self->toMapping(); },
        "toListing", [](ValueNode* self) -> ListingPtr { return self->toListing(); },
        "hasLineInfo", &ValueNode::hasLineInfo,
        "line", &ValueNode::line,
        "column", &ValueNode::column
        );

    module.new_usertype<Mapping>(
        "Mapping",
        sol::base_classes, sol::bases<ValueNode>(),
        "new", sol::factories([]() -> MappingPtr { return new Mapping; }),
        "empty", &Mapping::empty,
        "size", &Mapping::size,
        "clear", &Mapping::clear,
        "setFlowStyle", &Mapping::setFlowStyle,
        "isFlowStyle", &Mapping::isFlowStyle,
        "setFloatingNumberFormat", &Mapping::setFloatingNumberFormat,
        "floatingNumberFormat", &Mapping::floatingNumberFormat,
        "setKeyQuoteStyle", &Mapping::setKeyQuoteStyle,
        "find", [](Mapping* self, const char* key) -> ValueNodePtr { return self->find(key); },
        "findMapping", [](Mapping* self, const char* key) -> MappingPtr { return self->findMapping(key); },
        "findListing", [](Mapping* self, const char* key) -> ListingPtr { return self->findListing(key); },
        "insert", sol::overload(
            [](Mapping* self, const char* key, ValueNode* node) { self->insert(key, node); },
            [](Mapping* self, Mapping* other) { self->insert(other); }),
        "openMapping", [](Mapping* self, const char* key) -> MappingPtr {
            return self->openMapping(key); },
        "openFlowStyleMapping", [](Mapping* self, const char* key) -> MappingPtr {
            return self->openFlowStyleMapping(key); },
        "createMapping", [](Mapping* self, const char* key) -> MappingPtr {
            return self->createMapping(key); },
        "createFlowStyleMapping", [](Mapping* self, const char* key) -> MappingPtr {
            return self->createFlowStyleMapping(key); },
        "openListing", [](Mapping* self, const char* key) -> ListingPtr {
            return self->openListing(key); },
        "openFlowStyleListing", [](Mapping* self, const char* key) -> ListingPtr {
            return self->openFlowStyleListing(key); },
        "createListing", [](Mapping* self, const char* key) -> ListingPtr {
            return self->createListing(key); },
        "createFlowStyleListing", [](Mapping* self, const char* key) -> ListingPtr {
            return self->createFlowStyleListing(key); },
        "remove", &Mapping::remove
        );

    module.new_usertype<Listing>(
        "Listing",
        sol::base_classes, sol::bases<ValueNode>(),
        "new", sol::factories([]() -> ListingPtr { return new Listing; }),
        "empty", &Listing::empty,
        "size", &Listing::size,
        "clear", &Listing::clear,
        "reserve", &Listing::reserve,
        "setFlowStyle", &Listing::setFlowStyle,
        "isFlowStyle", &Listing::isFlowStyle,
        "setFloatingNumberFormat", &Listing::setFloatingNumberFormat,
        "floatingNumberFormat", &Listing::floatingNumberFormat,
        "front", [](Listing* self) -> ValueNodePtr { return self->front(); },
        "back", [](Listing* self) -> ValueNodePtr { return self->back(); },
        "at", [](Listing* self, int i) -> ValueNodePtr { return self->at(i); },
        "write", sol::overload(
            [](Listing* self, int i, int value) { self->write(i, value); },
            [](Listing* self, int i, const char* value) { self->write(i, value); }),
        "newMapping", [](Listing* self) -> MappingPtr { return self->newMapping(); },
        "append", sol::overload(
            [](Listing* self, ValueNode* node) { self->append(node); },
            [](Listing* self, double number) { self->append(number); },
            [](Listing* self, const char* value) { self->append(value); })
        );
}

}
