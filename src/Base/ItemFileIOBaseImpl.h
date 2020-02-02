#ifndef CNOID_BASE_ITEM_FILE_IO_BASE_IMPL_H
#define CNOID_BASE_ITEM_FILE_IO_BASE_IMPL_H

#include "ItemFileIO.h"
#include <functional>
#include <iosfwd>

namespace cnoid {

class ItemManager;
class ItemManagerImpl;
class MessageView;

class ItemFileIOBase::Impl
{
    int api;
    std::string formatId;
    std::vector<std::string> formatIdAlias;
    std::string caption;
    std::vector<std::string> extensions;
    std::function<std::string()> extensionFunction;
    ItemFileIOBase::InterfaceLevel interfaceLevel;
    Item* parentItem;
    ItemFileIOBase::InvocationType invocationType;
    std::ostream* os;
    MessageView* mv;
    std::string typeId;

    // This variable actualy points a instance of the ClassInfo class defined in ItemManager.cpp
    weak_ref_ptr<Referenced> classInfo; 

    friend class ItemFileIOBase;
    friend class ItemManager;
    friend class ItemManagerImpl;
};

}

#endif
