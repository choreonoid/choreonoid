/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ABSTRACT_TEXT_ITEM_H
#define CNOID_BASE_ABSTRACT_TEXT_ITEM_H

#include "Item.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractTextItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    AbstractTextItem();
    AbstractTextItem(const AbstractTextItem& org);

    virtual const std::string& textFilename() const = 0;

protected:
    virtual ~AbstractTextItem();
};

typedef ref_ptr<AbstractTextItem> AbstractTextItemPtr;

}

#endif
