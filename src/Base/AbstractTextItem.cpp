/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "AbstractTextItem.h"
#include "ItemManager.h"

using namespace cnoid;


void AbstractTextItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<AbstractTextItem>();
}
    

AbstractTextItem::AbstractTextItem()
{

}


AbstractTextItem::AbstractTextItem(const AbstractTextItem& org)
    : Item(org)
{

}


AbstractTextItem::~AbstractTextItem()
{

}
