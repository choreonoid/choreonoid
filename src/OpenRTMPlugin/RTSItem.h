/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_ITEM_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_ITEM_H_INCLUDED

#include <cnoid/Item>
//#include <cnoid/ItemManager>
//#include <cnoid/Archive>
#include <boost/shared_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

class RTSystemItemImpl;

/*!
 * @brief This is the RTSystem item.
 */
class CNOID_EXPORT RTSystemItem : public Item
{
public:
    RTSystemItem();
    RTSystemItem(const RTSystemItem& org);
    virtual ~RTSystemItem();
    static void initialize(ExtensionManager* ext);

    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    //virtual bool store(Archive& archive);
    //virtual bool restore(const Archive& archive);

private:
    RTSystemItemImpl* impl;
};

typedef ref_ptr<RTSystemItem> RTSystemItemPtr;
}

#endif
