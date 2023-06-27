#ifndef CNOID_BODY_PLUGIN_CONTROLLER_LOG_ITEM_H
#define CNOID_BODY_PLUGIN_CONTROLLER_LOG_ITEM_H

#include <cnoid/ReferencedObjectSeqItem>

namespace cnoid {

[[deprecated("Use ReferencedObjectSeqItem")]]
typedef ReferencedObjectSeqItem ControllerLogItem;
[[deprecated("Use ReferencedObjectSeqItemPtr")]]
typedef ref_ptr<ReferencedObjectSeqItem> ControllerLogItemPtr;

}

#endif
