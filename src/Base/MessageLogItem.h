/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_BASE_MESSAGE_LOG_ITEM_H
#define CNOID_BASE_MESSAGE_LOG_ITEM_H

#include "AbstractTextItem.h"
#include "exportdecl.h"

namespace cnoid {

class MessageLogItemImpl;

class CNOID_EXPORT MessageLogItem : public AbstractTextItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    MessageLogItem();
    MessageLogItem(const MessageLogItem& org);
    virtual ~MessageLogItem();
    virtual const std::string& textFilename() const override;
    enum FileMode { APPEND = 0, OVERWRITE, N_FILE_MODES };

protected:
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    MessageLogItemImpl* impl;
};

typedef ref_ptr<MessageLogItem> MessageLogItemPtr;

}

#endif

