#ifndef CNOID_BODY_PLUGIN_BODY_ITEM_FILE_IO_H
#define CNOID_BODY_PLUGIN_BODY_ITEM_FILE_IO_H

#include "BodyItem.h"
#include <cnoid/ItemFileIO>
#include <cnoid/BodyLoader>
#include <cnoid/StdBodyWriter>
#include <QBoxLayout>
#include <QComboBox>
#include <QCheckBox>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyItemFileIoBase : public ItemFileIoBase<BodyItem>
{
protected:
    BodyItemFileIoBase(const char* format, int api);
    ~BodyItemFileIoBase();
    
    virtual QWidget* getOptionPanelForSaving(BodyItem* item) override;
    virtual void createOptionPanelForSaving() = 0;
    void addExtModelFileModeCombo(QBoxLayout* box);
    void addTransformIntegrationCheck(QBoxLayout* box);

    QVBoxLayout* optionVBox;
    QComboBox* extModelFileModeCombo;
    QCheckBox* transformIntegrationCheck;

private:
    QWidget* optionPanel;
};


class CNOID_EXPORT BodyItemBodyFileIO : public BodyItemFileIoBase
{
public:
    BodyItemBodyFileIO();

    StdBodyWriter* bodyWriter(){ return ensureBodyWriter(); }

protected:
    virtual bool load(BodyItem* item, const std::string& filename) override;
    StdBodyWriter* ensureBodyWriter();
    virtual void createOptionPanelForSaving() override;
    virtual void fetchOptionPanelForSaving() override;
    virtual bool save(BodyItem* item, const std::string& filename) override;

private:
    BodyLoader bodyLoader_;
    std::unique_ptr<StdBodyWriter> bodyWriter_;
};

}

#endif
