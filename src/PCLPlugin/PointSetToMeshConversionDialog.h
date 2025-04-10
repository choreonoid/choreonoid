#ifndef CNOID_PCL_PLUGIN_POINT_SET_TO_MESH_CONVERSION_DIALOG_H
#define CNOID_PCL_PLUGIN_POINT_SET_TO_MESH_CONVERSION_DIALOG_H

#include <cnoid/Dialog>
#include "exportdecl.h"

namespace cnoid
{

class ExtensionManager;
class PointSetItem;

class CNOID_EXPORT PointSetToMeshConversionDialog : public Dialog
{
public:
    static void initializeClass(ExtensionManager* ext);
    static PointSetToMeshConversionDialog* instance();
    static void setAutoCloseAfterConversion(bool on);

    void show(PointSetItem* targetItem = nullptr);

protected:
    virtual void onFinished(int result) override;

private:
    PointSetToMeshConversionDialog();
    ~PointSetToMeshConversionDialog();

    class Impl;
    Impl* impl;;
};

}

#endif
