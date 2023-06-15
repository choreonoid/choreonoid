#ifndef CNOID_BASE_DISTANCE_MEASUREMENT_DIALOG_H
#define CNOID_BASE_DISTANCE_MEASUREMENT_DIALOG_H

#include "Dialog.h"
#include "exportdecl.h"

namespace cnoid {

class Item;
class DistanceMeasurementItem;

class CNOID_EXPORT DistanceMeasurementDialog : public Dialog
{
public:
    static DistanceMeasurementDialog* instance();

    ~DistanceMeasurementDialog();

    void setDefaultItemToAddNewMeasurementItem(Item* item);

    /**
       \param item A target DistanceMeasurementItem instance. If nullptr is given,
       a new instance is created and used internally in the dialog.
    */
    void show(DistanceMeasurementItem* item = nullptr);

    class Impl;

protected:
    DistanceMeasurementDialog();
    
    virtual void onFinished(int result) override;

private:
    Impl* impl;
};

}

#endif
