#ifndef CNOID_BASE_DISTANCE_MEASUREMENT_DIALOG_H
#define CNOID_BASE_DISTANCE_MEASUREMENT_DIALOG_H

#include "Dialog.h"
#include "exportdecl.h"

namespace cnoid {

class DistanceMeasurementDialog : public Dialog
{
public:
    static DistanceMeasurementDialog* instance();

    ~DistanceMeasurementDialog();

    void show();

    class Impl;

protected:
    DistanceMeasurementDialog();
    
    virtual void onFinished(int result) override;

private:
    Impl* impl;
};

}

#endif
