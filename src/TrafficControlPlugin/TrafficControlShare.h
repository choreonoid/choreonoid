/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include "TCSimulatorItem.h"
#include "exportdecl.h"

class CNOID_EXPORT TrafficControlShare
{
public:

    bool initialize();

    void finalize();

    static TrafficControlShare* instance();

    TCSimulatorItem* getTcsInstance() const;

    void setTcsInstance(TCSimulatorItem* value);

    bool getTcsRunning() const;

    void setTcsRunning(bool value);

private:

    TrafficControlShare();

    ~TrafficControlShare();

private:

    static TrafficControlShare* _inst;
    TCSimulatorItem *_val=nullptr;
    bool _isRunning=false;

};
