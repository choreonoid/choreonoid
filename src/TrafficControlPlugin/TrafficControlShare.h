/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include "exportdecl.h"

class CNOID_EXPORT TrafficControlShare
{
public:

    bool initialize();

    void finalize();

    static TrafficControlShare* instance();

    TrafficControlSimulatorItem* getTcsInstance() const;

    void setTcsInstance(TrafficControlSimulatorItem* value);

    bool getTcsRunning() const;

    void setTcsRunning(bool value);

private:

    TrafficControlShare();

    ~TrafficControlShare();

private:

    static TrafficControlShare* _inst;
    TrafficControlSimulatorItem *_val=nullptr;
    bool _isRunning=false;

};
