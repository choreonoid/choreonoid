/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_COMPETITION_PLUGIN_COMPETITOR_MARKER_H
#define CNOID_COMPETITION_PLUGIN_COMPETITOR_MARKER_H

#include <cnoid/MarkerDevice>

namespace cnoid {

class CompetitorMarker : public MarkerDevice
{
public:
    CompetitorMarker();
    CompetitorMarker(const CompetitorMarker& org, bool copyStateOnly = false);
    
    virtual const char* typeName();    
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func);
};

typedef ref_ptr<CompetitorMarker> CompetitorMarkerPtr;

}

#endif
