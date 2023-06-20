#include "GeometryMeasurementTracker.h"

using namespace std;
using namespace cnoid;

namespace {

vector<std::function<GeometryMeasurementTracker*(Item* item)>> factories;

}


void GeometryMeasurementTracker::registerItemTrackerFactory
(std::function<GeometryMeasurementTracker*(Item* item)> trackerFactory)
{
    factories.push_back(trackerFactory);
}


bool GeometryMeasurementTracker::checkIfMeasureable(Item* item)
{
    GeometryMeasurementTrackerPtr tracker = createTracker(item);
    return static_cast<bool>(tracker);
}


GeometryMeasurementTracker* GeometryMeasurementTracker::createTracker(Item* item)
{
    for(auto& factory : factories){
        if(auto tracker = factory(item)){
            return tracker;
        }
    }
    return nullptr;
}


int GeometryMeasurementTracker::getNumSubEntries()
{
    return 0;
}


std::string GeometryMeasurementTracker::getSubEntryName(int /* index */)
{
    return std::string();
}


int GeometryMeasurementTracker::findSubEntryIndex(const std::string& /* name */)
{
    return -1;
}


int GeometryMeasurementTracker::getCurrentSubEntryIndex()
{
    return -1;
}


bool GeometryMeasurementTracker::setCurrentSubEntry(int /* index */)
{
    return false;
}
