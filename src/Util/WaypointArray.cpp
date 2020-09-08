#include "WaypointArray.h"

using namespace std;
using namespace cnoid;


WaypointArray::WaypointArray()
{

}


WaypointArray::WaypointArray(const WaypointArray& org)
{

}


WaypointArray::~WaypointArray()
{


}


void WaypointArray::insert(int index, Waypoint* point)
{
    int size = waypoints_.size();
    if(index > size){
        index = size;
    }
    waypoints_.insert(waypoints_.begin() + index, point);

    sigPointAdded_(index);
}


void WaypointArray::append(Waypoint* point)
{
    insert(waypoints_.size(), point);
}


bool WaypointArray::removeAt(int index)
{
    if(index >= waypoints_.size()){
        return false;
    }
    WaypointPtr point = waypoints_[index];
    waypoints_.erase(waypoints_.begin() + index);
    sigPointRemoved_(index, point);
    return true;
}
