#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>

namespace cnoid {

class AGXSimulatorItemImpl;
class AGXBody :  public SimulationBody{
public:
	AGXBody(Body& orgBody);

};

}

#endif