#include "AGXBody.h"

namespace cnoid {

AGXBody::AGXBody(Body & orgBody) : SimulationBody(new Body(orgBody)) {


}

}
