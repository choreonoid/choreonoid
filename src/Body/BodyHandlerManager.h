#ifndef CNOID_BODY_BODY_HANDLER_MANAGER_H
#define CNOID_BODY_BODY_HANDLER_MANAGER_H

#include <iosfwd>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Body;
class BodyHandler;
class BodyHandlerManagerImpl;

class CNOID_EXPORT BodyHandlerManager
{
public:
    BodyHandlerManager();
    void setMessageSink(std::ostream& os);
    bool loadBodyHandler(Body* body, const std::string& filename);

private:
    BodyHandlerManagerImpl* impl;
};

}

#endif
