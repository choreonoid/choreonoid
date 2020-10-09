#ifndef CNOID_BODY_STD_BODY_WRITER_H
#define CNOID_BODY_STD_BODY_WRITER_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT StdBodyWriter
{
public:
    StdBodyWriter();

    bool writeBody(Body* body, const std::string& filename);

private:
    class Impl;
    Impl* impl;
};

}

#endif
