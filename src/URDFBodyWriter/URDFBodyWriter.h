#ifndef CNOID_URDF_BODY_WRITER_URDF_BODY_WRITER_H
#define CNOID_URDF_BODY_WRITER_URDF_BODY_WRITER_H

#include <cnoid/Body>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT URDFBodyWriter
{
public:
    URDFBodyWriter();
    ~URDFBodyWriter();
    void setMessageSink(std::ostream& os);
    void setMessageSinkStdErr();
    void setVerbose(bool _on);

    bool writeBody(Body* body, const std::string& filename);
    void writeBodyAsString(Body *body, std::string &outstr);

    void setUseXacro(bool _on);
    void setAddGeometry(bool _on);
    void setAddOffset(bool _on);

    void setExportDevices(bool _on);

    void setRobotName(const std::string &_name);
    void setMeshFilePrefix(const std::string &_pref);
    void setMeshURLPrefix(const std::string &_pref);
private:
    class Impl;
    Impl* impl;
};

};  // namespace cnoid

#endif
