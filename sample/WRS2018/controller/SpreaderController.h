/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Config>

namespace cnoid {

class CNOID_GENERAL_EXPORT SpreaderController : public SimpleController
{
    SimpleControllerIO* io;
    bool isToSpreadRequested;
    Link* flangeL;
    Link* flangeR;
    
public:
    static SpreaderController* instance();

    SpreaderController();
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
    void requestToSpread(bool on);
};

}
