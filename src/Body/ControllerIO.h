/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONTROLLER_IO_H
#define CNOID_BODY_CONTROLLER_IO_H

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ControllerIO
{
  public:
    virtual ~ControllerIO();

    virtual Body* body() = 0;
    virtual std::string optionString() const;
    std::vector<std::string> options() const;
    virtual std::ostream& os() const;
    virtual double timeStep() const;
    virtual double currentTime() const;
    
    // The following functions are only available in simulation
    virtual bool isNoDelayMode() const;
    virtual bool setNoDelayMode(bool on);

    //! \deprecated Use timeStep().
    double worldTimeStep() const { return timeStep(); };

  protected:
    std::string getIntegratedOptionString(const std::string& opt1, const std::string& opt2) const;
};

}

#endif
