/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter{

class MonitorView : public cnoid::View
{
public:

    MonitorView();

    void write(const std::string& msg);

    void writeln(const std::string& msg);

    void clear();


protected:
    virtual void onActivated();
    virtual void onDeactivated();
private:
    MonitorForm* _Monitor;
};
}
