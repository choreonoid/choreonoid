/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace Multicopter;


MonitorView::MonitorView()
{
    _Monitor = new MonitorForm();
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(_Monitor);
    setLayout(vbox);
    setDefaultLayoutArea(View::BOTTOM);
}

void
MonitorView::write(const string& msg)
{
    callLater(std::bind(&MonitorForm::write, _Monitor, msg));
}

void
MonitorView::writeln(const string& msg)
{
    callLater(std::bind(&MonitorForm::writeln, _Monitor, msg));
}

void
MonitorView::clear()
{
    callLater(std::bind(&MonitorForm::clear, _Monitor));
}


void MonitorView::onActivated()
{

}

void MonitorView::onDeactivated()
{

}
