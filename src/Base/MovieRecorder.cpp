/**
   @author Shin'ichiro Nakaoka
*/

#include "MovieRecorder.h"
#include "ExtensionManager.h"
#include "ToolBar.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

MovieRecorder* movieRecorder = 0;

class MovieRecorderBar : public ToolBar
{
public:
    MovieRecorderBar();


};

}

namespace cnoid {

class MovieRecorderImpl
{
public:
    MovieRecorderBar* toolBar;

    MovieRecorderImpl(ExtensionManager* ext);
};

}


void MovieRecorder::initialize(ExtensionManager* ext)
{
    if(!movieRecorder){
        movieRecorder = new MovieRecorder(ext);
    }
}


MovieRecorder* MovieRecorder::instance()
{
    return movieRecorder;
}


MovieRecorder::MovieRecorder(ExtensionManager* ext)
{
    impl = new MovieRecorderImpl(ext);
}


MovieRecorderImpl::MovieRecorderImpl(ExtensionManager* ext)
{
    toolBar = new MovieRecorderBar();
    ext->addToolBar(toolBar);
}


MovieRecorderBar::MovieRecorderBar()
    : ToolBar(N_("MovieRecorderBar"))
{
    addButton("P", _("Start Playback & Recording"));
    addButton("R", _("Start Recording"));

    addButton(QIcon(":/Base/icons/setup.png"), _("Open the setup dialog"));
}
