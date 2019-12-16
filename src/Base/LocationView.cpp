#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class LocationView::Impl : public PositionWidget
{
public:
    Impl(QWidget* parent);
};

}


void LocationView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LocationView>(
        "LocationView", N_("Location"), ViewManager::SINGLE_OPTIONAL);
}


LocationView::LocationView()
{
    setDefaultLayoutArea(View::CENTER);
    impl = new Impl(this);
    auto vbox = new QVBoxLayout;
    vbox->addWidget(impl);
    setLayout(vbox);
}


LocationView::Impl::Impl(QWidget* parent)
    : PositionWidget(parent)
{

}


LocationView::~LocationView()
{
    delete impl;
}


void LocationView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->setOptionMenu(menuManager);
}


bool LocationView::storeState(Archive& archive)
{
    impl->storeState(archive);
    return true;
}


bool LocationView::restoreState(const Archive& archive)
{
    impl->restoreState(archive);
    return true;
}
