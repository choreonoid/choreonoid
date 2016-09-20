/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "ScenarioItem.h"
#include <cnoid/PutPropertyFunction>
#include <cnoid/YAMLReader>
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;
}

void ScenarioItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<ScenarioItem>(N_("ScenarioItem"));
    ext->itemManager().addCreationPanel<ScenarioItem>();
}


ScenarioItem::ScenarioItem()
{

}


ScenarioItem::ScenarioItem(const ScenarioItem& org)
    : Item(org)
{

}


ScenarioItem::~ScenarioItem()
{

}


Item* ScenarioItem::doDuplicate() const
{
    return new ScenarioItem(*this);
}


void ScenarioItem::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool ScenarioItem::store(Archive& archive)
{
    return true;
}


bool ScenarioItem::restore(const Archive& archive)
{
    return true;
}
