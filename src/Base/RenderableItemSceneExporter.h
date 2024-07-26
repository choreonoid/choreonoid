#ifndef CNOID_BASE_RENDERABLE_ITEM_SCENE_EXPORTER_H
#define CNOID_BASE_RENDERABLE_ITEM_SCENE_EXPORTER_H

#include "ItemList.h"
#include "exportdecl.h"

namespace cnoid {

class MessageOut;

CNOID_EXPORT void showDialogToExportSelectedRenderableItemScene();
CNOID_EXPORT bool exportRenderableItemSceneAsObjFile(
    const ItemList<>& items, const std::string& filename, MessageOut* mout);

}

#endif
