#include "RenderableItemSceneExporter.h"
#include "MainWindow.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include "FileDialog.h"
#include <cnoid/SceneGraph>
#include <cnoid/ObjSceneWriter>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void cnoid::showDialogToExportSelectedRenderableItemScene()
{
    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Export the scene of selected items"));
    dialog.setViewMode(QFileDialog::List);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setLabelText(QFileDialog::Accept, _("Export"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    dialog.setNameFilter("OBJ file (*.obj)");
    dialog.updatePresetDirectories(true);
    if(dialog.exec() == QDialog::Accepted){
        auto mout = MessageOut::master();
        string filename = dialog.selectedFiles().value(0).toStdString();
        auto items = RootItem::instance()->selectedItems();
        if(items.empty()){
            mout->putErrorln(_("No selected items."));
        } else {
            exportRenderableItemSceneAsObjFile(items, filename, mout);
        }
    }
}


bool cnoid::exportRenderableItemSceneAsObjFile
(const ItemList<>& items, const std::string& filename, MessageOut* mout)
{
    SgGroupPtr scene = new SgGroup;
    vector<Item*> sceneItems;
    for(auto& item : items){
        if(auto renderableItem = dynamic_cast<RenderableItem*>(item.get())){
            scene->addChild(renderableItem->getScene());
            sceneItems.push_back(item.get());
        }
    }
    if(scene->empty()){
        mout->putWarningln(_("Scene to export is empty."));
        return false;
    }

    mout->put(formatR(_("Export the scene of the following items to \"{0}\":\n "), filename));
    for(int i=0; i < sceneItems.size(); ++i){
        mout->put(sceneItems[i]->displayName());
        if(i < sceneItems.size() - 1){
            mout->put(", ");
        }
    }
    mout->putln("");

    ObjSceneWriter sceneWriter;
    sceneWriter.setMessageSink(mout->cout());
    bool result = sceneWriter.writeScene(filename, scene);
    if(result){
        mout->putln(_("Completed!"));
    } else {
        mout->putErrorln(_("Failed."));
    }
    
    return result;
}
