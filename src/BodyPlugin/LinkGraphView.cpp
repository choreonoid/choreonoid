#include "LinkGraphView.h"
#include <cnoid/ViewManager>
#include <cnoid/EigenUtil>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void LinkGraphView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LinkGraphView>(
        N_("LinkGraphView"), N_("Link Trajectories"));
}


LinkGraphView::LinkGraphView()
{
    static const char* xyzLabels[] = { "X", "Y", "Z" };
    static const char* rpyLabels[] = { "R", "P", "Y" };

    auto hbox = new QHBoxLayout;
    hbox->setSpacing(0);
    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    vbox->addStretch();
    setupElementToggleSet(vbox, xyzToggles, xyzLabels, true);
    setupElementToggleSet(vbox, rpyToggles, rpyLabels, false);
    vbox->addStretch();
    
    hbox->addLayout(vbox);
    hbox->addWidget(&graph, 1);
    
    setLayout(hbox);
}


void LinkGraphView::setupElementToggleSet
(QBoxLayout* box, ToggleToolButton toggles[], const char* labels[], bool isActive)
{
    for(int i=0; i < 3; ++i){
        toggles[i].setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        box->addWidget(&toggles[i]);
        toggles[i].setChecked(isActive);
        toggles[i].setText(labels[i]);

        toggleConnections.add(
            toggles[i].sigToggled().connect(
                [&](bool){ setupGraphWidget(); }));
    }
}


void LinkGraphView::addTrajectory(ItemInfo& info, Link* link, std::shared_ptr<BodyPositionSeq> seq)
{
    if(link->index() >= seq->numLinkPositionsHint()){
        return;
    }
    
    for(int i=0; i < 2; ++i){
        ToggleToolButton* toggles = (i == 0) ? xyzToggles : rpyToggles;
        for(int j=0; j < 3; ++j){
            if(toggles[j].isChecked()){
                GraphDataHandlerPtr handler(new GraphDataHandler);
                handler->setLabel(link->name());
                handler->setFrameProperties(seq->numFrames(), seq->frameRate());
                handler->setDataRequestCallback(
                    [this, &info, link, i, j](int frame, int size, double* out_values){
                        onDataRequest(info, link->index(), i, j, frame, size, out_values); });
                handler->setDataModifiedCallback(
                    [this, &info, link, i, j](int frame, int size, double* values){
                        onDataModified(info, link->index(), i, j, frame, size, values); });
                graph.addDataHandler(handler);
                info.handlers.push_back(handler);
            }
        }
    }
}


void LinkGraphView::onDataRequest
(ItemInfo& info, int linkIndex, int type, int axis, int frameIndex0, int size, double* out_values)
{
    for(int i=0; i < size; ++i){
        int frameIndex = i + frameIndex0;
        auto& frame = info.seq->frame(frameIndex);
        if(linkIndex < frame.numLinkPositions()){
            auto position = frame.linkPosition(linkIndex);
            if(type == 0){ // Translation
                out_values[i] = position.translation()[axis];
            } else { // Rotation (RPY)
                auto rpy = rpyFromRot(position.rotation().toRotationMatrix());
                out_values[i] = rpy[axis];
            }
        }
    }
}


void LinkGraphView::onDataModified
(ItemInfo& info, int linkIndex, int type, int axis, int frameIndex0, int size, double* values)
{
    for(int i=0; i < size; ++i){
        int frameIndex = i + frameIndex0;
        auto& frame = info.seq->frame(frameIndex);
        frame.allocate(std::max(linkIndex + 1, frame.numLinkPositions()), frame.numJointDisplacements());
        auto position = frame.linkPosition(linkIndex);
        if(type == 0){ // Translation
            position.translation()[axis] = values[i];
        } else { // Rotation (RPY)
            auto rotation = position.rotation();
            Vector3 rpy(rpyFromRot(rotation.toRotationMatrix()));
            rpy[axis] = values[i];
            rotation = rotFromRpy(rpy);
        }
    }
    
    info.connections.block();
    info.item->notifyUpdate();
    info.connections.unblock();
}


bool LinkGraphView::storeState(Archive& archive)
{
    if(!BodyPositionGraphViewBase::storeState(archive)){
        return false;
    }
    Listing& visibleElements = *archive.createFlowStyleListing("visible_elements");
    for(int i=0; i < 3; ++i){
        if(xyzToggles[i].isChecked()){
            visibleElements.append(i);
        }
    }
    for(int i=0; i < 3; ++i){
        if(rpyToggles[i].isChecked()){
            visibleElements.append(i+3);
        }
    }
    return true;
}


bool LinkGraphView::restoreState(const Archive& archive)
{
    if(!BodyPositionGraphViewBase::restoreState(archive)){
        return false;
    }
    const Listing& visibleElements = *archive.findListing("visible_elements");
    if(visibleElements.isValid()){
        auto block = toggleConnections.scopedBlock();
        for(int i=0; i < 3; ++i){
            xyzToggles[i].setChecked(false);
            rpyToggles[i].setChecked(false);
        }
        for(int i=0; i < visibleElements.size(); ++i){
            int index = visibleElements[i].toInt();
            if(index < 3){
                xyzToggles[index].setChecked(true);
            } else {
                rpyToggles[index-3].setChecked(true);
            }
        }
    }
    return true;
}
