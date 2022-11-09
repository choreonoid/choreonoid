#include "BodyKeyPoseSelectionDialog.h"
#include "BodyKeyPose.h"
#include <cnoid/LinkDeviceTreeWidget>
#include <cnoid/BodyItem>
#include <cnoid/Separator>
#include <cnoid/ValueTree>
#include <QBoxLayout>
#include <QLabel>
#include "gettext.h"

using namespace std;
using namespace cnoid;


BodyKeyPoseSelectionDialog::BodyKeyPoseSelectionDialog
(LinkDeviceTreeWidget* linkTreeWidget, std::function<PoseSeqItem*()> targetPoseSeqItemFunc)
    : Dialog(linkTreeWidget),
      linkTreeWidget(linkTreeWidget),
      targetPoseSeqItemFunc(targetPoseSeqItemFunc)
{
    setWindowTitle(_("Specific Key Pose Selection"));

    auto vbox = new QVBoxLayout;

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Time range:")));
    startTimeSpin.setDecimals(2);
    startTimeSpin.setRange(0.00, 999.99);
    startTimeSpin.setSingleStep(0.01);
    hbox->addWidget(&startTimeSpin);

    hbox->addWidget(new QLabel("-"));

    endTimeSpin.setDecimals(2);
    endTimeSpin.setRange(0.00, 999.99);
    endTimeSpin.setSingleStep(0.01);
    endTimeSpin.setValue(100.0);
    hbox->addWidget(&endTimeSpin);
    hbox->addWidget(new QLabel(_("[s]")));
    hbox->addStretch();
    
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Target key poses:")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    allKeyPosesRadio.setText(_("All"));
    hbox->addWidget(&allKeyPosesRadio);
    includeSelectedPartsRadio.setText(_("Including selected parts"));
    includeSelectedPartsRadio.setChecked(true);
    hbox->addWidget(&includeSelectedPartsRadio);
    matchSelectedPartsRadio.setText(_("Matching selected parts"));
    hbox->addWidget(&matchSelectedPartsRadio);
    hbox->addStretch();
    
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);

    hbox = new QHBoxLayout;
    hbox->addStretch();

    auto clearButton = new PushButton(_("&Clear"));
    clearButton->sigClicked().connect([this](){ clearSelection(); });
    hbox->addWidget(clearButton);

    auto selectButton = new PushButton(_("&Select"));
    selectButton->sigClicked().connect([this](){ doSelection(); });
    hbox->addWidget(selectButton);
    
    vbox->addLayout(hbox);

    setLayout(vbox);
}


void BodyKeyPoseSelectionDialog::clearSelection()
{
    auto poseSeqItem = targetPoseSeqItemFunc();
    if(poseSeqItem){
        poseSeqItem->clearPoseSelection(true);
    }
}


void BodyKeyPoseSelectionDialog::doSelection()
{
    auto poseSeqItem = targetPoseSeqItemFunc();
    if(!poseSeqItem || !poseSeqItem->targetBodyItem()){
        return;
    }

    auto seq = poseSeqItem->poseSeq();
    auto body = poseSeqItem->targetBodyItem()->body();
    
    bool changed = poseSeqItem->clearPoseSelection();
    
    const vector<int> selected = linkTreeWidget->selectedBodyPartLinkIndices();
    const double t0 = startTimeSpin.value();
    const double t1 = endTimeSpin.value();

    auto it = seq->seek(seq->begin(), t0);

    while(it != seq->end() && it->time() <= t1){
        if(auto pose = it->get<BodyKeyPose>()){
            bool doSelect = false;
            if(allKeyPosesRadio.isChecked()){
                doSelect = true;
            } else {
                bool included = false;
                bool unmatched = false;
                for(size_t i=0; i < selected.size(); ++i){
                    int linkIndex = selected[i];
                    if(pose->isJointValid(body->link(linkIndex)->jointId()) || pose->ikLinkInfo(linkIndex)){
                        included = true;
                        if(includeSelectedPartsRadio.isChecked()){
                            break;
                        }
                    } else {
                        unmatched = true;
                        if(matchSelectedPartsRadio.isChecked()){
                            break;
                        }
                    }
                }
                if(matchSelectedPartsRadio.isChecked()){
                    if(!unmatched){
                        for(int jointId = 0; jointId < pose->numJoints(); ++jointId){
                            if(pose->isJointValid(jointId)){
                                bool isValidJointSelected = false;
                                for(auto& selectedLinkIndex : selected){
                                    if(body->link(selectedLinkIndex)->jointId() == jointId){
                                        isValidJointSelected = true;
                                        break;
                                    }
                                }
                                if(!isValidJointSelected){
                                    unmatched = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(!unmatched){
                        for(auto it = pose->ikLinkBegin(); it != pose->ikLinkEnd(); ++it){
                            auto linkIndex = it->first;
                            bool isValidIkLinkSelected = false;
                            for(auto& selectedLinkIndex : selected){
                                if(selectedLinkIndex == linkIndex){
                                    isValidIkLinkSelected = true;
                                    break;
                                }
                            }
                            if(!isValidIkLinkSelected){
                                unmatched = true;
                                break;
                            }
                        }
                    }
                }
                if(includeSelectedPartsRadio.isChecked()){
                    doSelect = included;
                } else {
                    doSelect = !unmatched;
                }
            }
            if(doSelect){
                poseSeqItem->selectPose(it);
                changed = true;
            }
        }
        ++it;
    }

    if(changed){
        poseSeqItem->notifyPoseSelectionChange();
    }
}


void BodyKeyPoseSelectionDialog::storeState(Mapping* archive)
{
    auto timeRangeNode = archive->openFlowStyleListing("selection_time_range");
    timeRangeNode->append(startTimeSpin.value());
    timeRangeNode->append(endTimeSpin.value());

    const char* type = nullptr;
    if(allKeyPosesRadio.isChecked()){
        type = "all";
    } else if(includeSelectedPartsRadio.isChecked()){
        type = "include_selected_parts";
    } else {
        type = "match_selected_parts";
    }
    if(type){
        archive->write("selection_type", type);
    }
}


void BodyKeyPoseSelectionDialog::restoreState(const Mapping* archive)
{
    auto timeRangeNode = archive->findListing("selection_time_range");
    if(timeRangeNode->isValid() && timeRangeNode->size() == 2){
        startTimeSpin.setValue(timeRangeNode->at(0)->toDouble());
        endTimeSpin.setValue(timeRangeNode->at(1)->toDouble());
    }
    string type;
    if(archive->read("selection_type", type)){
        if(type == "all"){
            allKeyPosesRadio.setChecked(true);
        } else if(type == "include_selected_parts"){
            includeSelectedPartsRadio.setChecked(true);
        } else if(type == "match_selected_parts"){
            matchSelectedPartsRadio.setChecked(true);
        }
    }
}
