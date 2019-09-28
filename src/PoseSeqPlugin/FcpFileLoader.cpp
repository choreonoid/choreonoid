/**
   @file
   FaceController Plugin File (*.poseset and *.poseseq) Loader
   @author Shin'ichiro Nakaoka
*/

#include "FcpFileLoader.h"
#include "PoseSeqItem.h"
#include <cnoid/RootItem>
#include <cnoid/AppConfig>
#include <cnoid/MainWindow>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/MenuManager>
#include <cnoid/Tokenizer>
#include <cnoid/stdx/filesystem>
#include <QFileDialog>
#include <map>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

struct FcPose
{
    vector<double> q;
    double transitionTime;
};
    
struct Part
{
    vector<int> jointIds;
    map<string, FcPose> poses;
};
typedef shared_ptr<Part> PartPtr;
    
vector<PartPtr> parts;


bool loadFaceControllerPoseSet(const string& filename)
{
    ostream& os = MessageView::mainInstance()->cout();
        
    parts.clear();
    
    ifstream ifs(filename.c_str());
    if(!ifs.is_open()){
        os << filename << " is not found.\n";
        return false;
    }

    os << "Loading " << filename << "..." << endl;

    int nLines = 0;
    try {
        CharSeparator<char> sep(" \t\r\n", 0, DROP_EMPTY_TOKENS);
        
        string line;
        while(getline(ifs, line)){
            ++nLines;

            // for a text with CRLF
            if(!line.empty() && line[line.length()-1] == '\r'){
                line = line.substr(0, line.length()-1);
            }
            
            Tokenizer<CharSeparator<char>> tokens(line, sep);
            auto it = tokens.begin();
            if(it != tokens.end()){
                if(*it == "*"){
                    ++it;
                    PartPtr part(new Part());
                    while(it != tokens.end()){
                        part->jointIds.push_back(std::stoi(*it++));
                    }
                    if(!part->jointIds.empty()){
                        os << "pose registration of the joint set ";
                        size_t i=0;
                        while(true){
                            os << part->jointIds[i++];
                            if(i == part->jointIds.size()){
                                break;
                            }
                            os << ", ";
                        }
                        os << endl;
                        parts.push_back(part);
                    }
                } else if(parts.empty()){
                    os << "Poses cannot be defined without a part definition";
                    os << " at line " << nLines << "." << endl;
                    break;
                } else {
                    PartPtr part = parts.back();
                    string label(*it++);
                    FcPose pose;
                    if(it != tokens.end()){
                        while(true){
                            double value = std::stod(*it++);
                            if(it == tokens.end()){
                                pose.transitionTime = value;
                                break;
                            }
                            pose.q.push_back(radian(value));
                        }
                    }
                    if(pose.q.size() != part->jointIds.size()){
                        os << "label \"" << label << "\" is not correctly defined";
                        os << " at line " << nLines << "." << endl;
                    } else {
                        part->poses[label] = pose;
                        os << label << " ";
                        for(size_t i=0; i < pose.q.size(); ++i){
                            os << pose.q[i] << ", ";
                        }
                        os << " : " << pose.transitionTime;
                        os << endl;
                    }
                }
            }
        }
        if(!parts.empty()){
            cout << endl;
        }
    }
    catch(const std::invalid_argument& ex){
        os << ex.what() << " at line " << nLines << endl;
        parts.clear();
    }

    return !parts.empty();
}


PoseSeqItemPtr loadFaceControllerPoseSeq(const string& filename)
{
    ostream& os = MessageView::mainInstance()->cout();
        
    ifstream ifs(filename.c_str());
    if(!ifs.is_open()){
        os << filename + "is not found" << endl;
        return 0;
    }

    os << "Loading " << filename << "..." << endl;

    PoseSeqItemPtr item = new PoseSeqItem();
    stdx::filesystem::path fpath(filename);
    item->setName(fpath.stem().string());
    PoseSeqPtr seq = item->poseSeq();

    int nLines = 0;
        
    try {
        CharSeparator<char> sep(" ,\t\r\n", 0, KEEP_EMPTY_TOKENS);
            
        int nPoses = 0;
        string line;
        while(getline(ifs, line)){
            nLines++;
            // for a text with CRLF
            if(!line.empty() && line[line.length()-1] == '\r'){
                line = line.substr(0, line.length()-1);
            }
            Tokenizer<CharSeparator<char>> tokens(line, sep);
                
            auto it = tokens.begin();
            if(it != tokens.end()){
                if(*it != "#"){
                    PoseSeq::iterator poseIter = seq->begin();
                    double time = std::stod(*it++);
                    int numParts = parts.size();
                    bool poseAdded = false;
                    for(int i=0; it != tokens.end(); ++i, ++it){
                        if(i == numParts){
                            os << "line at time " << time << " contains parts more than defined ones." << endl;
                            break;
                        }
                        Part& part = *parts[i];
                        string label(*it);
                        
                        if(!label.empty()){
                            map<string,FcPose>::iterator p = part.poses.find(label);
                            if(p == part.poses.end()){
                                os << "label \"" << label << "\" is not defined";
                                os << " at line " << nLines << "." << endl;
                            } else {
                                const FcPose& fcPose = p->second;
                                PosePtr pose(new Pose());
                                for(size_t j=0; j < part.jointIds.size(); ++j){
                                    pose->setJointPosition(part.jointIds[j], fcPose.q[j]);
                                }
                                poseIter = seq->insert(poseIter, time, pose);
                                poseIter->setMaxTransitionTime(fcPose.transitionTime);
                                poseAdded = true;
                            }
                        }
                    }
                    if(poseAdded){
                        nPoses++;
                    }
                }
            }
        }
        os << "A pose sequence with " << nPoses << " poses has been loaded." << endl;
    }
    catch(const std::invalid_argument& ex){
        os << "FaceController : " << ex.what() << endl;
        os << " at line " << nLines << "." << endl;
        item = 0;
    }
        
    return item;
}
    

void invokeFaceControllerPatternFileImportDialog()
{
    /// \todo The function by the following code shoulde be provided by ItemManger

    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Choose poseset file"));
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    QStringList filters;
    filters << _("FaceController poseset files (*.poseset)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    string currentFolder;
    if(AppConfig::archive()->read("currentFileDialogDirectory", currentFolder)){
        dialog.setDirectory(currentFolder.c_str());
    }

    string posesetFile;

    if(dialog.exec()){
        posesetFile = dialog.selectedFiles().front().toStdString();
        dialog.setWindowTitle(_("Choose poseseq files"));

        QStringList filters;
        filters << _("FaceController poseseq files (*.poseseq)");
        filters << _("Any files (*)");
        dialog.setNameFilters(filters);

        if(dialog.exec()){
            dialog.hide();
            MessageView::mainInstance()->flush();

            ItemTreeView* itv = ItemTreeView::mainInstance(); 
            Item* parentItem = itv->selectedItem<Item>();
            if(!parentItem){
                parentItem = itv->rootItem();
            }
            QStringList poseseqFiles = dialog.selectedFiles();
            AppConfig::archive()->write(
                "currentFileDialogDirectory",
                dialog.directory().absolutePath().toStdString(),
                DOUBLE_QUOTED);

            if(loadFaceControllerPoseSet(posesetFile)){
                for(int i=0; i < poseseqFiles.size(); ++i){
                    PoseSeqItemPtr item = loadFaceControllerPoseSeq(poseseqFiles[i].toStdString());
                    if(item){
                        parentItem->addChildItem(item);
                    }
                }
            }
        }
    }
    dialog.hide();
}
}


void cnoid::initializeFcpFileLoader(ExtensionManager& ext)
{
    MenuManager& mm = ext.menuManager();
    mm.setPath("/File/Import ...");
    mm.addItem(_("FaceController Plugin Pattern Files"))
        ->sigTriggered().connect(invokeFaceControllerPatternFileImportDialog);
}
