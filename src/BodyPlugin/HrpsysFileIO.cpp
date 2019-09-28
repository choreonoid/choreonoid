/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "HrpsysFileIO.h"
#include "BodyMotionItem.h"
#include "BodyItem.h"
#include "KinematicFaultChecker.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionUtil>
#include <cnoid/ZMPSeq>
#include <cnoid/Config>
#include <cnoid/Tokenizer>
#include <cnoid/stdx/filesystem>
#include <QMessageBox>
#include <fmt/format.h>
#include <boost/iostreams/filtering_stream.hpp>
#ifndef _WINDOWS
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#endif
#include <fstream>
#include <list>
#include <vector>
#include <map>
#include <regex>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace {

map<string,int> labelToTypeMap;

class HrpsysLogLoader
{
public:

    enum { NONE = 0,
           JOINT_POS, JOINT_VEL, JOINT_TORQUE,
           FORCE, TORQUE, ACC, OMEGA,
           ZMP, WAIST, RPY,
           NUM_DATA_TYPES };
    
    struct Element {
        Element(){
            type = NONE;
            axis = 0;
            index = 0;
        }
        int type;
        int axis;
        int index;
    };
    
    vector<Element> elements;
    
    int numComponents[NUM_DATA_TYPES];
    
    std::list< std::vector<double> > frames;

    regex labelPattern;
    
    HrpsysLogLoader()
        : labelPattern("^(JA|JV|TQ|F|M|A|W|zmp|waist|R|P|Y)([XYZRP]?)(\\d*)$")
    {
        if(labelToTypeMap.empty()){
            labelToTypeMap["JA"] = JOINT_POS;
            labelToTypeMap["JV"] = JOINT_VEL;
            labelToTypeMap["TQ"] = JOINT_TORQUE;
            labelToTypeMap["F"] = FORCE;
            labelToTypeMap["M"] = TORQUE;
            labelToTypeMap["A"] = ACC;
            labelToTypeMap["W"] = OMEGA;
            labelToTypeMap["zmp"] = ZMP;
            labelToTypeMap["waist"] = WAIST;
            labelToTypeMap["R"] = RPY;
            labelToTypeMap["P"] = RPY;
            labelToTypeMap["Y"] = RPY;
        }
    }

    bool loadLogFile(BodyMotionItem* item, const std::string& filename, std::ostream& os)
    {
        boost::iostreams::filtering_istream is;

#ifndef _WINDOWS
        string ext = filesystem::path(filename).extension().string();
        if(ext == ".gz"){
            is.push(boost::iostreams::gzip_decompressor());
        } else if(ext == ".bz2"){
            is.push(boost::iostreams::bzip2_decompressor());
        }
#endif
        ifstream ifs(filename.c_str());
        if(!ifs){
            os << format(_("\"{}\" cannot be opened."), filename) << endl;
            return false;
        }
        is.push(ifs);
        
        elements.clear();
        frames.clear();
        string line;

        regex header("(^\\s*%)(.*$)");
        smatch match; 
        while(getline(is, line)){
            if(regex_match(line, match, header)){
                if(match.size() == 3){
                    readHeader(match[2].str());
                    break;
                }
            }
        }
        if(elements.empty()){
            return false;
        }

        const size_t numElements = elements.size();

        Tokenizer<CharSeparator<char>> tokens(CharSeparator<char>(" \t\r\n"));

        while(getline(is, line)){
            tokens.assign(line);
            auto it = tokens.begin();
            if(it != tokens.end()){
                frames.push_back(vector<double>(numElements));
                vector<double>& frame = frames.back();
                size_t i;
                for(i=0; (i < numElements) && (it != tokens.end()); ++i, ++it){
                    frame[i] = std::stod(*it);
                }
                if(i < numElements /* || it != tokens.end() */ ){
                    os << format(_("\"{}\" contains different size columns."), filename) << endl;
                    return false;
                }
            }
        }

        const size_t numFrames = frames.size();

        auto motion = item->motion();
        motion->setDimension(numFrames, numComponents[JOINT_POS], 0);
        motion->setFrameRate(200);

        auto qseq = item->motion()->jointPosSeq();
        auto zmpseq = getOrCreateZMPSeq(*item->motion());

        std::list<std::vector<double>>::iterator p = frames.begin();
        
        for(size_t i=0; i < numFrames; ++i){
            vector<double>& frame = *p++;
            MultiValueSeq::Frame q = qseq->frame(i);
            for(size_t j=0; j < numElements; ++j){
                Element& e = elements[j];
                switch(e.type){
                case JOINT_POS:
                    q[e.index] = frame[j];
                    break;
                case ZMP:
                    (*zmpseq)[i][e.axis] = frame[j];
                    break;
                }
            }
        }

        return true;
    }

    void readHeader(const string& line)
    {
        for(int i=0; i < NUM_DATA_TYPES; ++i){
            numComponents[i] = 0;
        }

        smatch match;
        int waistIndex = 0;
        Tokenizer<CharSeparator<char>> tokens(line, CharSeparator<char>(" \t\r\n"));

        for(auto it = tokens.begin(); it != tokens.end(); ++it){
            Element element;
            if(regex_match(*it, match, labelPattern)){
                map<string,int>::iterator p = labelToTypeMap.find(match.str(1));
                if(p != labelToTypeMap.end()){
                    element.type = p->second;
                    const string& axisString = match.str(2);
                    if(!axisString.empty()){
                        if(element.type != WAIST || waistIndex < 3){
                            switch(axisString[0]){
                            case 'X': element.axis = 0; break;
                            case 'Y': element.axis = 1; break;
                            case 'Z': element.axis = 2; break;
                            }
                        } else {
                            switch(axisString[0]){
                            case 'R': element.axis = 0; break;
                            case 'P': element.axis = 1; break;
                            case 'Y': element.axis = 2; break;
                            }
                        }
                    }
                    const string& indexString = match.str(3);
                    if(!indexString.empty()){
                        element.index = std::stoi(indexString);
                    }
                    if(element.type == WAIST){
                        waistIndex++;
                    }
                }
            }
            elements.push_back(element);
            numComponents[element.type] += 1;
        }
    }
};


bool confirm(const std::string& message)
{
    return (QMessageBox::warning(
                0, _("Warning"), message.c_str(),
                QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok) == QMessageBox::Ok);
}


bool loadLogFile(BodyMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem)
{
    HrpsysLogLoader loader;
    return loader.loadLogFile(item, filename, os);
}


bool importHrpsysSeqFileSet(BodyMotionItem* item, const std::string& filename, std::ostream& os)
{
    if(loadHrpsysSeqFileSet(*item->motion(), filename, os)){
        return true;
    }
    return false;
}

    
bool exportHrpsysSeqFileSet(BodyMotionItem* item, const std::string& filename, std::ostream& os)
{
    double frameRate = item->motion()->frameRate();
    if(frameRate != 200.0){
        if(!confirm(
               format(
                   _("The frame rate of a body motion exported as HRPSYS files should be standard value 200, "
                     "but the frame rate of \"{0}\" is {1}. The exported data may cause a problem.\n\n"
                     "Do you continue to export ?"),
                   item->name(), frameRate))){
            return false;
        }
    }
    
    BodyPtr body;
    BodyItem* bodyItem = item->findOwnerItem<BodyItem>();
    if(bodyItem){
        body = bodyItem->body();
        KinematicFaultChecker* checker = KinematicFaultChecker::instance();
        int numFaults = checker->checkFaults(bodyItem, item, os);
        if(numFaults > 0){
            bool result;
            if(numFaults == 1){
                result = confirm(_("A fault has been detected. Please check the report in the MessageView.\n\n"
                                   "Do you continue to export ?"));
            } else {
                result = confirm(
                    format(
                        _("{} faults have been detected. Please check the report in the MessageView.\n\n"
                          "Do you continue to export ?"), numFaults));
            }
            
            if(!result){
                return false;
            }
        }
    }
    
    if(!getZMPSeq(*item->motion())){
        if(!confirm(_("There is no ZMP data. Do you continue to export ?"))){
            return false;
        }
    }
    
    return saveHrpsysSeqFileSet(*item->motion(), body, filename, os);
}

} // namespace {


void cnoid::initializeHrpsysFileIO(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    
    im.addLoaderAndSaver<BodyMotionItem>(
        _("HRPSYS Sequence File Set"), "HRPSYS-SEQ-FILE-SET", "pos;vel;acc;hip;waist;gsens;zmp",
        [](BodyMotionItem* item, const std::string& filename, std::ostream& os, Item* /* parentItem */){
            return importHrpsysSeqFileSet(item, filename, os);
        },
        [](BodyMotionItem* item, const std::string& filename, std::ostream& os, Item* /* parentItem */){
            return exportHrpsysSeqFileSet(item, filename, os);
        },
        ItemManager::PRIORITY_CONVERSION);

    im.addLoader<BodyMotionItem>(
        _("HRPSYS Log File"), "HRPSYS-LOG", "log;log.gz;log.bz2", loadLogFile, ItemManager::PRIORITY_CONVERSION);
}
