/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"
#include <fmt/format.h>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using namespace Multicopter;
using fmt::format;

void
UtilityImpl::printMessage(const std::string& msg, bool sync)
{
    if( sync == true ){
        callSynchronously(std::bind(&UtilityImpl::printMessageImpl, msg));
    }
    else{
        callLater(std::bind(&UtilityImpl::printMessageImpl, msg));
    }
}

void
UtilityImpl::printErrorMessage(const std::string& msg, bool sync)
{
    if( sync == true ){
        callSynchronously(std::bind(&UtilityImpl::printErrorMessageImpl, msg));

    }
    else{
        callLater(std::bind(&UtilityImpl::printErrorMessageImpl, msg));
    }
}


void
UtilityImpl::printWarningMessage(const std::string& msg,  bool sync)
{
    if( sync == true ){
        callSynchronously(std::bind(&UtilityImpl::printWarningMessageImpl, msg));
    }
    else{
        callLater(std::bind(&UtilityImpl::printWarningMessageImpl, msg));
    }
}


Mapping*
UtilityImpl::mapping(cnoid::Mapping* mapping, const std::string& key)
{
    if( mapping == nullptr ){
        return nullptr;        
    }
    ValueNode* node = mapping->find(key);
    if( node->isValid() == false ){
        return nullptr;
    }
    if( node->isMapping() == false ){
        return nullptr;
    }
    return node->toMapping();
}

bool
UtilityImpl::exist(cnoid::Mapping* mapping, const std::string& key)
{
    ValueNode* node = mapping->find(key);
    if( node->isValid() == false || node->isScalar() == false ){
        return false;
    }
    return true;
}

bool
UtilityImpl::value(Mapping* mapping, const string& key, bool& val)
{
    ValueNode* node = mapping->find(key);
    if( node->isValid() == false || node->isScalar() == false ){
        return false;
    }
    try{
        val = node->toBool();
    }
    catch(...){
        return false;
    }
    return true;
}

bool
UtilityImpl::value(Mapping* mapping, const string& key, double& val)
{
    ValueNode* node = mapping->find(key);
    if( node->isValid() == false || node->isScalar() == false ){
        return false;
    }
    try{
        val = node->toDouble();
    }
    catch(...){
        return false;
    }
    return true;
}

bool
UtilityImpl::value(cnoid::Mapping* mapping, const std::string& key, std::vector<double>& valAry)
{
    ValueNode* node = mapping->find(key);
    
    if( node->isValid() == false || node->isListing() == false ){
        return false;
    }
        
    Listing* valAryAttr = node->toListing();
    
    int size = valAryAttr->size();
    valAry.reserve(size);
    for(int i=0 ; i<size  ; ++i){
        valAry.push_back(valAryAttr->get(i).toDouble());
    }
    return true;    
}

bool
UtilityImpl::value(cnoid::Mapping* mapping, const std::string& key, std::vector<int>& valAry)
{
    ValueNode* node = mapping->find(key);

    if( node->isValid() == false || node->isListing() == false ){
        return false;
    }

    Listing* valAryAttr = node->toListing();

    int size = valAryAttr->size();
    valAry.reserve(size);
    for(int i=0 ; i<size  ; ++i){
        valAry.push_back(valAryAttr->get(i).toInt());
    }
    return true;
}

bool
UtilityImpl::value(cnoid::Mapping* mapping, const std::string& key, std::vector<bool>& valAry)
{
    ValueNode* node = mapping->find(key);
    
    if( node->isValid() == false || node->isListing() == false ){
        return false;
    }
        
    Listing* valAryAttr = node->toListing();
    
    int size = valAryAttr->size();
    valAry.reserve(size);
    for(int i=0 ; i<size  ; ++i){
        valAry.push_back(valAryAttr->get(i).toBool());
    }
    return true;        
}


bool
UtilityImpl::value(Mapping* mapping, const string& key, Eigen::Vector3d& val)
{
    vector<double> valAry;
    bool ret = value(mapping, key, valAry);
    if( ret == false ){
        return false;
    }
    if( valAry.size() != 3 ){
        return false;
    }

    for(int i=0 ; i<3  ; ++i){
        val[i] = valAry[i];
    }
    return true;
}

bool
UtilityImpl::value(Mapping* mapping, const string& key, Eigen::Matrix3d& val)
{
    vector<double> valAry;
    bool ret = value(mapping, key, valAry);
    if( ret == false ){
        return false;
    }
    
    if( valAry.size() != 9 ){
        return false;
    }
    
    val = Matrix3d(valAry.data());
    return true;
}

bool
UtilityImpl::value(Mapping* mapping, const string& key, int& val)
{
    ValueNode* node = mapping->find(key);
    if( node->isValid() == false || node->isScalar() == false ){
        return false;
    }
    try{
        val = node->toInt();
    }
    catch(...){
        return false;
    }
    return true;
}

void
UtilityImpl::splitStringArray(const string& line, std::vector<string>& retAry)
{
    const string RF = "\r";
    retAry.clear();
    retAry.reserve(10);
    Tokenizer<EscapedListSeparator<char>> tokens(line, EscapedListSeparator<char>('\\', ',','\"'));
    for(auto it=begin(tokens) ; it!=end(tokens) ; ++it){
        if( (*it).empty() == false ){
            if( *it != RF ){
                retAry.push_back(*it);
            }
        }
    }
}

void
UtilityImpl::removeSelfNodeFromParent(SgNode* self)
{
    vector<SgObject*> parents;
    for(SgObject::const_parentIter it = self->parentBegin() ; it != self->parentEnd() ; ++it){
        parents.push_back(*it);
    }

    for(auto p : parents){
        SgGroup* grp= dynamic_cast<SgGroup*>(p);
        if( grp != nullptr ){
            grp->removeChild(self);
        }
    }
}

list<string>
UtilityImpl::fileList(const string& dir, const string& ext, const bool withExt)
{
    list<string> retAry;
    QDir qdir(QString::fromStdString(dir));
    QFileInfoList fileInfoList = qdir.entryInfoList(QStringList(QString("*.")+QString::fromStdString(ext)), QDir::Filter::Files);
    if( withExt == true ){
        for(auto& f : fileInfoList){
            retAry.push_back(f.fileName().toStdString());
        }
    }
    else{
        for(auto& f : fileInfoList){
            retAry.push_back(f.baseName().toStdString());
        }
    }
    return retAry;
}

void
UtilityImpl::removeFiles(const std::string& dir)
{
    list<string> retAry;
    QDir qdir(QString::fromStdString(dir));
    QFileInfoList entList = qdir.entryInfoList();
    for(auto& ent : entList){
        QFile::remove(ent.filePath());
    }
}

bool
UtilityImpl::stringArray2Data(const std::vector<std::string>& strAry, std::string& tag, std::vector<double>& valAry)
{
    valAry.clear();
    if( strAry.size() < 2 ){
        return false;
    }
    int valSize = strAry.size()-1;
    valAry.reserve(valSize);
    try{
        tag = strAry[0];
        for(int i=0 ; i<valSize ; ++i){            
            valAry.push_back(stod(strAry[i+1]));
        }
    }
    catch(...){
        return false;
    }

    return true;
}

bool
UtilityImpl::stringArray2Data(const std::vector<std::string>& strAry, std::string& tag, std::vector<int>& valAry)
{
    valAry.clear();
    if( strAry.size() < 2 ){
        return false;
    }
    int valSize = strAry.size()-1;
    valAry.reserve(valSize);
    try{
        tag = strAry[0];
        for(int i=0 ; i<valSize ; ++i){
            valAry.push_back(stoi(strAry[i+1]));
        }
    }
    catch(...){
        return false;
    }

    return true;
}

bool
UtilityImpl::stringArray2Data(const std::vector<std::string>& strAry, std::string& tag, int& val)
{
    vector<int> valAry;
    bool ret = stringArray2Data(strAry, tag, valAry);
    if( ret == false ){
        return false;
    }
    if( valAry.size() != 1 ){
        return false;
    }
    val = valAry.front();
    return true;
}

void
UtilityImpl::linkArray(const cnoid::Body* body, std::vector<cnoid::Link*>& linkAry)
{
    linkAry.clear();
    size_t linkNum = body->numLinks();
    linkAry.reserve(linkNum);
    for(size_t i=0 ; i<linkNum ; ++i){
        linkAry.push_back(body->link(i));
    }
}

string
UtilityImpl::toString(const vector<int>& ary)
{
    string ret;
    for(size_t i=0 ; i<ary.size() ; ++i){
        ret += to_string(ary[i]);
        if( i < ary.size() - 1 ){
            ret += string(",");
        }
    }
    return ret;
}

string
UtilityImpl::toString(const std::vector<double>& ary, const std::string& fmt)
{
    vector<string> strAry;
    for(size_t i=0 ; i<ary.size() ; ++i){
        strAry.push_back(format(fmt, ary[i]));
    }
    return toString(strAry);
}

string
UtilityImpl::toString(const std::vector<std::string>& ary)
{
    string ret;
    const string DELM = ",";
    for(size_t i=0 ; i<ary.size() ; ++i){
        if( i == ary.size()-1){
            ret += ary[i];
        }
        else{
            ret += (ary[i] + DELM);
        }
    }
    return ret;
}



bool
UtilityImpl::toIntegerArray(const string& str, vector<int>& ary)
{
    ary.clear();

    Tokenizer<CharSeparator<char>> tokens(str, CharSeparator<char>(","));

    vector<string> tmpAry;
    tmpAry.reserve(10);
    try{
        for(auto& token : tokens){
            tmpAry.push_back(token.data());
        }
    }
    catch(...){
        return false;
    }

    vector<int> retAry;
    retAry.reserve(tmpAry.size());
    try{
        for(auto& tmp : tmpAry){
            retAry.push_back(std::stoi(tmp));
        }
    }
    catch(...){
        return false;
    }

    ary = retAry;

    return true;
}

bool
UtilityImpl::toFloatArray(const string& str, vector<double>& ary)
{
    ary.clear();

    Tokenizer<CharSeparator<char>> tokens(str, CharSeparator<char>(","));

    vector<double> tmpAry;
    tmpAry.reserve(10);

    try{
        for(auto& token : tokens){
            tmpAry.push_back(std::stod(token.data()));
        }
    }
    catch(...){
        return false;
    }

    ary = tmpAry;

    return true;
}

bool
UtilityImpl::toVector3d(const string& str, Eigen::Vector3d& vec)
{
    vector<double> ary;
    bool ret = toFloatArray(str, ary);
    if( ret == false ){
        return false;
    }
    if( ary.size() != 3 ){
        return false;
    }
    vec = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(ary.data(), ary.size());
    return true;
}

void
UtilityImpl::printMessageImpl(const string& msg)
{
    MessageView::instance()->putln(MessageView::NORMAL, msg);
}

void
UtilityImpl::printErrorMessageImpl(const string& msg)
{
    MessageView::instance()->putln(MessageView::ERROR, msg);    
}

void
UtilityImpl::printWarningMessageImpl(const string& msg)
{
    MessageView::instance()->putln(MessageView::WARNING, msg);    
}

void
UtilityImpl::printSomethingWrongAtBody(const string& key, const string& bodyName)
{
    UtilityImpl::printWarningMessage(format(_("[{0}] tag is false in body[{1}]."), key, bodyName));
}

void
UtilityImpl::printSomethingWrongAtLink(const string& key, const string& linkName)
{    
    UtilityImpl::printWarningMessage(format(_("[{0}] tag is missing, or wrong parameter in link[{1}]."), key, linkName));
}

void
UtilityImpl::printSomethingWrongAtRotor(const string& key)
{
    UtilityImpl::printWarningMessage(format(_("[{}] tag is missing, or wrong parameter in RotorDevice."), key));
}


bool
UtilityImpl::findLinkAttribute(const std::string& bodyName, const std::string& linkName, const std::map<const cnoid::Link*, LinkAttribute>& linkAttrMap, LinkAttribute& attr)
{
    for(auto& linkAttr : linkAttrMap){
        auto link = linkAttr.first;
        auto body = link->body();
        if( bodyName == body->name() && linkName == link->name() ){
            attr = linkAttr.second;
            return true;
        }
    }
    return false;
}

void
UtilityImpl::linkAttributeFromBodyMappingInfo(Body* body, std::map<const Link*, LinkAttribute>& linkAttrMap, bool createDevice)
{
    LinkManager* lnkMgr = LinkManager::instance();

    Mapping* mapInfo = body->info();

    linkAttrMap.clear();

    vector<Link*> linkAry;
    lnkMgr->linkArray(body, linkAry);

    double cutoffDist=-1;
    double normMidVal=0;

    Mapping* bodyAttr=mapping(mapInfo,YAML_BODY_TAG);
    if(bodyAttr == nullptr){
        bool flg=exist(mapInfo,YAML_BODY_TAG);
        if(flg==false){
            return;
        }
    }else{
        if( value(bodyAttr, YAML_BODY_CUTOFF_DISTANCE_TAG, cutoffDist) == false ){
            cutoffDist=-1;
        }
        if( value(bodyAttr, YAML_BODY_NORM_MIDDLE_VALUE_TAG, normMidVal) == false ){
            normMidVal=0;
        }
    }

    Link* root =body->rootLink();
    int effectMode=0;
    if(root->isFixedJoint()){
        effectMode=2;
    }else{
        DeviceList<RotorDevice> rotorDevAry = body->devices();
        if(rotorDevAry.size()>0)effectMode=1;
        else effectMode=0;
    }

    for(auto it =begin(linkAry);it!=end(linkAry);++it){
        string linkName = (*it)->name();
         LinkAttribute attr;

         attr.setEffectMode(effectMode);
        if(effectMode==2){
            linkAttrMap[*it] = attr;
            continue;
        }

        attr.setCutoffDistance(cutoffDist);
        attr.setNormMiddleValue(normMidVal);
        Mapping* linkMapInfo=(*it)->info();

        Mapping* fluidLinkMap=mapping(linkMapInfo,YAML_LINK_TAG);

        if(fluidLinkMap==nullptr){
            linkAttrMap[*it] = attr;
            continue;
        }

        vector<bool> forceApplyFlgAry = {true, true, true, true};
        vector<bool> linkForceApplyFlgAry;

        if( value(fluidLinkMap,YAML_LINK_APPLYFORCE_TAG, linkForceApplyFlgAry) == true ){
            if( linkForceApplyFlgAry.size() != 4 ){
                linkForceApplyFlgAry = forceApplyFlgAry;
            }
        }
        else{
            linkForceApplyFlgAry = forceApplyFlgAry;
        }
        attr.setLinkForceApplyFlgAry(linkForceApplyFlgAry);

        double tmpVal;
        if( value(fluidLinkMap, YAML_LINK_DENSITY_TAG, tmpVal) == true ){
            attr.setDensity(tmpVal);
        }
        else{
            printSomethingWrongAtLink(YAML_LINK_DENSITY_TAG, linkName);
            linkAttrMap[*it] = attr;
            continue;
        }

        Eigen::Vector3d tmpVec;
        if( value(fluidLinkMap, YAML_LINK_BUOYANCY_CENTER_TAG, tmpVec) == true ){
            attr.setCenterOfBuoyancy(tmpVec);
        }
        else{
        }

        if( value(fluidLinkMap, YAML_LINK_ADDITIONAL_MASS_TAG, tmpVal) == true ){
            attr.setAdditionalMassCoef(tmpVal);
        }else{
            printSomethingWrongAtLink(YAML_LINK_ADDITIONAL_MASS_TAG, linkName);
            linkAttrMap[*it] = attr;
            continue;
        }

        Eigen::Matrix3d tmpMtx;
        if( value(fluidLinkMap, YAML_LINK_ADDITIONAL_INERTIA_TAG, tmpMtx) == true ){
            attr.setAdditionalInertiaMatrix(tmpMtx);

        }else{
            printSomethingWrongAtLink(YAML_LINK_ADDITIONAL_INERTIA_TAG, linkName);
            linkAttrMap[*it] = attr;
            continue;
        }

        attr.checkDone();
        linkAttrMap[*it] = attr;
    }
}
