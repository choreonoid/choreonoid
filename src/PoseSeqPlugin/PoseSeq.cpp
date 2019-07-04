/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "PoseSeq.h"
#include "PronunSymbol.h"
#include "LipSyncTranslator.h"
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <fstream>

using namespace std;
using namespace cnoid;

PoseRef::PoseRef(PoseSeq* owner, PoseUnitPtr poseUnit, double time)
    : poseUnit_(poseUnit)
{
    this->owner = owner;
    time_ = time;
    maxTransitionTime_ = 0.0;
}

PoseSeq::PoseSeq()
{
    
}


PoseSeq::PoseSeq(const PoseSeq& org)
    : PoseUnit(org)
{
    iterator current = begin();
    for(const_iterator it = org.begin(); it != org.end(); ++it){
        current = copyElement(current, it);
    }
}


PoseSeq::~PoseSeq()
{

}


void PoseSeq::setName(const std::string& name)
{
    name_ = name;
}


PoseSeq::iterator PoseSeq::copyElement(iterator seekpos, const_iterator org, double offset)
{
    bool inserted = false;

    iterator pos = seekpos;

    const string& name = org->name();
    if(!name.empty()){
        PoseUnitMap::iterator p = poseUnitMap.find(name);
        if(p != poseUnitMap.end()){
            pos = insert(seekpos, org->time() + offset, org->name());
            pos->setMaxTransitionTime(org->maxTransitionTime());
            inserted = true;
        }
    }

    if(!inserted){
        PoseUnitPtr orgPoseUnit = org->poseUnit();
        if(orgPoseUnit){
            PoseUnitPtr copiedUnit(orgPoseUnit->duplicate());
            pos = insert(seekpos, org->time() + offset, copiedUnit);
            pos->setMaxTransitionTime(org->maxTransitionTime());
        }
    }

    return seekpos;
}


PoseUnit* PoseSeq::duplicate()
{
    return new PoseSeq(*this);
}


bool PoseSeq::load(const std::string& filename, const BodyPtr body)                   
{
    errorMessage_.clear();

    /// \todo emit signals
    refs.clear();
    poseUnitMap.clear();
    
    YAMLReader parser;
    if(parser.load(filename)){
        const Mapping& archive = *parser.document()->toMapping();
        restore(archive, body);
        setName(archive["name"]);
        return true;
    }

    return false;
}


bool PoseSeq::restore(const Mapping& archive, const BodyPtr body)
{
    setTargetBodyName(archive.get("targetBody", body->name()));
                                   
    const Listing& refs = *archive.findListing("refs");

    if(!refs.isValid()){
        return false;
    }
        
    PoseSeq::iterator current = begin();
    
    for(int i=0; i < refs.size(); ++i){
        const Mapping& ref = *refs[i].toMapping();

        bool isInserted = false;

        double time = ref["time"].toDouble();
        
        const ValueNode& referred = ref["refer"];

        if(referred.isScalar()){
            const string& name = referred;
            if(!name.empty()){
                current = insert(current, time, name);
                isInserted = true;
            }
        } else if(referred.isMapping()){
            const Mapping& mReferred = *referred.toMapping();
            const string& type = mReferred["type"];
            PoseUnitPtr poseUnit;
            if(type == "Pose"){
                poseUnit = new Pose();
            } else if(type == "PronunSymbol"){
                poseUnit = new PronunSymbol();
            }
            /*
              else if(type == "PoseSeq"){
              poseUnit = createLocalPoseSeq();
              }
            */
            if(poseUnit && poseUnit->restore(mReferred, body)){
                poseUnit->name_ = mReferred["name"];
                current = insert(current, time, poseUnit);
                isInserted = true;
            }
        }

        if(isInserted){
            current->setMaxTransitionTime(ref.get("maxTransitionTime", 0.0));
        }
    }

    return true;
}


bool PoseSeq::save(const std::string& filename, const BodyPtr body)
{
    YAMLWriter writer(filename);
    writer.setKeyOrderPreservationMode(true);
    storedNames.clear();
    MappingPtr archive = new Mapping();
    archive->setDoubleFormat("%.9g");
    store(*archive, body);
    writer.putComment("Body pose sequence format version 1.0 defined by cnoid-Robotics\n");
    writer.putNode(archive);
    return true;
}


void PoseSeq::store(Mapping& archive, const BodyPtr body) const
{
    archive.write("type", "PoseSeq");
    archive.write("name", name(), DOUBLE_QUOTED);
    archive.write("targetBody", body->name(), DOUBLE_QUOTED);

    Listing& refsNode = *archive.createListing("refs");
    
    for(PoseRefList::const_iterator p = refs.begin(); p != refs.end(); ++p){
        const PoseRef& ref = *p;
        MappingPtr refNode = refsNode.newMapping();
        refNode->write("time", ref.time());
        if(ref.maxTransitionTime() > 0.0){
            refNode->write("maxTransitionTime", ref.maxTransitionTime());
        }
        const string& name = ref.name();
        if((storedNames.find(name) == storedNames.end() /* && !ref.isExternalReference()*/) ||
           name.empty()){
            const_cast<PoseSeq*>(this)->storedNames.insert(name);
            MappingPtr childNode = refNode->createMapping("refer");
            ref.poseUnit()->store(*childNode, body);
        } else {
            refNode->write("refer", name, DOUBLE_QUOTED);
        }
    }
}


bool PoseSeq::exportTalkPluginFile(const std::string& filename)
{
    ofstream ofs(filename.c_str());
    double standardTransitionTiem = 0.135;

    double prevTime = 0.0;
    string prevSymbol;

    static const bool ENABLE_QUICK_MODE = true;
    
    if(ENABLE_QUICK_MODE){
        
        if(!refs.empty()){
            bool isInitial = true;
            for(PoseRefList::iterator p = refs.begin(); p != refs.end(); ++p){
                PoseRef& ref = *p;
                PronunSymbolPtr symbol = ref.get<PronunSymbol>();
                if(symbol && !symbol->name().empty()){
                    const double time = ref.time();
                    if(isInitial){
                        isInitial = false;
                    } else {
                        double durationTime = time - prevTime;
                        if(durationTime <= 0.6){
                            ofs << prevSymbol << " " << durationTime << "\n";
                        } else {
                            ofs << prevSymbol << " " << 0.6 << "\n";
                            ofs << "n" << " " << (durationTime - 0.6) << "\n";
                        }
                    }
                    prevTime = time;
                    prevSymbol = symbol->name();
                }
            }
            
            ofs << prevSymbol << " " << standardTransitionTiem << "\n";
        }
    } else {
        for(PoseRefList::iterator p = refs.begin(); p != refs.end(); ++p){
            PoseRef& ref = *p;
            PronunSymbolPtr symbol = ref.get<PronunSymbol>();
            if(symbol && !symbol->name().empty()){
                const double time = ref.time();
                double transitionTime = time - prevTime;
                ofs << symbol->name() << " " << transitionTime << "\n";
                prevTime = time;
            }
        }
    }

    ofs.close();

    return true;
}



bool PoseSeq::exportSeqFileForFaceController(const std::string& filename)
{
    LipSyncTranslator translator;
    translator.translatePoseSeq(*this);
    return translator.exportSeqFileForFaceController(filename);
}


PoseSeq::iterator PoseSeq::changeTime(iterator it, double newTime)
{
    iterator newpos;
    iterator insertpos = seek(it, newTime, true);
    iterator nextpos = it;
    nextpos++;
    
    if(insertpos == it || insertpos == nextpos){
        beginPoseModification(it);
        it->time_ = newTime;
        endPoseModification(it);
        newpos = it;
    } else {
        sigPoseRemoving_(it, true);
        PoseRef newRef(this, it->poseUnit(), newTime);
        newRef.setMaxTransitionTime(it->maxTransitionTime());
        refs.erase(it);
        newpos = refs.insert(insertpos, newRef);
        sigPoseInserted_(newpos, true);
    }

    return newpos;
}


PoseUnitPtr PoseSeq::find(const std::string& name)
{
    PoseUnitMap::iterator p = poseUnitMap.find(name);
    if(p != poseUnitMap.end()){
        return p->second;
    }

    return PoseUnitPtr(); // null pointer
}


PoseSeq::iterator PoseSeq::seek(PoseSeq::iterator current, double time, bool seekPosToInsert)
{
    if(current == refs.end()){
        if(current == refs.begin()){
            return current;
        }
        current--;
    }

    double ct = current->time();

    if(ct == time){
        if(seekPosToInsert){
            current++;
        }
    } else if(ct > time){
        while(current != refs.begin()){
            current--;
            ct = current->time();
            if(ct == time){
                if(seekPosToInsert){
                    current++;
                }
                break;
            } else if(ct < time){
                current++;
                break;
            }
        }
    } else {
        while(current != refs.end() && (current->time() < time)){
            current++;
        }
    }

    return current;
}
            

PoseSeq::iterator PoseSeq::insert(PoseSeq::iterator current, double time, PoseUnitPtr poseUnit)
{
    const string& name = poseUnit->name();


    if(!name.empty()){
        PoseUnitMap::iterator p = poseUnitMap.find(name);
        if(p != poseUnitMap.end()){
            return insertSub(current, time, p->second);
        } else {
            poseUnitMap.insert(make_pair(name, poseUnit));
        }
    }
    
    return insertSub(current, time, poseUnit);
}


PoseSeq::iterator PoseSeq::insertSub(PoseSeq::iterator current, double time, PoseUnitPtr poseUnit)
{
    PoseRef ref(this, poseUnit, time);

    poseUnit->owner = this;
    poseUnit->seqLocalReferenceCounter++;

    return insert(current, time, ref);
}


PoseSeq::iterator PoseSeq::insert(iterator current, double time, const std::string& name)
{
    if(name.empty()){
        return refs.end();
    }
    
    PoseUnitPtr punit = find(name);
    if(punit){
        return insertSub(current, time, punit);
    } else {
        PoseRef ref(this, PoseUnitPtr(), time);
        return insert(current, time, ref);
    }
}


PoseSeq::iterator PoseSeq::insert(iterator current, double time, PoseRef& ref)
{
    iterator it = seek(current, time);
    it = refs.insert(it, ref);
    sigPoseInserted_(it, false);
    return it;
}


PoseSeq::iterator PoseSeq::erase(iterator it)
{
    sigPoseRemoving_(it, false);

    PoseUnitPtr unit = it->poseUnit();
    if(unit){
        unit->seqLocalReferenceCounter--;
        if(unit->seqLocalReferenceCounter == 0){
            const string& name = unit->name();
            if(!name.empty()){
                poseUnitMap.erase(name);
            }
            unit->owner = 0;
        }
    }
    
    return refs.erase(it);
}


void PoseSeq::rename(iterator it, const std::string newName)
{
    // unref current shared pose unit
    PoseUnitPtr currentPoseUnit = it->poseUnit();
    if(currentPoseUnit){
        const string& currentName = currentPoseUnit->name();
        if(!currentName.empty()){
            if(--currentPoseUnit->seqLocalReferenceCounter == 0){
                poseUnitMap.erase(currentName);
            }
        }
    }
        
    PoseUnitPtr sharedPoseUnit = find(newName);
    if(sharedPoseUnit){
        it->poseUnit_ = sharedPoseUnit;
        sharedPoseUnit->seqLocalReferenceCounter++;
    } else {
        if(currentPoseUnit){
            PoseUnitPtr newUnit(currentPoseUnit->duplicate());
            newUnit->name_ = newName;
            newUnit->owner = this;
            newUnit->seqLocalReferenceCounter++;
            it->poseUnit_ = newUnit;
            if(!newName.empty()){
                poseUnitMap.insert(make_pair(newName, newUnit));
            }
        }
    }
}


void PoseSeq::getDomain(double& out_lower, double& out_upper)
{
    if(refs.empty()){
        out_lower = 0.0;
        out_upper = 0.0;
    } else {
        out_lower = refs.front().time();
        out_upper = refs.back().time();
    }
}


ConnectionSet PoseSeq::connectSignalSet
(const Signal<void(iterator, bool isMoving)>::Function& slotInserted,
 const Signal<void(iterator, bool isMoving)>::Function& slotRemoving,
 const Signal<void(iterator)>::Function& slotModified)
{
    ConnectionSet connections;

    connections.add(sigPoseInserted_.connect(slotInserted));
    connections.add(sigPoseRemoving_.connect(slotRemoving));
    connections.add(sigPoseModified_.connect(slotModified));

    return connections;
}


ConnectionSet PoseSeq::connectSignalSet
(const Signal<void(iterator, bool isMoving)>::Function& slotInserted,
 const Signal<void(iterator, bool isMoving)>::Function& slotRemoving,
 const Signal<void(iterator)>::Function& slotModifying,
 const Signal<void(iterator)>::Function& slotModified)
{
    ConnectionSet connections;

    connections.add(sigPoseInserted_.connect(slotInserted));
    connections.add(sigPoseRemoving_.connect(slotRemoving));
    connections.add(sigPoseModifying_.connect(slotModifying));
    connections.add(sigPoseModified_.connect(slotModified));

    return connections;
}
