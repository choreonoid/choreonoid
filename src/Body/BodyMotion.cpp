#include "BodyMotion.h"
#include "Body.h"
#include "Link.h"
#include "ZMPSeq.h"
#include <cnoid/Vector3Seq>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


BodyMotion::BodyMotion()
    : AbstractSeq("CompositeSeq"),
      positionSeq_(new BodyPositionSeq),
      linkPosSeq_(new MultiSE3Seq),
      jointPosSeq_(new MultiValueSeq)
{
    setSeqContentName("BodyMotion");
    linkPosSeq_->setSeqContentName("MultiLinkPositionSeq");
    jointPosSeq_->setSeqContentName("MultiJointDisplacementSeq");
}


BodyMotion::BodyMotion(const BodyMotion& org)
    : AbstractSeq(org),
      positionSeq_(new BodyPositionSeq(*org.positionSeq_)),
      linkPosSeq_(new MultiSE3Seq(*org.linkPosSeq_)),
      jointPosSeq_(new MultiValueSeq(*org.jointPosSeq_))
{
    for(ExtraSeqMap::const_iterator p = org.extraSeqs.begin(); p != org.extraSeqs.end(); ++p){
        extraSeqs.insert(ExtraSeqMap::value_type(p->first, p->second->cloneSeq()));
    }
}


BodyMotion& BodyMotion::operator=(const BodyMotion& rhs)
{
    if(this != &rhs){
        AbstractSeq::operator=(rhs);
    }
    *positionSeq_ = *rhs.positionSeq_;
    *linkPosSeq_ = *rhs.linkPosSeq_;
    *jointPosSeq_ = *rhs.jointPosSeq_;

    //! \todo do copy instead of replacing the pointers to the cloned ones
    extraSeqs.clear();
    for(ExtraSeqMap::const_iterator p = rhs.extraSeqs.begin(); p != rhs.extraSeqs.end(); ++p){
        extraSeqs.insert(ExtraSeqMap::value_type(p->first, p->second->cloneSeq()));
    }
    sigExtraSeqsChanged_();

    return *this;
}


std::shared_ptr<AbstractSeq> BodyMotion::cloneSeq() const
{
    return std::make_shared<BodyMotion>(*this);
}


/*
  This function sets the number of joints
*/
void BodyMotion::setNumJoints(int numJoints, bool clearNewElements)
{
    jointPosSeq_->setNumParts(numJoints, clearNewElements);
}


double BodyMotion::getFrameRate() const
{
    return frameRate();
}


void BodyMotion::setFrameRate(double frameRate)
{
    positionSeq_->setFrameRate(frameRate);
    linkPosSeq_->setFrameRate(frameRate);
    jointPosSeq_->setFrameRate(frameRate);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setFrameRate(frameRate);
    }
}


int BodyMotion::getNumFrames() const
{
    return numFrames();
}


void BodyMotion::setNumFrames(int n, bool clearNewArea)
{
    positionSeq_->setNumFrames(n);
    linkPosSeq_->setNumFrames(n, clearNewArea);
    jointPosSeq_->setNumFrames(n, clearNewArea);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setNumFrames(n, clearNewArea);
    }
}


double BodyMotion::getOffsetTime() const
{
    return positionSeq_->offsetTime();
}


void BodyMotion::setOffsetTime(double time)
{
    positionSeq_->setOffsetTime(time);
    linkPosSeq_->setOffsetTime(time);
    jointPosSeq_->setOffsetTime(time);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setOffsetTime(time);
    }
}    


void BodyMotion::setDimension(int numFrames, int numJoints, int numLinks, bool clearNewArea)
{
    positionSeq_->setNumFrames(numFrames);
    linkPosSeq_->setDimension(numFrames, numLinks, clearNewArea);
    jointPosSeq_->setDimension(numFrames, numJoints, clearNewArea);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setNumFrames(numFrames, clearNewArea);
    }
}


void BodyMotion::setExtraSeq(const std::string& name, std::shared_ptr<AbstractSeq> seq)
{
    extraSeqs[name] = seq;
    sigExtraSeqsChanged_();
}


void BodyMotion::clearExtraSeq(const std::string& name)
{
    if(extraSeqs.erase(name) > 0){
        sigExtraSeqsChanged_();
    }
}


static BodyMotion& emptyMotion()
{
    static BodyMotion motion;
    return motion;
}


BodyMotion::Frame::Frame()
    : motion_(emptyMotion()),
      frame_(-1)
{

}


BodyMotion::ConstFrame::ConstFrame()
    : motion_(emptyMotion()),
      frame_(-1)
{

}

    
static void copyBodyStateToFrame(const Body& body, BodyMotion::Frame& frame)
{
    int frameIndex = frame.frame();
    BodyMotion& motion = frame.motion();
    int numLinks = std::min(body.numLinks(), motion.numLinks());
    int numJoints = std::min(body.numJoints(), motion.numJoints());

    auto& pframe = motion.positionSeq()->frame(frameIndex);
    pframe.allocate(numLinks, numJoints);

    auto lframe = motion.linkPosSeq()->frame(frameIndex);
    for(int i=0; i < numLinks; ++i){
        auto& p = lframe[i];
        p.set(body.link(i)->position());
        pframe.linkPosition(i).set(p);
    }

    auto jframe = motion.jointPosSeq()->frame(frameIndex);
    auto pjframe = pframe.jointDisplacements(); 
    for(int i=0; i < numJoints; ++i){
        auto q = body.joint(i)->q();
        jframe[i] = q;
        pjframe[i] = q;
    }
}
    

template<class FrameType>
static void copyFrameToBodyState(FrameType& frame, Body& body)
{
    auto& pframe = frame.motion().positionSeq()->frame(frame.frame());

    int numLinkPositions = pframe.numLinkPositions();
    if(numLinkPositions > 0){
        int n = std::min(body.numLinks(), numLinkPositions);
        for(int i=0; i < n; ++i){
            auto link = body.link(i);
            auto linkPosition = pframe.linkPosition(i);
            link->setTranslation(linkPosition.translation());
        }
    }

    int numJointDisplacements = pframe.numJointDisplacements();
    if(numJointDisplacements > 0){
        int n = std::min(body.numJoints(), numJointDisplacements);
        auto q = pframe.jointDisplacements();
        for(int i=0; i < n; ++i){
            body.joint(i)->q() = q[i];
        }
    }
}


BodyMotion::Frame cnoid::operator<<(BodyMotion::Frame frame, const Body& body)
{
    copyBodyStateToFrame(body, frame);
    return frame;
}


BodyMotion::Frame cnoid::operator>>(BodyMotion::Frame frame, Body& body)
{
    copyFrameToBodyState(frame, body);
    return frame;
}


BodyMotion::ConstFrame cnoid::operator>>(BodyMotion::ConstFrame frame, Body& body)
{
    copyFrameToBodyState(frame, body);
    return frame;
}


Body& cnoid::operator<<(Body& body, BodyMotion::Frame frame)
{
    copyFrameToBodyState(frame, body);
    return body;
}


Body& cnoid::operator<<(Body& body, BodyMotion::ConstFrame frame)
{
    copyFrameToBodyState(frame, body);
    return body;
}


const Body& cnoid::operator>>(const Body& body, BodyMotion::Frame frame)
{
    copyBodyStateToFrame(body, frame);
    return body;
}


bool BodyMotion::load(const std::string& filename, std::ostream& os)
{
    YAMLReader reader;
    reader.expectRegularMultiListing();
    bool result = false;

    try {
        result = readSeq(reader.loadDocument(filename)->toMapping(), os);
    } catch(const ValueNode::Exception& ex){
        os << ex.message();
    }

    return result;
}


bool BodyMotion::doReadSeq(const Mapping* archive, std::ostream& os)
{
    setDimension(0, 1, 1);

    bool loaded = false;
    
    double version;
    if(!archive->read("formatVersion", version)){
        version = 1.0;
    }
    if(version >= 4.0){
        os << format(_("Format version {} is not supported"), version) << endl;
        return false;
    }
    
    string type;
    const char* jointContent;
    const char* linkContent;
    std::function<string(Mapping* mapping)> readContent;
    if(version >= 2.0){
        type = seqType();
        if(version >= 3.0){
            jointContent = "MultiJointDisplacementSeq";
            linkContent = "MultiLinkPositionSeq";
        } else {
            jointContent = "JointDisplacement";
            linkContent = "LinkPosition";
        }
        readContent = [](Mapping* mapping){
            string content;
            mapping->read("content", content);
            return content;
        };
    } else {
        type = "BodyMotion";
        jointContent = "JointPosition";
        linkContent = "LinkPosition";
        readContent = [](Mapping* mapping){
            string content;
            if(!mapping->read("content", content)){
                mapping->read("purpose", content);
            }
            return content;
        };
    }
    
    if(archive->get<string>("type") == type){
        
        const Listing& components = *(*archive)["components"].toListing();
        
        for(int i=0; i < components.size(); ++i){

            // Merge the parameters of the parent node into the child (component) node
            MappingPtr component = components[i].toMapping()->cloneMapping();
            component->insert(archive);

            const ValueNode& typeNode = (*component)["type"];
            const string type = typeNode.toString();
            string content = readContent(component);
            
            if((type == "MultiSE3Seq" || (version < 2.0 && (type == "MultiSe3Seq" || type == "MultiAffine3Seq")))){
                if(content == linkContent){
                    loaded = linkPosSeq()->readSeq(component, os);
                    if(!loaded) break;
                    linkPosSeq()->setSeqContentName("MultiLinkPositionSeq");
                } else {
                    os << format(_("Unknown content \"{0}\" of type \"{1}\"."), content, type) << endl;
                }
            } else if(type == "MultiValueSeq"){
                if(content == jointContent){
                    loaded = jointPosSeq_->readSeq(component, os);
                    if(!loaded) break;
                    jointPosSeq_->setSeqContentName("MultiJointDisplacementSeq");
                } else {
                    os << format(_("Unknown content \"{0}\" of type \"{1}\"."), content, type) << endl;
                }
            } else if(type == "Vector3Seq") {
                if((version >= 3.0 && content == "ZMPSeq") ||
                   (version < 3.0 && content == "ZMP") ||
                   ((version < 2.0) && (content == "RelativeZMP" || content == "RelativeZmp"))){
                    auto zmpSeq = getOrCreateZMPSeq(*this);
                    loaded = zmpSeq->readSeq(component, os);
                    if(!loaded){
                        break;
                    }
                    if(version < 2.0){
                        zmpSeq->setRootRelative(content != "ZMP");
                    }
                } else {
                    //----------- user defined Vector3 data --------- 
                    auto userVec3 = getOrCreateExtraSeq<Vector3Seq>(content);
                    loaded = userVec3->readSeq(component, os);
                    if(!loaded){
                        break;
                    }
                }
            } else {
                os << format(_("Unknown type \"{}\"."), type) << endl;
            }
        }
    }
    
    if(!loaded){
        setDimension(0, 1, 1);
    }
    
    return loaded;
}


bool BodyMotion::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    double version = writer.getOrCreateInfo("formatVersion", 3.0);
    bool isVersion1 = version >= 1.0 && version < 2.0;
    if(version < 2.0){
        writer.putComment("Body motion data set format version 1.0 defined by Choreonoid\n");
        setSeqType("BodyMotion");
    }
    
    return AbstractSeq::doWriteSeq(
        writer,
        [&](){
            writer.setDoubleFormat("%.9g");

            if(additionalPartCallback) additionalPartCallback();

            writer.putKey("components");
            writer.setInfo("isComponent", true);

            writer.startListing();

            if(linkPosSeq_->numFrames() > 0){
                linkPosSeq_->writeSeq(writer);
            }

            if(jointPosSeq_->numFrames() > 0){
                string orgContentName;
                if(version < 3.0){
                    orgContentName = jointPosSeq_->seqContentName();
                    if(version >= 2.0){
                        jointPosSeq_->setSeqContentName("JointDisplacement");
                    } else {
                        jointPosSeq_->setSeqContentName("JointPosition");
                    }
                }
                jointPosSeq_->writeSeq(writer);
                if(version < 3.0){
                    jointPosSeq_->setSeqContentName(orgContentName);
                }
            }
            
            for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
                auto& seq = p->second;
                seq->writeSeq(writer);
            }
            
            writer.endListing();
        });

    if(isVersion1){
        setSeqType("CompositeSeq");
    }
}


bool BodyMotion::save(const std::string& filename, std::ostream& os)
{
    return save(filename, 0.0 /* latest */, os);
}
        

bool BodyMotion::save(const std::string& filename, double version, std::ostream& os)
{
    YAMLWriter writer(filename);
    if(version > 0.0){
        writer.setInfo("formatVersion", version);
    }

    writer.setMessageSink(os);

    return writeSeq(writer);
}
