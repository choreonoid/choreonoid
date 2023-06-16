#include "MarkerMotion.h"
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/EasyScanner>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


MarkerMotion::MarkerMotion()
{
    setSeqContentName("MarkerMotion");
    positionOffset_.setZero();
}


MarkerMotion::MarkerMotion(const MarkerMotion& org)
    : MultiVector3Seq(org),
      labelToIndexMap(org.labelToIndexMap),
      labels(org.labels),
      positionOffset_(org.positionOffset_)
{
    setSeqContentName("MarkerMotion");
}


MarkerMotion& MarkerMotion::operator=(const MarkerMotion& rhs)
{
    MultiVector3Seq::operator=(rhs);
    labelToIndexMap = rhs.labelToIndexMap;
    labels = rhs.labels;
    positionOffset_ = rhs.positionOffset_;
    return *this;
}


void MarkerMotion::copySeqProperties(const MarkerMotion& source)
{
    MultiVector3Seq::copySeqProperties(source);
    labelToIndexMap = source.labelToIndexMap;
    labels = source.labels;
    positionOffset_ = source.positionOffset_;
}


MarkerMotion::~MarkerMotion()
{

}


void MarkerMotion::clearLabels()
{
    labelToIndexMap.clear();
    labels.clear();
}


void MarkerMotion::setLabel(const std::string& label, int markerIndex)
{
    labelToIndexMap[label] = markerIndex;
    if(labels.size() <= markerIndex){
        labels.resize(markerIndex + 1);
    }
    labels[markerIndex] = label;
}


int MarkerMotion::partIndex(const std::string& label) const
{
    LabelToIndexMap::const_iterator p = labelToIndexMap.find(label);
    if(p != labelToIndexMap.end()){
        return p->second;
    }
    return -1;
}


const std::string& MarkerMotion::partLabel(int partIndex) const
{
    return labels[partIndex];
}


void MarkerMotion::setPositionOffset(const Vector3& offset)
{
    positionOffset_ = offset;
}


bool MarkerMotion::loadStdYAMLformat(const std::string& filename)
{
    bool result = false;
    clearSeqMessage();
    YAMLReader reader;
    reader.expectRegularMultiListing();
    try {
        result = readSeq(reader.loadDocument(filename)->toMapping());
    } catch(const ValueNode::Exception& ex){
        addSeqMessage(ex.message());
    }
    return result;
}


bool MarkerMotion::doReadSeq(const Mapping* archive, std::ostream& os)
{
    // read marker labels
    clearLabels();

    string content;
    bool isValidContent = archive->read("content", content) && (content == "MarkerMotion");

    if(!isValidContent){
        if(content.empty()){
            os << format("Content of {0} should be \"MarkerMotion\" but it is not specified.", seqType()) << endl;
        } else {
            os << format("Content \"{0}\" of {1} is different from the required content \"MarkerMotion\".",
                         content, seqType()) << endl;
        }
    } else {
        if(MultiVector3Seq::doReadSeq(archive, os)){
            labels = readSeqPartLabels(archive);
            for(size_t i=0; i < labels.size(); ++i){
                labelToIndexMap[labels[i]] = i;
            }
        }
    }
    return !labels.empty();
}


bool MarkerMotion::saveAsStdYAMLformat(const std::string& filename)
{
    YAMLWriter writer(filename);
    writer.setDoubleFormat("%.9g");
    writer.putComment("Marker motoin data format version 1.0\n");
    writer.startMapping();
    bool result = writeSeq(writer);
    writer.endMapping();
    return result;
}


bool MarkerMotion::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    // write marker labels
    if(writeSeqPartLabels(writer)){
        return MultiVector3Seq::doWriteSeq(writer, additionalPartCallback);
    }
    return false;
}
        

bool MarkerMotion::loadVPM(const std::string& filename)
{
    bool result = true;

    clearSeqMessage();
    
    try {
        labelToIndexMap.clear();
        labels.clear();
        setDimension(0, 0);
        setTimeStep(1.0);
        
        EasyScanner scanner(filename);

        typedef shared_ptr<vector<Vector3>> Vector3ArrayPtr;
        vector<Vector3ArrayPtr> allSegments;

        int minNumFrames = std::numeric_limits<int>::max();
        while(scanner.skipBlankLines()){
            scanner.checkStringEx("Segment:", "Segment must begin with \"Segment\"");
            const string label = scanner.readWordEx("The name of a segment is not specified");
            scanner.readLFex("Line feed is missing");

            Vector3ArrayPtr segment(new vector<Vector3>());
            int numFrames = readVpmSegment(scanner, *segment, label);
            if(numFrames == 0){
                result = false;
                break;
            } else {
                allSegments.push_back(segment);
                labelToIndexMap[label] = labels.size();
                labels.push_back(label);
            }
            minNumFrames = std::min(numFrames, minNumFrames);
        }

        int numSegments = allSegments.size();
        result = result && numSegments > 0 && minNumFrames > 0;

        if(!result){
            addSeqMessage("Error: the number of segments or the minimum number of frames is zero.");

        } else {
            setDimension(minNumFrames, numSegments);
            for(size_t i=0; i < numSegments; ++i){
                vector<Vector3>::iterator iter = allSegments[i]->begin();
                std::copy(iter, iter + minNumFrames, part(i).begin());
            }
        }
            
    } catch(EasyScanner::Exception& ex){
        addSeqMessage(ex.getFullMessage() + "\n");
        result = false;
    }

    return result;
}


int MarkerMotion::readVpmSegment
(EasyScanner& scanner, std::vector<Vector3>& segment, const std::string& label)
{
    int numFrames;
    double timeStep;

    scanner >> "Frames:" >> numFrames >> scanner.endl;
    scanner >> "Frame Time:" >> timeStep >> scanner.endl;
        
    scanner >> "XTRAN"   >> "YTRAN"   >> "ZTRAN"   >> "XROT" >> "YROT" >> "ZROT"
            >> "XSCALE"  >> "YSCALE"  >> "ZSCALE"  >> scanner.endl;
    scanner >> "INCHES"	 >> "INCHES"  >> "INCHES"  >> "DEGREES" >> "DEGREES" >> "DEGREES"
            >> "PERCENT" >> "PERCENT" >> "PERCENT" >> scanner.endl;

    segment.reserve(numFrames);

    double prevTimeStep = getTimeStep();
    if(prevTimeStep == 1.0){
        setTimeStep(timeStep); // initial setup
    } else if(timeStep != prevTimeStep){
        addSeqMessage(
            format("Warning: Frame time {0} of Segment {1} is different from {2} of the previous segment.\n",
                   timeStep, label, prevTimeStep));
    }

    double x, y, z;
    static const double inchToMeter = 0.0254;
    while(scanner.readDouble()){
        x = scanner.doubleValue;
        scanner >> y >> z;
        scanner.skipLine();
        segment.push_back(Vector3(x, -z, y) * inchToMeter);
    }

    if(segment.size() != numFrames){
        addSeqMessage(
            format("Warning: Frames: {0} of Segment {1} is different from the actual number of frames {2}.\n",
                   numFrames, label,segment.size()));
    }
    
    return segment.size();
}


bool MarkerMotion::loadTRC(const std::string& filename)
{
    bool result = true;

    clearSeqMessage();
    
    try {
        labelToIndexMap.clear();
        labels.clear();
        
        EasyScanner scanner(filename);

        scanner.skipLine();

        scanner >> "DataRate" >> "CameraRate" >> "NumFrames" >> "NumMarkers" >> "Units"
                >> "OrigDataRate" >> "OrigDataStartFrame" >> "OrigNumFrames" >> scanner.endl;

        double DataRate, CameraRate, OrigDataRate;
        int NumFrames, NumMarkers, OrigDataStartFrame, OrigNumFrames;
        string Units;

        scanner >> DataRate >> CameraRate >> NumFrames >> NumMarkers >> Units
                >> OrigDataRate >> OrigDataStartFrame >> OrigNumFrames >> scanner.endl;

        if(NumMarkers < 0){
            scanner.throwException("Negative value cannot be specified for NumMarkers");
        }
        if(NumMarkers > 1000){
            scanner.throwException("The value of NumMarkers is too many");
        }
        if(NumFrames <= 0){
            scanner.throwException("NumFrames value is invalid");
        }
        if(Units != "mm"){
            scanner.throwException(format("Unit {0} is not supported.", scanner.stringValue));
        }   

        scanner >> "Frame#" >> "Time";

        int index = 0;
        while(scanner.readWord()){
            setLabel(scanner.stringValue, index++);
        }
        if(index != NumMarkers){
            scanner.throwException("The number of marker labels is different from the number of markers.");
        }
        scanner.readLFex("Missing actual data");
        
        scanner.skipLine();

        scanner.skipBlankLines();

        setFrameRate(DataRate);
        setDimension(NumFrames, NumMarkers);

        while(!scanner.isEOF()){
            int frameIndex = scanner.readIntEx("Frame index is not specified") - 1;
            if(frameIndex < 0){
                scanner.throwException("Negative frame index");
            }
            if(frameIndex >= NumFrames){
                scanner.throwException("Frame index is over the number of frames");
            }
            Frame markers = frame(frameIndex);
            double time = scanner.readDoubleEx("Time is not specified");

            int markerIndex = 0;
            while(!scanner.readLF()){
                Vector3 p; 
                scanner >> p.x() >> p.y() >> p.z();
                p *= 0.001; // to meter
                if(markerIndex == NumMarkers){
                    scanner.throwException(
                        format("Frame {0} has elements more than the number of markers", frameIndex));
                }
                markers[markerIndex++] = p;
            }
            if(markerIndex + 1 < NumMarkers){
                scanner.throwException(
                    format("Frame {0} does not contain all marker positions", frameIndex));
            }
        }
            
    } catch(EasyScanner::Exception& ex){
        addSeqMessage(ex.getFullMessage() + "\n");
        result = false;
    }

    return result;
}
