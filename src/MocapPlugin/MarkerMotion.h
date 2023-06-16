#ifndef CNOID_MOCAP_PLUGIN_MARKER_MOTION_H
#define CNOID_MOCAP_PLUGIN_MARKER_MOTION_H

#include <cnoid/MultiVector3Seq>
#include <vector>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class EasyScanner;

class CNOID_EXPORT MarkerMotion : public MultiVector3Seq
{
public:
    typedef std::shared_ptr<MarkerMotion> Ptr;

    MarkerMotion();
    MarkerMotion(const MarkerMotion& org);
    ~MarkerMotion();

    MarkerMotion& operator=(const MarkerMotion& rhs);

    void copySeqProperties(const MarkerMotion& source);

    void clearLabels();
    void setLabel(const std::string& label, int markerIndex);

    int numMarkers() const { return numParts(); }

    virtual int partIndex(const std::string& partLabel) const override;
    virtual const std::string& partLabel(int partIndex) const override;

    //! \return -1 if the marker of the label is not found.
    int markerIndex(const std::string& label) const { return partIndex(label); }
    const std::string& markerLabel(int markerIndex) const { return labels[markerIndex]; }

    const Vector3& positionOffset() const { return positionOffset_; }
    void setPositionOffset(const Vector3& offset);

    Vector3 applyOffset(const Vector3& position) const {
        return position + positionOffset_;
    }

    bool loadStdYAMLformat(const std::string& filename);
    bool saveAsStdYAMLformat(const std::string& filename);
    bool loadVPM(const std::string& filename);
    bool loadTRC(const std::string& filename);

protected:
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;

private:
    typedef std::map<std::string, int> LabelToIndexMap;
    LabelToIndexMap labelToIndexMap;
    std::vector<std::string> labels;
    Vector3 positionOffset_;

    int readVpmSegment(EasyScanner& scanner, std::vector<Vector3>& segment, const std::string& label);
};

typedef MarkerMotion::Ptr MarkerMotionPtr;

}

#endif
