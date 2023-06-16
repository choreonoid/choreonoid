#ifndef CNOID_MOCAP_PLUGIN_MOCAP_MAAPING_H
#define CNOID_MOCAP_PLUGIN_MOCAP_MAAPING_H

#include <cnoid/Referenced>
#include <cnoid/NullOut>
#include <cnoid/EigenTypes>
#include <cnoid/stdx/optional>
#include <vector>
#include <map>
#include <set>
#include "exportdecl.h"

namespace cnoid {

class Listing;

/**
   This class determines how a skeleton or a marker set is rendered
   or calculated.
*/
class CNOID_EXPORT MocapMapping : public Referenced
{
public:
    MocapMapping();
    MocapMapping(const MocapMapping& org);
    ~MocapMapping();

    bool load(const std::string filename, std::ostream& os = nullout());

    const std::string name() const { return name_; }

    typedef std::map<std::string, std::string> LabelMap;
    const LabelMap& labelMap() const { return labelMap_; }
    const std::string& convertLabel(const std::string& orgLabel) const {
        LabelMap::const_iterator p = labelMap_.find(orgLabel);
        if(p == labelMap_.end()){
            return orgLabel;
        } else {
            return p->second;
        }
    }

    bool isMarkerStatic(const std::string& markerLabel) const {
        return isStaticBody || (staticMarkers.find(markerLabel) != staticMarkers.end());
    }

    class Edge : public Referenced
    {
    public:
        std::string label[2];
        stdx::optional<double> length;
    };

    int numSkeletonEdges() const { return skeletonEdges.size(); }
    const Edge& skeletonEdge(int index) const { return *skeletonEdges[index]; }

    int numMarkerEdges() const { return markerEdges.size(); }
    const Edge& markerEdge(int index) const { return *markerEdges[index]; }

    class CoeffPoint
    {
    public:
        std::string label;
        double r;
    };

    class Segment : public Referenced
    {
    public:
        std::string label;
        enum DrawType { CYLINDER };
        std::vector<CoeffPoint> axis[3];
        double spanRatio[3];
        // used when there is no specified axis or corresponding spanRatio is zero
        double spanConst[3];
    };

    int numSkeletonSegments() const { return skeletonSegments.size(); }
    const Segment& skeletonSegment(int index) const { return *skeletonSegments[index]; }

    int numMarkerSegments() const { return markerSegments.size(); }
    const Segment& markerSegment(int index) const { return *markerSegments[index]; }

    class ExtraMarker
    {
    public:
        std::string markerLabel;
        std::string boneLabel;
        Vector3 localPosition;
    };

    int numExtraMarkers() const { return extraMarkers.size(); }
    const ExtraMarker extraMarker(int index) const { return extraMarkers[index]; }

    bool matchUnnecessaryMarker(const std::string& label) const {
        return unnecessaryMarkers.find(label) != unnecessaryMarkers.end();
    }

private:
    std::string name_;

    typedef ref_ptr<Edge> EdgePtr;
    typedef ref_ptr<Segment> SegmentPtr;

    LabelMap labelMap_;
    std::vector<EdgePtr> skeletonEdges;
    std::vector<EdgePtr> markerEdges;
    std::vector<SegmentPtr> skeletonSegments;
    std::vector<SegmentPtr> markerSegments;
    std::vector<ExtraMarker> extraMarkers;
    std::set<std::string> unnecessaryMarkers;
    std::set<std::string> staticMarkers;
    bool isStaticBody;

    struct NamePair
    {
        std::string name1;
        std::string name2;
        NamePair(const std::string& name1, const std::string& name2) {
            if(name1 < name2){
                this->name1 = name1;
                this->name2 = name2;
            } else {
                this->name1 = name2;
                this->name2 = name1;
            }
        }
        bool operator<(const NamePair& rhs) const {
            if(name1 < rhs.name1){
                return true;
            }
            if(name1 == rhs.name1){
                return (name2 < rhs.name2);
            }
            return false;
        };
    };
    typedef std::set<NamePair> NamePairSet;

    void loadLabelMap(Listing& labelMapNode, std::ostream& os);
    void loadEdges(Listing& edgesNode, std::vector<EdgePtr>& edges, NamePairSet& namePairs, std::ostream& os);
    void loadExtraMarkers(Listing& extraMarkersNode);
    void setStaticMarkers(Listing& staticMarkersNode);
};

typedef ref_ptr<MocapMapping> MocapMappingPtr;

}

#endif
