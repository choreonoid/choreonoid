#include "MocapMapping.h"
#include <cnoid/YAMLReader>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


MocapMapping::MocapMapping()
{

}


MocapMapping::MocapMapping(const MocapMapping& org)
    : name_(org.name_),
      labelMap_(org.labelMap_),
      extraMarkers(org.extraMarkers),
      unnecessaryMarkers(org.unnecessaryMarkers),
      staticMarkers(org.staticMarkers),
      isStaticBody(org.isStaticBody)
{
    // do deep copy keeping shareing between skeletonEdges and markerEdges
    std::map<EdgePtr, EdgePtr> copiedEdgeMap;
    for(int i=0; i < org.markerEdges.size(); ++i){
        const EdgePtr& orgEdge = org.markerEdges[i];
        copiedEdgeMap[orgEdge] = new Edge(*orgEdge);
        // Copying to markerEdges is not done here to reduce the memory fragments
    }
    markerEdges.reserve(org.markerEdges.size());
    for(int i=0; i < org.markerEdges.size(); ++i){
        markerEdges.push_back(copiedEdgeMap[org.markerEdges[i]]);
    }
    skeletonEdges.reserve(org.skeletonEdges.size());
    for(int i=0; i < org.skeletonEdges.size(); ++i){
        skeletonEdges.push_back(copiedEdgeMap[org.skeletonEdges[i]]);
    }

    // do deep copy keeping shareing between skeletonSegments and markerSegments
    std::map<SegmentPtr, SegmentPtr> copiedSegmentMap;
    for(int i=0; i < org.markerSegments.size(); ++i){
        const SegmentPtr& orgSegment = org.markerSegments[i];
        copiedSegmentMap[orgSegment] = new Segment(*orgSegment);
        // Copying to markerSegments is not done here to reduce the memory fragments
    }
    markerSegments.reserve(org.markerSegments.size());
    for(int i=0; i < org.markerSegments.size(); ++i){
        markerSegments.push_back(copiedSegmentMap[org.markerSegments[i]]);
    }
    skeletonSegments.reserve(org.skeletonSegments.size());
    for(int i=0; i < org.skeletonSegments.size(); ++i){
        skeletonSegments.push_back(copiedSegmentMap[org.skeletonSegments[i]]);
    }
}


MocapMapping::~MocapMapping()
{

}


bool MocapMapping::load(const std::string filename, std::ostream& os)
{
    name_.clear();
    labelMap_.clear();
    skeletonEdges.clear();
    markerEdges.clear();
    skeletonSegments.clear();
    markerSegments.clear();
    extraMarkers.clear();
    unnecessaryMarkers.clear();
    staticMarkers.clear();
    isStaticBody = false;

    YAMLReader reader;
    if(!reader.load(filename)){
        os << reader.errorMessage() << endl;
    } else {
        Mapping* top = reader.document()->toMapping();
        if(top->isValid()){
            top->read("character_name", name_);

            NamePairSet namePairs;

            Listing* labelMapNode = top->findListing("labelMap");
            if(labelMapNode->isValid()){
                loadLabelMap(*labelMapNode, os);
            }

            Listing* edgesNode = top->findListing("edges");
            if(edgesNode->isValid()){
                skeletonEdges.reserve(edgesNode->size());
                loadEdges(*edgesNode, skeletonEdges, namePairs, os);
            }

            Listing* unnecessaryMarkersNode = top->findListing("unnecessaryMarkers");
            if(unnecessaryMarkersNode->isValid()){
                for(int i=0; i < unnecessaryMarkersNode->size(); ++i){
                    unnecessaryMarkers.insert((*unnecessaryMarkersNode)[i].toString());
                }
            }

            // make marker edges excluding ones related with unnecessary markers
            for(int i=0; i < skeletonEdges.size(); ++i){
                EdgePtr& e = skeletonEdges[i];
                if(!matchUnnecessaryMarker(e->label[0]) && !matchUnnecessaryMarker(e->label[1])){
                    markerEdges.push_back(e);
                }
            }
            
            Listing* extraMarkersNode = top->findListing("extraMarkers");
            if(extraMarkersNode->isValid()){
                loadExtraMarkers(*extraMarkersNode);
            }

            Listing* extraEdgesNode = top->findListing("extraMarkerEdges");
            if(extraEdgesNode->isValid()){
                loadEdges(*extraEdgesNode, markerEdges, namePairs, os);
            }

            Listing* staticMarkersNode = top->findListing("staticMarkers");
            if(staticMarkersNode->isValid()){
                setStaticMarkers(*staticMarkersNode);
            }

        }
        return true;
    }
    return false;
}


void MocapMapping::loadLabelMap(Listing& labelMapNode, std::ostream& os)
{
    for(int i=0; i < labelMapNode.size(); ++i){
        Listing& labelPairNode = *labelMapNode[i].toListing();
        if(labelPairNode.isValid()){
            if(labelPairNode.size() != 2){
                os << _("Warning: labelMap contains a node which is not a pair of labels.") << endl;
            } else {
                pair<string, string> m;
                m.first = labelPairNode[1];
                m.second = labelPairNode[0];
                if(m.first.empty() || m.second.empty()){
                    os << _("Warning: labelMap contains a node which has an empty label.") << endl;
                } else {
                    pair<LabelMap::iterator, bool> ret = labelMap_.insert(m);
                    if(!ret.second){
                        os << format(_("Warning: Label mapping (\"{0}\", {1}) collides with an existing one."),
                                     m.first, m.second) << endl;
                    }
                }
            }
        }
    }
}


void MocapMapping::loadEdges
(Listing& edgesNode, std::vector<EdgePtr>& edges, NamePairSet& namePairs, std::ostream& os)
{
    for(int i=0; i < edgesNode.size(); ++i){
        Listing& edgeNode = *edgesNode[i].toListing();
        if(edgeNode.isValid() && edgeNode.size() >= 2){
            EdgePtr edge = new Edge();
            edge->label[0] = edgeNode[0].toString();
            edge->label[1] = edgeNode[1].toString();
            if(edgeNode.size() >= 3){
                edge->length = edgeNode[2].toDouble();
            }
            if(!edge->label[0].empty() && !edge->label[1].empty()){
                if(!namePairs.insert(NamePair(edge->label[0], edge->label[1])).second){ // exsiting pair?
                    os << format(_("Warning: Edge \"{0}\" to \"{1}\" is doubly defined."),
                                 edge->label[0], edge->label[1]) << endl;
                } else {
                    edges.push_back(edge);
                }
            }
        }
    }
}


void MocapMapping::loadExtraMarkers(Listing& extraMarkersNode)
{
    extraMarkers.reserve(extraMarkersNode.size());
    
    for(int i=0; i < extraMarkersNode.size(); ++i){
        Mapping& extraMarkerNode = *extraMarkersNode[i].toMapping();
        if(extraMarkerNode.isValid()){
            ExtraMarker eMarker;
            eMarker.markerLabel = extraMarkerNode["marker"].toString();
            eMarker.boneLabel = extraMarkerNode["bone"].toString();
            if(!eMarker.markerLabel.empty() && !eMarker.boneLabel.empty()){
                if(!read(extraMarkerNode, "pos", eMarker.localPosition)){
                    eMarker.localPosition.setZero();
                }
                extraMarkers.push_back(eMarker);
            }
        }
    }
}


void MocapMapping::setStaticMarkers(Listing& staticMarkersNode)
{
    for(int i=0; i < staticMarkersNode.size(); ++i){
        string label = staticMarkersNode[i].toString();
        if(label == "all"){
            isStaticBody = true;
            break;
        }
        staticMarkers.insert(label);
    }
}
