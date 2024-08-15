#include "AbstractSceneLoader.h"
#include "ValueTree.h"

using namespace std;
using namespace cnoid;


AbstractSceneLoader::AbstractSceneLoader()
{
    clearHintsForLoading();
}


AbstractSceneLoader::~AbstractSceneLoader()
{

}


void AbstractSceneLoader::setMessageSink(std::ostream& /* os */)
{

}


void AbstractSceneLoader::setDefaultDivisionNumber(int /* n */)
{

}


void AbstractSceneLoader::setDefaultCreaseAngle(double /* theta */)
{

}


void AbstractSceneLoader::setLengthUnitHint(LengthUnitType hint)
{
    lengthUnitHint_ = hint;
}


void AbstractSceneLoader::setUpperAxisHint(UpperAxisType hint)
{
    upperAxisHint_ = hint;
}


void AbstractSceneLoader::clearHintsForLoading()
{
    lengthUnitHint_ = Meter;
    upperAxisHint_ = Z_Upper;
}


void AbstractSceneLoader::restoreLengthUnitAndUpperAxisHints(Mapping* metadata)
{
    if(metadata){
        string symbol;
        if(metadata->read("length_unit", symbol)){
            if(symbol == "millimeter"){
                lengthUnitHint_ = Millimeter;
            } else if(symbol == "inch"){
                lengthUnitHint_ = Inch;
            }
        }
        if(metadata->read("upper_axis", symbol)){
            if(symbol == "Y"){
                upperAxisHint_ = Y_Upper;
            }
        }
    }
}


/**
   This function inserts a SgScaleTransform node and a SgPosTransform node to adjust the
   length unit and the upper direction axis. Each loader can use this function when all
   the model data is loaded from a file in the load function.
   \param node Original top node
   \return New top node of the scene graph where the transform nodes are inserted
   \note This function should not be used. For the length unit, it is better to directly
   modify the individual coordinate values including vertex positions and transform parameters.
*/
SgNode* AbstractSceneLoader::insertTransformNodesToAdjustLengthUnitAndUpperAxis(SgNode* node)
{
    if(!node){
        return nullptr;
    }
    if(lengthUnitHint_ != Meter){
        auto scale = new SgScaleTransform;
        if(lengthUnitHint_ == Millimeter){
            scale->setScale(1.0 / 1000.0);
        } else if(lengthUnitHint_ == Inch){
            scale->setScale(0.0254);
        }
        scale->addChild(node);
        node = scale;
    }

    return insertTransformNodeToAdjustUpperAxis(node);
}


/**
   This function inserts a SgPosTransform node to adjust the the upper direction axis. Each
   loader can use this function when all the model data is loaded from a file inthe load function.
   \param node Original top node
   \return New top node of the scene graph where the transform nodes are inserted
*/
SgNode* AbstractSceneLoader::insertTransformNodeToAdjustUpperAxis(SgNode* node)
{
    if(!node){
        return nullptr;
    }
    if(upperAxisHint_ == Y_Upper){
        auto transform = new SgPosTransform;
        Matrix3 R;
        R << 0, 0, 1,
             1, 0, 0,
             0, 1, 0;
        transform->setRotation(R);
        transform->addChild(node);
        node = transform;
    }
    return node;
}


void AbstractSceneLoader::storeLengthUnitAndUpperAxisHintsAsMetadata(SgObject* object)
{
    MappingPtr metadata = new Mapping;
    if(lengthUnitHint_ == Millimeter){
        metadata->write("length_unit", "millimeter");
    } else if(lengthUnitHint_ == Inch){
        metadata->write("length_unit", "inch");
    }
    if(upperAxisHint_ == Y_Upper){
        metadata->write("upper_axis", "Y");
    }
    if(!metadata->empty()){
        object->setUriMetadata(metadata);
    }
}
