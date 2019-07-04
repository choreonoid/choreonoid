/*!
  @file
  @author Shin'ichiro Nakaoka
  @author K.Fukuda
  @author Takafumi.Tawara
*/

#include "VRMLParser.h"
#include "EasyScanner.h"
#include "UTF8.h"
#include "NullOut.h"
#include "FileUtil.h"
#include <boost/algorithm/string/predicate.hpp>
#include <list>
#include <cmath>
#include <vector>
#include <mutex>

using namespace std;
using namespace cnoid;

namespace {

string removeURLScheme(string url)
{
    static const string fileProtocolHeader1("file://");
    static const string fileProtocolHeader2("file:");

    size_t pos = url.find(fileProtocolHeader1);
    if(0 == pos){
        url.erase(0, fileProtocolHeader1.size());
    } else {
        size_t pos = url.find(fileProtocolHeader2);
        if(0 == pos) {
            url.erase(0, fileProtocolHeader2.size());
        }
    }

    // Remove the first slash if the successive part is a Windows drive letter
    if(url.find(":") == 2 && url.find("/") ==0){
        url.erase(0, 1);
    }

    return url;
}


bool isFileProtocol(const string& ref)
{
    bool ret = false;
    string::size_type pos = ref.find(":");
    if(pos == string::npos || pos == 1){
        // Directly local path || Windows drive letter separator
        ret = true;
    } else if(ref.find("file:") == 0){
        ret = true;
    }
    return ret;
}

/**
   The definition of the reserved word IDs
*/
enum {

    NO_SYMBOL = 0,

    // values
    V_TRUE,
    V_FALSE,
    V_NULL,

    // types
    T_SFINT32,
    T_MFINT32,
    T_SFFLOAT,
    T_MFFLOAT,
    T_SFVEC2F,
    T_MFVEC2F,
    T_SFVEC3F,
    T_MFVEC3F,
    T_SFSTRING,
    T_MFSTRING,
    T_SFROTATION,
    T_MFROTATION,
    T_SFTIME,
    T_MFTIME,
    T_SFCOLOR,
    T_MFCOLOR,
    T_SFNODE,
    T_MFNODE,
    T_SFBOOL,
    T_SFIMAGE,

    // Nodes
    N_PROTO,
    N_INLINE,
    N_BACKGROUND,
    N_NAVIGATION_INFO,
    N_VIEWPOINT,
    N_GROUP,
    N_TRANSFORM,
    N_APPEARANCE,
    N_MATERIAL,
    N_IMAGE_TEXTURE,
    N_TEXTURE_TRANSFORM,
    N_SHAPE,
    N_BOX,
    N_CONE,
    N_CYLINDER,
    N_SPHERE,
    N_TEXT,
    N_FONT_STYLE,
    N_INDEXED_LINE_SET,
    N_INDEXED_FACE_SET,
    N_COLOR,
    N_COORDINATE,
    N_TEXTURE_COORDINATE,
    N_NORMAL,
    N_CYLINDER_SENSOR,

    N_POINTSET,
    N_PIXEL_TEXTURE,
    N_MOVIE_TEXTURE,
    N_ELEVATION_GRID,
    N_EXTRUSION,
    N_SWITCH,
    N_LOD,
    N_COLLISION,
    N_ANCHOR,
    N_FOG,
    N_BILLBOARD,
    N_WORLD_INFO,
    N_POINT_LIGHT,
    N_DIRECTIONAL_LIGHT,
    N_SPOT_LIGHT,

    N_AUDIO_CLIP,
    N_SOUND,
    N_COLOR_INTERPOLATOR,
    N_COORDINATE_INTERPOLATOR,
    N_ORIENTATION_INTERPOLATOR,
    N_NORMAL_INTERPOLATOR,
    N_POSITION_INTERPOLATOR,
    N_SCALAR_INTERPOLATOR,
    N_PLANE_SENSOR,
    N_PROXIMITY_SENSOR,
    N_SPHERE_SENSOR,
    N_TIME_SENSOR,
    N_TOUCH_SENSOR,
    N_VISIBILITY_SENSOR,

    // Fields
    F_IS,

    F_URL,

    F_GROUND_ANGLE,
    F_GROUND_COLOR,
    F_SKY_ANGLE,
    F_SKY_COLOR,
    F_BACK_URL,
    F_BOTTOM_URL,
    F_FRONT_URL,
    F_LEFT_URL,
    F_RIGHT_URL,
    F_TOP_URL,

    F_AVATAR_SIZE,
    F_HEADLIGHT,
    F_SPEED,
    F_TYPE,
    F_VISIBILITY_LIMIT,

    F_FIELD_OF_VIEW,
    F_JUMP,
    F_ORIENTATION,
    F_POSITION,
    F_DESCRIPTION,

    F_CHILDREN,
    F_ADD_CHILDREN,
    F_REMOVE_CHILDREN,
    F_BBOX_CENTER,
    F_BBOX_SIZE,

    F_CENTER,
    F_ROTATION,
    F_SCALE,
    F_SCALE_ORIENTATION,
    F_TRANSLATION,

    F_APPEARANCE,
    F_GEOMETRY,

    F_MATERIAL,
    F_TEXTURE,
    F_TEXTURE_TRANSFORM,

    F_AMBIENT_INTENSITY,
    F_DIFFUSE_COLOR,
    F_EMISSIVE_COLOR,
    F_SHININESS,
    F_SPECULAR_COLOR,
    F_TRANSPARANCY,

    F_REPEAT_S,
    F_REPEAT_T,

    F_SIZE,

    F_BOTTOM,
    F_BOTTOM_RADIUS,
    F_HEIGHT,
    F_SIDE,

    F_RADIUS,
    F_TOP,

    F_STRING,
    F_FONT_STYLE,
    F_LENGTH,
    F_MAX_EXTENT,

    F_FAMILY,
    F_HORIZONTAL,
    F_JUSTIFY,
    F_LANGUAGE,
    F_LEFT_TO_RIGHT,
    F_SPACING,
    F_STYLE,
    F_TOP_TO_BOTTOM,

    F_COLOR,
    F_COORD,
    F_COLOR_INDEX,
    F_COLOR_PER_VERTEX,
    F_COORD_INDEX,

    F_CCW,
    F_CONVEX,
    F_SOLID,
    F_CREASE_ANGLE,
    F_NORMAL,
    F_NORMAL_INDEX,
    F_NORMAL_PER_VERTEX,
    F_TEX_COORD_INDEX,
    F_TEX_COORD,

    F_POINT,
    F_VECTOR,

    F_BBOXCENTER,
    F_BBOXSIZE,

    F_AUTO_OFFSET,
    F_DISK_ANGLE,
    F_ENABLED,
    F_MAX_ANGLE,
    F_MIN_ANGLE,
    F_OFFSET,

    F_IMAGE,

    F_X_DIMENSION,
    F_Z_DIMENSION,
    F_X_SPACING,
    F_Z_SPACING,

    F_LOOP,
    F_START_TIME,
    F_STOP_TIME,

    F_CROSS_SECTION,
    F_SPINE,
    F_BEGIN_CAP,
    F_END_CAP,

    F_CHOICE,
    F_WHICH_CHOICE,

    F_RANGE,
    F_LEVEL,

    F_COLLIDE,
    F_PROXY,

    F_PARAMETER,

    F_VISIBILITY_RANGE,
    F_FOG_TYPE,

    F_AXIS_OF_ROTATION,

    F_TITLE,
    F_INFO,

    F_LOCATION,
    F_ON,
    F_INTENSITY,
    F_ATTENUATION,
    F_DIRECTION,
    F_BEAM_WIDTH,
    F_CUT_OFF_ANGLE,

    // event type
    E_FIELD,
    E_EXPOSED_FIELD,
    E_EVENTIN,
    E_EVENTOUT,

    // def & route
    D_DEF,
    D_USE,
    D_ROUTE,

    // unsupported keywords
    U_SCRIPT,
    U_EXTERNPROTO
};

}

namespace cnoid {

class VRMLParserImpl
{
public:
    VRMLParserImpl(VRMLParser* self);

    VRMLParser* self;
    ostream* os_;
    ostream& os() { return *os_; }
    std::shared_ptr<EasyScanner> topScanner;
    EasyScanner* scanner; // current one
    VRMLProtoInstancePtr currentProtoInstance;

    bool protoInstanceActualNodeExtractionMode;

    typedef map<VRMLProto*, std::shared_ptr<EasyScanner> > ProtoToEntityScannerMap;
    ProtoToEntityScannerMap protoToEntityScannerMap;

    typedef map<string, VRMLNodePtr> TDefNodeMap;
    typedef pair<string, VRMLNodePtr> TDefNodePair;
    typedef map<string, VRMLProtoPtr> TProtoMap;
    typedef pair<string, VRMLProtoPtr> TProtoPair;

    TProtoMap protoMap;
    TDefNodeMap defNodeMap;

    void load(const string& filename, bool doClearAncestorPathsList);
    VRMLNodePtr readSpecificNode(VRMLNodeCategory nodeCategory, int symbol, const std::string& symbolString);
    VRMLNodePtr readInlineNode(VRMLNodeCategory nodeCategory);
    VRMLNodePtr newInlineSource(string& io_filename);
    VRMLProtoPtr defineProto();

    void checkEOF();
    VRMLNodePtr readNode(VRMLNodeCategory nodeCategory);
    VRMLProtoInstancePtr readProtoInstanceNode(const std::string& proto_name, VRMLNodeCategory nodeCategory, const string& defName);
    VRMLNodePtr evalProtoInstance(VRMLProtoInstancePtr proto, VRMLNodeCategory nodeCategory, const string& defName);
    VRMLUnsupportedNodePtr skipUnsupportedNode(const std::string& nodeTypeName);
    VRMLUnsupportedNodePtr skipScriptNode();
    VRMLUnsupportedNodePtr skipExternProto();

    VRMLViewpointPtr readViewpointNode();
    VRMLNavigationInfoPtr readNavigationInfoNode();
    VRMLBackgroundPtr readBackgroundNode();
    VRMLGroupPtr readGroupNode();
    VRMLTransformPtr readTransformNode();
    VRMLShapePtr readShapeNode();
    VRMLCylinderSensorPtr readCylinderSensorNode();
    VRMLBoxPtr readBoxNode();
    VRMLConePtr readConeNode();
    VRMLCylinderPtr readCylinderNode();

    VRMLPointSetPtr readPointSetNode();
    VRMLPixelTexturePtr readPixelTextureNode();
    VRMLMovieTexturePtr readMovieTextureNode();
    VRMLElevationGridPtr readElevationGridNode();
    VRMLExtrusionPtr readExtrusionNode();
    VRMLSwitchPtr readSwitchNode();
    VRMLLODPtr readLODNode();
    VRMLCollisionPtr readCollisionNode();
    VRMLAnchorPtr readAnchorNode();
    VRMLFogPtr readFogNode();
    VRMLBillboardPtr readBillboardNode();
    VRMLWorldInfoPtr readWorldInfoNode();
    VRMLPointLightPtr readPointLightNode();
    VRMLDirectionalLightPtr	readDirectionalLightNode();
    VRMLSpotLightPtr readSpotLightNode();

    VRMLSpherePtr readSphereNode();
    VRMLTextPtr readTextNode();
    VRMLFontStylePtr readFontStyleNode();
    VRMLIndexedLineSetPtr readIndexedLineSetNode();
    VRMLIndexedFaceSetPtr readIndexedFaceSetNode();
    void checkIndexedFaceSet(VRMLIndexedFaceSetPtr node);
    VRMLCoordinatePtr readCoordNode();
    VRMLTextureCoordinatePtr readTextureCoordinateNode();
    VRMLColorPtr readColorNode();
    VRMLAppearancePtr readAppearanceNode();
    VRMLMaterialPtr readMaterialNode();
    VRMLImageTexturePtr readImageTextureNode();
    VRMLTextureTransformPtr readTextureTransformNode();
    VRMLNormalPtr readNormalNode();
  
    VRMLVariantField& readProtoField(VRMLFieldTypeId fieldTypeId);
  
    void readSFInt32(SFInt32& out_value);
    void readMFInt32(MFInt32& out_value);
    void readSFFloat(SFFloat& out_value);
    void readMFFloat(MFFloat& out_value);
    void readSFColor(SFColor& out_value); 
    void readMFColor(MFColor& out_value); 
        
    void readSFString(SFString& out_value);
    void readMFString(MFString& out_value);
    inline void readSFVec2f(SFVec2f& out_value);
    void readMFVec2f(MFVec2f& out_value);
    inline void readSFVec2s(SFVec2s& out_value);
    void readMFVec2s(MFVec2s& out_value);
    inline void readSFVec3f(SFVec3f& out_value);
    void readMFVec3f(MFVec3f& out_value);
    inline void readSFVec3s(SFVec3s& out_value);
    void readMFVec3s(MFVec3s& out_value);
    void readSFRotation(SFRotation& out_value);
    void readMFRotation(MFRotation& out_value);
    void readSFBool(SFBool& out_value);
    void readSFTime(SFTime& out_value);
    void readMFTime(MFTime& out_value);
    void readSFNode(SFNode& out_node, VRMLNodeCategory nodeCategory);
    SFNode readSFNode(VRMLNodeCategory nodeCategory);
    void readMFNode(MFNode& out_nodes, VRMLNodeCategory nodeCategory);
    void readSFImage( SFImage& out_image );
private:
    VRMLParserImpl(const VRMLParserImpl& self, const list<string>& ref);
    const list<string>* getAncestorPathsList() const {return &ancestorPathsList;}
    void setSymbols();
    void init();
    void convertUrl(MFString& url);
    list<string> ancestorPathsList;
    string getRealPath(string url);
};
}


VRMLParser::VRMLParser()
{
    init();
}


VRMLParser::VRMLParser(const string& filename)
{
    init();
    load(filename);
}



void VRMLParser::init()
{

    impl = new VRMLParserImpl(this);
}


VRMLParserImpl::VRMLParserImpl(VRMLParser* self)
    : self(self)
{
    init();
}

VRMLParserImpl::VRMLParserImpl(const VRMLParserImpl& refThis, const list<string>& refSet)
    : self(refThis.self), ancestorPathsList(refSet)
{
    init();
}


VRMLParser::~VRMLParser()
{
    delete impl;
}


void VRMLParser::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void VRMLParser::setProtoInstanceActualNodeExtractionMode(bool isOn)
{
    impl->protoInstanceActualNodeExtractionMode = isOn;
}


/**
   This function throws EasyScanner::Exception when an error occurs.
*/
void VRMLParser::load(const string& filename)
{
    impl->load(filename, true);
}


void VRMLParserImpl::load(const string& filename, bool doClearAncestorPathsList)
{
    currentProtoInstance = 0;
    protoToEntityScannerMap.clear();
    protoMap.clear();
    defNodeMap.clear();
    
    stdx::filesystem::path path(stdx::filesystem::lexically_normal(filename));
    string pathString(path.string());
    if(doClearAncestorPathsList){
        ancestorPathsList.clear();
    }
    ancestorPathsList.push_back(pathString);
    scanner->loadFile(pathString);
    
    // header check
    scanner->setCommentChar(0);
    bool ok = scanner->readString("#VRML V2.0");
    if(ok){
        scanner->skipLine();
    }
    
    scanner->setCommentChar('#');
    scanner->setQuoteChar('"');
    scanner->setWhiteSpaceChar(',');
    scanner->setLineOriented(false);
}


void VRMLParserImpl::checkEOF()
{
    if(!scanner->isEOF()){
        const int c = scanner->peekChar();
        if(c == ']'){
            scanner->throwException("Extra closing bracket exists in the end of file");
        } else if(c == '}'){
            scanner->throwException("Extra closing brace exists in the end of file");
        } else {
            scanner->throwException("The file is not correctly ended");
        }
    }
}


void VRMLParser::checkEOF()
{
    impl->checkEOF();
}


VRMLNodePtr VRMLParser::readNode()
{
    return impl->readNode(TOP_NODE);
}


VRMLNodePtr VRMLParserImpl::readNode(VRMLNodeCategory nodeCategory)
{
    VRMLNodePtr node;

    /* comment out this to allow empty proto instance node
       if(scanner->isEOF()){
       if(currentProtoInstance){
       scanner->throwException("Illegal proto instance node");
       }
       }
    */

    if(!scanner->readWord()){
        return 0;
    }

    string def_name;
    string nodeTypeName(scanner->stringValue);
    int symbol = scanner->getSymbolID(scanner->stringValue);

    if(symbol){

        if(symbol==N_INLINE){
            return readInlineNode(nodeCategory);
        }

        if(symbol==N_PROTO){
            if(nodeCategory == TOP_NODE){
                return defineProto();
            } else {
                scanner->throwException("PROTO node cannot be defined here");
            }
        }

        if(symbol==D_USE){
            scanner->readString();
            const string& label = scanner->stringValue;
            TDefNodeMap::iterator p = defNodeMap.find(label);
            if(p != defNodeMap.end()){
                return p->second;
            } else {
                scanner->throwException
                    (string("A node \"") + label + "\" specified by the USE directive does not exist");
            }
        }

        // ROUTE has 3 parameters to skip
        // return as VRMLUnsupportedNode
        if(symbol == D_ROUTE){
            scanner->readString();	// eventOut or exposedField
            if(!scanner->readString("TO")){	// "TO"
                scanner->throwException("Illegal ROUTE (without TO)");
            }
            scanner->readString();	// eventIn or exposedField
            // recursive call to continue reading without node construction
            return readNode(nodeCategory);
        }

        // unsupported keywords
        if(symbol == U_SCRIPT){
            os() << "Script is not supported. " << endl;
            skipScriptNode();
            return readNode(nodeCategory);
        }
        if(symbol == U_EXTERNPROTO){
            os() << "ExternProto is not supported." << endl;
            skipExternProto();
            return readNode(nodeCategory);
        }

        if(symbol == D_DEF){
            def_name = scanner->readStringEx("Illegal DEF name");
            scanner->readWord();
            symbol = scanner->getSymbolID(scanner->stringValue);
            nodeTypeName = scanner->stringValue;
        }

    }

    if(!scanner->readChar('{')){
        scanner->throwException
            (string("The entity of a ") + nodeTypeName + " node does not correctly begin with '{'");
    }

    if(symbol){
        node = readSpecificNode(nodeCategory, symbol, nodeTypeName);
    } else {
        node = readProtoInstanceNode(scanner->stringValue, nodeCategory, def_name);
    }

    if(!scanner->readChar('}')){
        scanner->throwException
            (string("A ") + nodeTypeName + " node is not correctly closed with '}'");
    }

    if(def_name.size() > 0) {
        defNodeMap.insert(TDefNodePair(def_name, node));
        node->defName = def_name;
    }

    return node;
}


VRMLNodePtr VRMLParserImpl::readSpecificNode(VRMLNodeCategory nodeCategory, int symbol, const string& nodeTypeName)
{
    VRMLNodePtr node;

    switch(symbol){

    case N_BACKGROUND:         node = readBackgroundNode();        break;
    case N_NAVIGATION_INFO:    node = readNavigationInfoNode();    break;
    case N_VIEWPOINT:          node = readViewpointNode();         break;

    case N_GROUP:              node = readGroupNode();             break;
    case N_TRANSFORM:          node = readTransformNode();         break;
    case N_SHAPE:              node = readShapeNode();             break;
    case N_CYLINDER_SENSOR:    node = readCylinderSensorNode();    break;

    case N_POINTSET:           node = readPointSetNode();          break;
    case N_PIXEL_TEXTURE:      node = readPixelTextureNode();      break;
    case N_MOVIE_TEXTURE:      node = readMovieTextureNode();      break;
    case N_ELEVATION_GRID:     node = readElevationGridNode();     break;
    case N_EXTRUSION:          node = readExtrusionNode();         break;
    case N_SWITCH:             node = readSwitchNode();            break;
    case N_LOD:                node = readLODNode();               break;
    case N_COLLISION:          node = readCollisionNode();         break;
    case N_ANCHOR:             node = readAnchorNode();            break;
    case N_FOG:                node = readFogNode();               break;
    case N_BILLBOARD:          node = readBillboardNode();         break;
    case N_WORLD_INFO:         node = readWorldInfoNode();         break;
    case N_POINT_LIGHT:        node = readPointLightNode();        break;
    case N_DIRECTIONAL_LIGHT:  node = readDirectionalLightNode();  break;
    case N_SPOT_LIGHT:         node = readSpotLightNode();         break;

    case N_MATERIAL:           node = readMaterialNode();          break;
    case N_APPEARANCE:         node = readAppearanceNode();        break;
    case N_IMAGE_TEXTURE:      node = readImageTextureNode();      break;
    case N_TEXTURE_TRANSFORM:  node = readTextureTransformNode();  break;

    case N_BOX:                node = readBoxNode();               break;
    case N_CONE:               node = readConeNode();              break;
    case N_CYLINDER:           node = readCylinderNode();          break;
    case N_SPHERE:             node = readSphereNode();            break;
    case N_TEXT:               node = readTextNode();              break;
    case N_INDEXED_FACE_SET:   node = readIndexedFaceSetNode();    break;
    case N_INDEXED_LINE_SET:   node = readIndexedLineSetNode();    break;

    case N_COORDINATE:         node = readCoordNode();             break;
    case N_TEXTURE_COORDINATE: node = readTextureCoordinateNode(); break;
    case N_COLOR:              node = readColorNode();             break;
    case N_NORMAL:             node = readNormalNode();            break;
    case N_FONT_STYLE:         node = readFontStyleNode();         break;

        // unsupported nodes
    case N_AUDIO_CLIP:         node = skipUnsupportedNode("AudioClip");          break;
    case N_SOUND:              node = skipUnsupportedNode("Sound");              break;
    case N_COLOR_INTERPOLATOR: node = skipUnsupportedNode("ColorInterpolator");  break;
    case N_COORDINATE_INTERPOLATOR:  node = skipUnsupportedNode("CoordinateInterpolator");  break;
    case N_ORIENTATION_INTERPOLATOR: node = skipUnsupportedNode("OrientationInterpolator"); break;
    case N_NORMAL_INTERPOLATOR:      node = skipUnsupportedNode("NormalInterpolator");      break;
    case N_POSITION_INTERPOLATOR:    node = skipUnsupportedNode("PositionInterpolator");    break;
    case N_SCALAR_INTERPOLATOR:      node = skipUnsupportedNode("ScalarInterpolator");      break;
    case N_PLANE_SENSOR:       node = skipUnsupportedNode("PlaneSensor");        break;
    case N_PROXIMITY_SENSOR:   node = skipUnsupportedNode("ProximitySensor");    break;
    case N_SPHERE_SENSOR:      node = skipUnsupportedNode("SphereSensor");       break;
    case N_TIME_SENSOR:        node = skipUnsupportedNode("TimeSensor");         break;
    case N_TOUCH_SENSOR:       node = skipUnsupportedNode("TouchSensor");        break;
    case N_VISIBILITY_SENSOR:  node = skipUnsupportedNode("VisibilitySensor");   break;

    default: scanner->throwException
            (string("Node type \"") + nodeTypeName + "\" is not supported");

    }

    if(!node->isCategoryOf(nodeCategory)){
        scanner->throwException
            (string("A ") + nodeTypeName + " node is put in illegal place");
    }

    return node;
}


VRMLUnsupportedNodePtr VRMLParserImpl::skipUnsupportedNode(const std::string& nodeTypeName)
{
    while(true){
        if(!scanner->readQuotedString()){
            if(scanner->peekChar() == '}'){
                break;
            } if(!scanner->readChar()){
                scanner->throwException( "Node is not closed." );
            }
        }
    }
    return new VRMLUnsupportedNode( nodeTypeName );
}


VRMLUnsupportedNodePtr VRMLParserImpl::skipScriptNode()
{
    // '}' appears twice in "Script" node
    for(int i=0; i<1; i++){
        while(true){
            if(!scanner->readQuotedString()){
                if(scanner->peekChar() == '}'){
                    scanner->readChar();
                    break;
                }
                if(!scanner->readChar()){
                    scanner->throwException( "Script is not closed." );
                }
            }
        }
    }

    //	return new VRMLUnsupportedNode( "Script" );
    return NULL;
}


VRMLUnsupportedNodePtr VRMLParserImpl::skipExternProto()
{
    // read untill ']' appears
    while(true){
        if(!scanner->readQuotedString()){
            if( scanner->peekChar() == ']' ){
                // read found ']' and break this loop
                scanner->readChar();
                break;
            }
            if(!scanner->readChar()){
                scanner->throwException( "EXTERNPROTO is not closed." );
            }
        }
    }
    // read URL after ']'
    SFString url;
    readSFString( url );

    //	return new VRMLUnsupportedNode( "EXTERNPROTO" );
    return NULL;
}


VRMLNodePtr VRMLParserImpl::readInlineNode(VRMLNodeCategory nodeCategory)
{
    scanner->readChar('{');

    if(scanner->readSymbol() && scanner->symbolValue == F_URL){
        MFString inlineUrls;
        readMFString(inlineUrls);
        scanner->readCharEx('}', "syntax error 2");

        VRMLInlinePtr inlineNode = new VRMLInline();
        for(size_t i=0; i < inlineUrls.size(); ++i){
            string url(fromUTF8(inlineUrls[i]));
            if(boost::algorithm::iends_with(url, "wrl")){
                inlineNode->children.push_back(newInlineSource(url));
            } else {
                auto nonVrmlInline = new VRMLNonVrmlInline;
                nonVrmlInline->url = getRealPath(url);
                inlineNode->children.push_back(nonVrmlInline);
            }
            inlineNode->urls.push_back(inlineUrls[i]);
        }
        return inlineNode;
    }
    return 0;
}


string VRMLParserImpl::getRealPath(string url)
{ 
    if(!isFileProtocol(url)){
        return url;
    } else {
        stdx::filesystem::path path(stdx::filesystem::lexically_normal(removeURLScheme(url)));
        if(!checkAbsolute(path)){
            stdx::filesystem::path parentPath(scanner->filename);
            path = stdx::filesystem::lexically_normal(parentPath.parent_path() / path);
        }
        return getAbsolutePathString(path);
    }
}


VRMLNodePtr VRMLParserImpl::newInlineSource(string& io_filename)
{
    string chkFile = getRealPath(io_filename);
    for(list<string>::const_iterator p = ancestorPathsList.begin(); p != ancestorPathsList.end(); ++p){
        if(*p == chkFile){
            scanner->throwException("Infinity loop ! " + chkFile + " is included ancestor list");
        }
    }

    VRMLParserImpl inlineParser(*this, ancestorPathsList);

    inlineParser.load(chkFile, false);
    io_filename = chkFile;

    VRMLGroupPtr group = new VRMLGroup();
    while(VRMLNodePtr node = inlineParser.readNode(TOP_NODE)){
        if(node->isCategoryOf(CHILD_NODE)){
            group->children.push_back(node);
        }
    }
    inlineParser.checkEOF();

    if(group->children.size() == 1){
        return group->children.front();
    } else {
        return group;
    }
}


VRMLProtoPtr VRMLParserImpl::defineProto()
{
    string proto_name = scanner->readWordEx("illegal PROTO name");
    scanner->readCharEx('[', "syntax error 3");

    VRMLProtoPtr proto = new VRMLProto(proto_name);

    while(!scanner->readChar(']')){
        int event_type = scanner->readSymbolEx("illegal field event type");
        int field_symbol = scanner->readSymbolEx("illegal field type");
        string field_name = scanner->readWordEx("syntax error 4");

        // insert a new empty field and get it, contents of which will be set below
        VRMLVariantField& field = proto->fields.insert(
            VRMLProtoFieldPair(field_name, VRMLVariantField())).first->second;

        switch(event_type){

        case E_FIELD:
        case E_EXPOSED_FIELD:

            switch(field_symbol){

            case T_SFINT32:    field = SFInt32();    readSFInt32(stdx::get<SFInt32>(field));         break;
            case T_SFFLOAT:    field = SFFloat();    readSFFloat(stdx::get<SFFloat>(field));         break;
            case T_MFINT32:    field = MFInt32();    readMFInt32(stdx::get<MFInt32>(field));         break;
            case T_MFFLOAT:    field = MFFloat();    readMFFloat(stdx::get<MFFloat>(field));         break;
            case T_SFVEC3F:    field = SFVec3f();    readSFVec3f(stdx::get<SFVec3f>(field));         break;
            case T_MFVEC3F:    field = MFVec3f();    readMFVec3f(stdx::get<MFVec3f>(field));         break;
            case T_SFCOLOR:    field = SFColor();    readSFColor(stdx::get<SFColor>(field));         break;
            case T_MFCOLOR:    field = MFColor();    readMFColor(stdx::get<MFColor>(field));         break;
            case T_SFSTRING:   field = SFString();   readSFString(stdx::get<SFString>(field));       break;
            case T_MFSTRING:   field = MFString();   readMFString(stdx::get<MFString>(field));       break;
            case T_SFROTATION: field = SFRotation(); readSFRotation(stdx::get<SFRotation>(field));   break;
            case T_MFROTATION: field = MFRotation(); readMFRotation(stdx::get<MFRotation>(field));   break;
            case T_SFBOOL:     field = SFBool();     readSFBool(stdx::get<SFBool>(field));           break;
            case T_SFNODE:     field = SFNode();     readSFNode(stdx::get<SFNode>(field), ANY_NODE); break;
            case T_MFNODE:     field = MFNode();     readMFNode(stdx::get<MFNode>(field), ANY_NODE); break;
            case T_SFIMAGE:    field = SFImage();    readSFImage(stdx::get<SFImage>(field));         break;

            default: scanner->throwException("illegal field type");
            }
            break;

        case E_EVENTIN:
        case E_EVENTOUT:

            switch(field_symbol){

            case T_SFINT32:    field = SFInt32();    break;
            case T_MFINT32:    field = MFInt32();    break;
            case T_SFFLOAT:    field = SFFloat();    break;
            case T_MFFLOAT:    field = MFFloat();    break;
            case T_SFVEC3F:    field = SFVec3f();    break;
            case T_MFVEC3F:    field = MFVec3f();    break;
            case T_SFCOLOR:    field = SFColor();    break;
            case T_MFCOLOR:    field = MFColor();    break;
            case T_SFSTRING:   field = SFString();   break;
            case T_MFSTRING:   field = MFString();   break;
            case T_SFROTATION: field = SFRotation(); break;
            case T_MFROTATION: field = MFRotation(); break;
            case T_SFBOOL:     field = SFBool();     break;
            case T_SFNODE:     field = SFNode();     break;
            case T_MFNODE:     field = MFNode();     break;
            case T_SFIMAGE:    field = SFImage();    break;
            }
            break;

        default: scanner->throwException("illegal field event type");

        }
    }

    scanner->readCharEx('{', "A PROTO definition has no entity");
    char* begin = scanner->text;
    int brace_level = 1;
    while(true){
        int token = scanner->readToken();
        if(token == EasyScanner::T_NONE){
            scanner->throwException("syntax error 5");
        } else if(token == EasyScanner::T_SIGLUM){
            if(scanner->charValue == '{') {
                brace_level++;
            } else if(scanner->charValue == '}') {
                brace_level--;
                if(brace_level==0)
                    break;
            }
        }
    }

    std::shared_ptr<EasyScanner> entityScanner = std::make_shared<EasyScanner>(*scanner, false);
    entityScanner->setText(begin, scanner->text - begin - 1);
    entityScanner->setLineNumberOffset(scanner->lineNumber);

    protoToEntityScannerMap[proto.get()] = entityScanner;

    protoMap.insert(TProtoPair(proto_name, proto));

    return proto;
}


VRMLProtoInstancePtr VRMLParserImpl::readProtoInstanceNode(const string& proto_name, VRMLNodeCategory nodeCategory, const string& defName)
{
    TProtoMap::iterator p = protoMap.find(proto_name);
    if(p == protoMap.end()){
        scanner->throwException("undefined node");
    }

    VRMLProtoPtr proto = p->second;
    VRMLProtoInstancePtr protoInstance(new VRMLProtoInstance(proto));

    while(scanner->readWord()){
        VRMLProtoFieldMap::iterator p = protoInstance->fields.find(scanner->stringValue);
        if(p == protoInstance->fields.end())
            scanner->throwException("undefined field");

        VRMLVariantField& field = p->second;

        switch(stdx::get_variant_index(field)){

        case SFINT32:    readSFInt32(stdx::get<SFInt32>(field));         break;
        case MFINT32:    readMFInt32(stdx::get<MFInt32>(field));         break;
        case SFFLOAT:    readSFFloat(stdx::get<SFFloat>(field));         break;
        case MFFLOAT:    readMFFloat(stdx::get<MFFloat>(field));         break;
        case SFVEC2F:    readSFVec2f(stdx::get<SFVec2f>(field));         break;
        case MFVEC2F:    readMFVec2f(stdx::get<MFVec2f>(field));         break;
        case SFVEC3F:    readSFVec3f(stdx::get<SFVec3f>(field));         break;
        case MFVEC3F:    readMFVec3f(stdx::get<MFVec3f>(field));         break;
        case SFCOLOR:    readSFColor(stdx::get<SFColor>(field));         break;
        case MFCOLOR:    readMFColor(stdx::get<MFColor>(field));         break;
        case SFSTRING:   readSFString(stdx::get<SFString>(field));       break;
        case MFSTRING:   readMFString(stdx::get<MFString>(field));       break;
        case SFROTATION: readSFRotation(stdx::get<SFRotation>(field));   break;
        case MFROTATION: readMFRotation(stdx::get<MFRotation>(field));   break;
        case SFBOOL:     readSFBool(stdx::get<SFBool>(field));           break;
        case SFNODE:     readSFNode(stdx::get<SFNode>(field), ANY_NODE); break;
        case MFNODE:     readMFNode(stdx::get<MFNode>(field), ANY_NODE); break;
        case SFIMAGE:    readSFImage(stdx::get<SFImage>(field));         break;
        default:
            break;
        }
    }

    if(protoInstanceActualNodeExtractionMode){
        protoInstance->actualNode = evalProtoInstance(protoInstance, nodeCategory, defName);
    }

    return protoInstance;
}


VRMLNodePtr VRMLParserImpl::evalProtoInstance(VRMLProtoInstancePtr protoInstance, VRMLNodeCategory nodeCategory, const string& defName)
{
    EasyScanner* orgScanner = scanner;
    ProtoToEntityScannerMap::iterator p;
    p = protoToEntityScannerMap.find(protoInstance->proto.get());
    if(p == protoToEntityScannerMap.end()){
        scanner->throwException("Undefined proto node instance");
    }
    scanner = p->second.get();
    scanner->moveToHead();

    VRMLProtoInstancePtr orgProtoInstance = currentProtoInstance;
    currentProtoInstance = protoInstance;
    
    VRMLNodePtr node = readNode(nodeCategory);
    if(node){
        node->defName = defName;
    }
    
    scanner = orgScanner;
    currentProtoInstance = orgProtoInstance;
    
    return node;
}


VRMLViewpointPtr VRMLParserImpl::readViewpointNode()
{
    VRMLViewpointPtr node(new VRMLViewpoint);

    while(scanner->readSymbol()){

        switch(scanner->symbolValue){
        case F_FIELD_OF_VIEW: readSFFloat(node->fieldOfView);    break;
        case F_JUMP:          readSFBool(node->jump);            break;
        case F_ORIENTATION:   readSFRotation(node->orientation); break;
        case F_POSITION:      readSFVec3f(node->position);       break;
        case F_DESCRIPTION:   readSFString(node->description);   break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLNavigationInfoPtr VRMLParserImpl::readNavigationInfoNode()
{
    VRMLNavigationInfoPtr node(new VRMLNavigationInfo);

    while(scanner->readSymbol()){

        switch(scanner->symbolValue){
        case F_AVATAR_SIZE:      readMFFloat(node->avatarSize);      break;
        case F_HEADLIGHT:        readSFBool(node->headlight);        break;
        case F_SPEED:            readSFFloat(node->speed);           break;
        case F_TYPE:             readMFString(node->type);           break;
        case F_VISIBILITY_LIMIT: readSFFloat(node->visibilityLimit); break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLBackgroundPtr VRMLParserImpl::readBackgroundNode()
{
    VRMLBackgroundPtr node(new VRMLBackground);

    while(scanner->readSymbol()){

        switch(scanner->symbolValue){
        case F_GROUND_ANGLE: readMFFloat(node->groundAngle); break;
        case F_GROUND_COLOR: readMFColor(node->groundColor); break;
        case F_SKY_ANGLE:    readMFFloat(node->skyAngle);    break;
        case F_SKY_COLOR:    readMFColor(node->skyColor);    break;
        case F_BACK_URL:     readMFString(node->backUrl);    break;
        case F_BOTTOM_URL:   readMFString(node->bottomUrl);  break;
        case F_FRONT_URL:    readMFString(node->frontUrl);   break;
        case F_LEFT_URL:     readMFString(node->leftUrl);    break;
        case F_RIGHT_URL:    readMFString(node->rightUrl);   break;
        case F_TOP_URL:      readMFString(node->topUrl);     break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLGroupPtr VRMLParserImpl::readGroupNode()
{
    VRMLGroupPtr node(new VRMLGroup);

    while(scanner->readSymbol()){

        switch(scanner->symbolValue){

        case F_BBOX_CENTER:     readSFVec3f(node->bboxCenter);          break;
        case F_BBOX_SIZE:       readSFVec3f(node->bboxSize);            break;
        case F_CHILDREN:        readMFNode(node->children, CHILD_NODE); break;

        case F_ADD_CHILDREN:
        case F_REMOVE_CHILDREN:
        {
            MFNode dummy;
            readMFNode(dummy, CHILD_NODE);
        }
        break;

        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLTransformPtr VRMLParserImpl::readTransformNode()
{
    VRMLTransformPtr node(new VRMLTransform);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_CENTER:            readSFVec3f(node->center);              break;
        case F_ROTATION:          readSFRotation(node->rotation);         break;
        case F_SCALE:             readSFVec3f(node->scale);               break;
        case F_SCALE_ORIENTATION: readSFRotation(node->scaleOrientation); break;
        case F_TRANSLATION:       readSFVec3f(node->translation);         break;
        case F_BBOX_CENTER:       readSFVec3f(node->bboxCenter);          break;
        case F_BBOX_SIZE:         readSFVec3f(node->bboxSize);            break;
        case F_CHILDREN:          readMFNode(node->children, CHILD_NODE); break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLShapePtr VRMLParserImpl::readShapeNode()
{
    VRMLShapePtr node(new VRMLShape);

    while(scanner->readSymbol()){

        switch(scanner->symbolValue){

        case F_APPEARANCE:
            node->appearance =
                dynamic_pointer_cast<VRMLAppearance>(readSFNode(APPEARANCE_NODE));
            break;
        case F_GEOMETRY:
            node->geometry = readSFNode(GEOMETRY_NODE);
            break;

        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLCylinderSensorPtr VRMLParserImpl::readCylinderSensorNode()
{
    VRMLCylinderSensorPtr node(new VRMLCylinderSensor);

    while(scanner->readSymbol()){

        switch(scanner->symbolValue){
        case F_AUTO_OFFSET: readSFBool(node->autoOffset); break;
        case F_DISK_ANGLE:  readSFFloat(node->diskAngle); break;
        case F_ENABLED:     readSFBool(node->enabled);    break;
        case F_MAX_ANGLE:   readSFFloat(node->maxAngle);  break;
        case F_MIN_ANGLE:   readSFFloat(node->minAngle);  break;
        case F_OFFSET:      readSFFloat(node->offset);    break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLPointSetPtr VRMLParserImpl::readPointSetNode()
{
    VRMLPointSetPtr node(new VRMLPointSet);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_COORD:
            node->coord = dynamic_pointer_cast<VRMLCoordinate>(readSFNode(COORDINATE_NODE));
            break;
            
        case F_COLOR:
            node->color = dynamic_pointer_cast<VRMLColor>(readSFNode(COLOR_NODE));
            break;
            
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLPixelTexturePtr VRMLParserImpl::readPixelTextureNode()
{
    VRMLPixelTexturePtr node(new VRMLPixelTexture);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_IMAGE:    readSFImage(node->image);  break;
        case F_REPEAT_S: readSFBool(node->repeatS); break;
        case F_REPEAT_T: readSFBool(node->repeatT); break;
        default:
            scanner->throwException("Undefined field");
        }
    }
    
    return node;
}


VRMLMovieTexturePtr VRMLParserImpl::readMovieTextureNode()
{
    VRMLMovieTexturePtr node(new VRMLMovieTexture);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_URL:        readMFString(node->url);     break;
        case F_LOOP:       readSFBool(node->loop);      break;
        case F_SPEED:      readSFFloat(node->speed);    break;
        case F_START_TIME: readSFTime(node->startTime); break;
        case F_STOP_TIME:  readSFTime(node->stopTime);  break;
        case F_REPEAT_S:   readSFBool(node->repeatS);   break;
        case F_REPEAT_T:   readSFBool(node->repeatT);   break;
        default:
            scanner->throwException("Undefined field");
        }
    }
    
    convertUrl(node->url);

    return node;
}


VRMLElevationGridPtr VRMLParserImpl::readElevationGridNode()
{
    VRMLElevationGridPtr node(new VRMLElevationGrid);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_X_DIMENSION:       readSFInt32(node->xDimension);     break;
        case F_Z_DIMENSION:       readSFInt32(node->zDimension);     break;
        case F_X_SPACING:         readSFFloat(node->xSpacing);       break;
        case F_Z_SPACING:         readSFFloat(node->zSpacing);       break;
        case F_HEIGHT:            readMFFloat(node->height);         break;
        case F_CCW:               readSFBool(node->ccw);             break;
        case F_COLOR_PER_VERTEX:  readSFBool(node->colorPerVertex);  break;
        case F_CREASE_ANGLE:      readSFFloat(node->creaseAngle);    break;
        case F_NORMAL_PER_VERTEX: readSFBool(node->normalPerVertex); break;
        case F_SOLID:             readSFBool(node->solid);           break;
            
        case F_COLOR:
            node->color = dynamic_pointer_cast<VRMLColor>(readSFNode(COLOR_NODE));
            break;
        case F_NORMAL:
            node->normal = dynamic_pointer_cast<VRMLNormal>(readSFNode(NORMAL_NODE));
            break;
        case F_TEX_COORD:
            node->texCoord =
                dynamic_pointer_cast<VRMLTextureCoordinate>(readSFNode(TEXTURE_COORDINATE_NODE));
            break;
            
        default:
            scanner->throwException("Undefined field");
        }
    }
    
    return node;
}


VRMLExtrusionPtr VRMLParserImpl::readExtrusionNode()
{
    VRMLExtrusionPtr node(new VRMLExtrusion);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_CROSS_SECTION: readMFVec2f(node->crossSection);   break;
        case F_SPINE:         readMFVec3f(node->spine);          break;
        case F_SCALE:         readMFVec2f(node->scale);          break;
        case F_ORIENTATION:   readMFRotation(node->orientation); break;
        case F_BEGIN_CAP:     readSFBool(node->beginCap);        break;
        case F_END_CAP:       readSFBool( node->endCap );        break;
        case F_SOLID:         readSFBool( node->solid );         break;
        case F_CCW:           readSFBool( node->ccw );           break;
        case F_CONVEX:        readSFBool( node->convex );        break;
        case F_CREASE_ANGLE:  readSFFloat( node->creaseAngle );  break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}


VRMLSwitchPtr VRMLParserImpl::readSwitchNode()
{
    VRMLSwitchPtr node( new VRMLSwitch );

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_CHOICE:        readMFNode(node->choice, CHILD_NODE); break;
        case F_WHICH_CHOICE:  readSFInt32(node->whichChoice);       break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}


VRMLLODPtr VRMLParserImpl::readLODNode()
{
    VRMLLODPtr node( new VRMLLOD );

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_RANGE:  readMFFloat(node->range);          break;
        case F_CENTER: readSFVec3f(node->center);         break;
        case F_LEVEL:  readMFNode(node->level, ANY_NODE); break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLCollisionPtr VRMLParserImpl::readCollisionNode()
{
    VRMLCollisionPtr node(new VRMLCollision);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_COLLIDE:     readSFBool(node->collide);              break;
        case F_CHILDREN:    readMFNode(node->children, CHILD_NODE); break;
        case F_PROXY:       readSFNode(node->proxy, SHAPE_NODE);    break;
        case F_BBOX_CENTER: readSFVec3f(node->bboxCenter);          break;
        case F_BBOX_SIZE:   readSFVec3f(node->bboxSize);            break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}


VRMLAnchorPtr VRMLParserImpl::readAnchorNode()
{
    VRMLAnchorPtr node(new VRMLAnchor);

    while(scanner->readSymbol()){
        switch( scanner->symbolValue){
        case F_CHILDREN:    readMFNode(node->children, CHILD_NODE); break;
        case F_DESCRIPTION: readSFString(node->description);        break;
        case F_PARAMETER:   readMFString(node->parameter);          break;
        case F_URL:         readMFString(node->url);                break;
        case F_BBOX_CENTER: readSFVec3f(node->bboxCenter);          break;
        case F_BBOX_SIZE:   readSFVec3f(node->bboxSize);            break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}


VRMLFogPtr VRMLParserImpl::readFogNode()
{
    VRMLFogPtr node(new VRMLFog);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_COLOR:            readSFColor(node->color);           break;
        case F_VISIBILITY_RANGE: readSFFloat(node->visibilityRange); break;
        case F_FOG_TYPE:         readSFString(node->fogType);        break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}


VRMLBillboardPtr VRMLParserImpl::readBillboardNode()
{
    VRMLBillboardPtr node( new VRMLBillboard );

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_AXIS_OF_ROTATION: readSFVec3f(node->axisOfRotation);      break;
        case F_CHILDREN:         readMFNode(node->children, CHILD_NODE); break;
        case F_BBOX_CENTER:      readSFVec3f(node->bboxCenter);          break;
        case F_BBOX_SIZE:        readSFVec3f(node->bboxSize);            break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}


VRMLWorldInfoPtr VRMLParserImpl::readWorldInfoNode()
{
    VRMLWorldInfoPtr node(new VRMLWorldInfo);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_TITLE: readSFString(node->title); break;
        case F_INFO:  readMFString(node->info);  break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLPointLightPtr VRMLParserImpl::readPointLightNode()
{
    VRMLPointLightPtr node( new VRMLPointLight );

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_LOCATION:          readSFVec3f(node->location);         break;
        case F_ON:                readSFBool(node->on);                break;
        case F_INTENSITY:         readSFFloat(node->intensity);        break;
        case F_COLOR:             readSFColor(node->color);            break;
        case F_RADIUS:            readSFFloat(node->radius);           break;
        case F_AMBIENT_INTENSITY: readSFFloat(node->ambientIntensity); break;
        case F_ATTENUATION:       readSFVec3f(node->attenuation);      break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}



VRMLDirectionalLightPtr VRMLParserImpl::readDirectionalLightNode()
{
    VRMLDirectionalLightPtr node(new VRMLDirectionalLight);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_DIRECTION:         readSFVec3f(node->direction);        break;
        case F_ON:                readSFBool(node->on);                break;
        case F_INTENSITY:         readSFFloat(node->intensity);        break;
        case F_COLOR:             readSFColor(node->color);            break;
        case F_AMBIENT_INTENSITY: readSFFloat(node->ambientIntensity); break;
        default:
            scanner->throwException("Undefined field");
        }
    }
    
    return node;
}


VRMLSpotLightPtr VRMLParserImpl::readSpotLightNode()
{
    VRMLSpotLightPtr node(new VRMLSpotLight);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_LOCATION:          readSFVec3f(node->location);         break;
        case F_DIRECTION:         readSFVec3f(node->direction);        break;
        case F_ON:                readSFBool(node->on);                break;
        case F_COLOR:             readSFColor(node->color);            break;
        case F_INTENSITY:         readSFFloat(node->intensity);        break;
        case F_RADIUS:            readSFFloat(node->radius);           break;
        case F_AMBIENT_INTENSITY: readSFFloat(node->ambientIntensity); break;
        case F_ATTENUATION:       readSFVec3f(node->attenuation);      break;
        case F_BEAM_WIDTH:        readSFFloat(node->beamWidth);        break;
        case F_CUT_OFF_ANGLE:     readSFFloat(node->cutOffAngle);      break;
        default:
            scanner->throwException( "Undefined field" );
        }
    }
    
    return node;
}


VRMLBoxPtr VRMLParserImpl::readBoxNode()
{
    VRMLBoxPtr node(new VRMLBox);

    if(scanner->readSymbol()){
        if(scanner->symbolValue != F_SIZE)
            scanner->throwException("Undefined field");
        readSFVec3f(node->size);
    }

    return node;
}


VRMLConePtr VRMLParserImpl::readConeNode()
{
    VRMLConePtr node(new VRMLCone);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_BOTTOM:        readSFBool(node->bottom);        break;
        case F_BOTTOM_RADIUS: readSFFloat(node->bottomRadius); break;
        case F_HEIGHT:        readSFFloat(node->height);       break;
        case F_SIDE:          readSFBool(node->side);          break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLCylinderPtr VRMLParserImpl::readCylinderNode()
{
    VRMLCylinderPtr node(new VRMLCylinder);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_BOTTOM: readSFBool(node->bottom);  break;
        case F_HEIGHT: readSFFloat(node->height); break;
        case F_RADIUS: readSFFloat(node->radius); break;
        case F_SIDE:   readSFBool(node->side);    break;
        case F_TOP:    readSFBool(node->top);     break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLSpherePtr VRMLParserImpl::readSphereNode()
{
    VRMLSpherePtr node(new VRMLSphere);

    if(scanner->readSymbol()){
        if(scanner->symbolValue != F_RADIUS)
            scanner->throwException("Undefined field");
        readSFFloat(node->radius);
    }

    return node;
}


VRMLTextPtr VRMLParserImpl::readTextNode()
{
    VRMLTextPtr node(new VRMLText);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_STRING:     readMFString(node->fstring);  break;
        case F_LENGTH:     readMFFloat(node->length);    break;
        case F_MAX_EXTENT: readSFFloat(node->maxExtent); break;
        case F_FONT_STYLE:
            node->fontStyle = dynamic_pointer_cast<VRMLFontStyle>(readSFNode(FONT_STYLE_NODE));
            break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLFontStylePtr VRMLParserImpl::readFontStyleNode()
{
    VRMLFontStylePtr node(new VRMLFontStyle);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_FAMILY:        readMFString(node->family);    break;
        case F_HORIZONTAL:    readSFBool(node->horizontal);  break;
        case F_JUSTIFY:       readMFString(node->justify);   break;
        case F_LANGUAGE:      readSFString(node->language);  break;
        case F_LEFT_TO_RIGHT: readSFBool(node->leftToRight); break;
        case F_SIZE:          readSFFloat(node->size);       break;
        case F_SPACING:       readSFFloat(node->spacing);    break;
        case F_STYLE:         readSFString(node->style);     break;
        case F_TOP_TO_BOTTOM: readSFBool(node->topToBottom); break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLIndexedLineSetPtr VRMLParserImpl::readIndexedLineSetNode()
{
    VRMLIndexedLineSetPtr node(new VRMLIndexedLineSet);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){

        case F_COLOR:
            node->color = dynamic_pointer_cast<VRMLColor>(readSFNode(COLOR_NODE));
            break;
            
        case F_COORD:
            node->coord = dynamic_pointer_cast<VRMLCoordinate>(readSFNode(COORDINATE_NODE));
            break;
            
        case F_COLOR_INDEX:      readMFInt32(node->colorIndex);    break;
        case F_COLOR_PER_VERTEX: readSFBool(node->colorPerVertex); break;
        case F_COORD_INDEX:      readMFInt32(node->coordIndex);    break;

        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLIndexedFaceSetPtr VRMLParserImpl::readIndexedFaceSetNode()
{
    VRMLIndexedFaceSetPtr node(new VRMLIndexedFaceSet);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){

        case F_COLOR:
            node->color = dynamic_pointer_cast<VRMLColor>(readSFNode(COLOR_NODE));
            break;

        case F_COORD:
            node->coord = dynamic_pointer_cast<VRMLCoordinate>(readSFNode(COORDINATE_NODE));
            break;

        case F_COLOR_INDEX:       readMFInt32(node->colorIndex);     break;
        case F_COLOR_PER_VERTEX:  readSFBool(node->colorPerVertex);  break;
        case F_COORD_INDEX:       readMFInt32(node->coordIndex);     break;
        case F_CCW:               readSFBool(node->ccw);             break;
        case F_CONVEX:            readSFBool(node->convex);          break;
        case F_SOLID:             readSFBool(node->solid);           break;
        case F_CREASE_ANGLE:      readSFFloat(node->creaseAngle);    break;
        case F_NORMAL_INDEX:      readMFInt32(node->normalIndex);    break;
        case F_NORMAL_PER_VERTEX: readSFBool(node->normalPerVertex); break;
        case F_TEX_COORD_INDEX:   readMFInt32(node->texCoordIndex);  break;

        case F_TEX_COORD:
            node->texCoord =
                dynamic_pointer_cast<VRMLTextureCoordinate>(readSFNode(TEXTURE_COORDINATE_NODE));
            break;
            
        case F_NORMAL:
            node->normal = dynamic_pointer_cast<VRMLNormal>(readSFNode(NORMAL_NODE));
            break;

        default:
            scanner->throwException("Undefined field");
        }
    }

    //checkIndexedFaceSet(node);

    return node;
}


void VRMLParserImpl::checkIndexedFaceSet(VRMLIndexedFaceSetPtr node)
{
    MFInt32& index = node->coordIndex;
    MFVec3s& coord = node->coord->point;

    int numUsedVertices = 0;
    vector<bool> usedVertices(coord.size(), false);

    int n = index.size();

    int i = 0;
    vector<int> polygon;
    while(i < n){
        polygon.resize(0);
        bool isSeparated = false;
        while(i < n){
            if(index[i] < 0){
                isSeparated = true;
                i++;
                break;
            }
            polygon.push_back(index[i]);
            if(!usedVertices[index[i]]){
                usedVertices[index[i]] = true;
                numUsedVertices++;
            }
            i++;
        }

        const int numVertices = polygon.size();

        if(numVertices < 3){
            os() << "Number of vertices is less than 3 !" << endl;
        }
        if(numVertices > 3){
            os() << "Polygon is not a triangle in ";
            os() << scanner->filename << endl;
            for(int j=0; j < numVertices; j++){
                os() << polygon[j] << ",";
            }
            os() << endl;
        }
        if(!isSeparated){
            os() << "Vertex index is not correctly separated by '-1'" << endl;
        }

        int n = coord.size();
        for(int j=0; j < numVertices; j++){
            if(polygon[j] >= n){
                os() << "index " << polygon[j] << " is over the number of vertices" << endl;
            }
        }

        bool isIndexOverlapped = false;
        bool isVertexOverlapped = false;
        for(int j = 0; j < numVertices - 1; j++){
            for(int k = j+1; k < numVertices; k++){
                if(polygon[j] == polygon[k]){
                    isIndexOverlapped = true;
                }
                SFVec3s& v1 = coord[polygon[j]];
                SFVec3s& v2 = coord[polygon[k]];
                if(v1 == v2){
                    isVertexOverlapped = true;
                }
            }
        }
        if(isIndexOverlapped){
            os() << "overlapped vertex index in one polygon: ";
            for(int l = 0; l < numVertices; l++){
                os() << polygon[l] << ",";
            }
            os() << endl;
        }

        if(isVertexOverlapped){
            os() << "In " << scanner->filename << ":";
            os() << "two vertices in one polygon have the same position\n";

            for(int l = 0; l < numVertices; l++){
                SFVec3s& v = coord[polygon[l]];
                os() << polygon[l] << " = (" << v[0] << "," << v[1] << "," << v[2] << ") ";
            }
            os() << endl;
        }
    }

    if(numUsedVertices < static_cast<int>(coord.size())){
        os() << "There are vertices which are not used in" << scanner->filename << ".\n";
        os() << "Number of vertices is " << coord.size();
        os() << ", Number of used ones is " << numUsedVertices << endl;
    }
}


VRMLCoordinatePtr VRMLParserImpl::readCoordNode()
{
    VRMLCoordinatePtr node(new VRMLCoordinate);

    if(scanner->readSymbol()){
        if(scanner->symbolValue != F_POINT){
            scanner->throwException("Undefined field");
        }
        readMFVec3s(node->point);
    }

    return node;
}


VRMLTextureCoordinatePtr VRMLParserImpl::readTextureCoordinateNode()
{
    VRMLTextureCoordinatePtr node(new VRMLTextureCoordinate);

    if(scanner->readSymbol()){
        if(scanner->symbolValue != F_POINT){
            scanner->throwException("Undefined field");
        }
        readMFVec2s(node->point);
    }

    return node;
}


VRMLColorPtr VRMLParserImpl::readColorNode()
{
    VRMLColorPtr node(new VRMLColor);

    if(scanner->readSymbol()){
        if(scanner->symbolValue != F_COLOR){
            scanner->throwException("Undefined field");
        }
        readMFColor(node->color);
    }

    return node;
}


VRMLNormalPtr VRMLParserImpl::readNormalNode()
{
    VRMLNormalPtr node(new VRMLNormal);

    if(scanner->readSymbol()){
        if(scanner->symbolValue != F_VECTOR){
            scanner->throwException("Undefined field");
        }
        readMFVec3s(node->vector);
    }

    return node;
}


VRMLAppearancePtr VRMLParserImpl::readAppearanceNode()
{
    VRMLAppearancePtr node(new VRMLAppearance);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){

        case F_MATERIAL:
            node->material = dynamic_pointer_cast<VRMLMaterial>(readSFNode(MATERIAL_NODE));
            break;
            
        case F_TEXTURE:
            node->texture = dynamic_pointer_cast<VRMLTexture>(readSFNode(TEXTURE_NODE));
            break;
            
        case F_TEXTURE_TRANSFORM:
            node->textureTransform =
                dynamic_pointer_cast<VRMLTextureTransform>(readSFNode(TEXTURE_TRANSFORM_NODE));
            break;

        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLMaterialPtr VRMLParserImpl::readMaterialNode()
{
    VRMLMaterialPtr node(new VRMLMaterial);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_AMBIENT_INTENSITY: readSFFloat(node->ambientIntensity); break;
        case F_DIFFUSE_COLOR:     readSFColor(node->diffuseColor);     break;
        case F_EMISSIVE_COLOR:    readSFColor(node->emissiveColor);    break;
        case F_SHININESS:         readSFFloat(node->shininess);        break;
        case F_SPECULAR_COLOR:    readSFColor(node->specularColor);    break;
        case F_TRANSPARANCY:      readSFFloat(node->transparency);     break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLImageTexturePtr VRMLParserImpl::readImageTextureNode()
{
    VRMLImageTexturePtr node = new VRMLImageTexture;

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_URL:      readMFString(node->url);   break;
        case F_REPEAT_S: readSFBool(node->repeatS); break;
        case F_REPEAT_T: readSFBool(node->repeatT); break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    convertUrl(node->url);

    return node;
}


VRMLTextureTransformPtr VRMLParserImpl::readTextureTransformNode()
{
    VRMLTextureTransformPtr node(new VRMLTextureTransform);

    while(scanner->readSymbol()){
        switch(scanner->symbolValue){
        case F_CENTER:      readSFVec2f(node->center);      break;
        case F_ROTATION:    readSFFloat(node->rotation);    break;
        case F_SCALE:       readSFVec2f(node->scale);       break;
        case F_TRANSLATION: readSFVec2f(node->translation); break;
        default:
            scanner->throwException("Undefined field");
        }
    }

    return node;
}


VRMLVariantField& VRMLParserImpl::readProtoField(VRMLFieldTypeId fieldTypeId)
{
    if(!currentProtoInstance){
        scanner->throwException("cannot use proto field value here");
    }
    scanner->readWordEx("illegal field");
    VRMLProtoFieldMap::iterator p = currentProtoInstance->fields.find(scanner->stringValue);

    if(p == currentProtoInstance->fields.end()){
        string msg = "This field(";
        msg += scanner->stringValue +") does not exist in proto node";
        scanner->throwException(msg);
    }
    if(stdx::get_variant_index(p->second) != fieldTypeId){
        scanner->throwException("Unmatched field type");
    }

    return p->second;
}


void VRMLParserImpl::readSFInt32(SFInt32& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFInt32>(readProtoField(SFINT32));
    } else {
        out_value = scanner->readIntEx("illegal int value");
    }
}


void VRMLParserImpl::readMFInt32(MFInt32& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFInt32>(readProtoField(MFINT32));
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(scanner->readIntEx("illegal int value"));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(scanner->readIntEx("illegal int value"));
            }
        }
    }
}


void VRMLParserImpl::readSFFloat(SFFloat& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFFloat>(readProtoField(SFFLOAT));
    } else {
        out_value = scanner->readDoubleEx("illegal float value");
    }
}


void VRMLParserImpl::readMFFloat(MFFloat& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFFloat>(readProtoField(MFFLOAT));
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(scanner->readDoubleEx("illegal float value"));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(scanner->readDoubleEx("illegal float value"));
            }
        }
    }
}


static inline SFColor readSFColor(EasyScanner* scanner)
{
    SFColor c;
    for(int i=0; i < 3; ++i){
        c[i] = static_cast<SFColor::Scalar>(scanner->readDoubleEx("illegal color element"));
    }
    return c;
}
    

void VRMLParserImpl::readSFColor(SFColor& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFColor>(readProtoField(SFCOLOR));
    } else {
        out_value = ::readSFColor(scanner);
    }
}


void VRMLParserImpl::readMFColor(MFColor& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFColor>(readProtoField(MFCOLOR));
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(::readSFColor(scanner));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(::readSFColor(scanner));
            }
        }
    }
}


void VRMLParserImpl::readSFString(SFString& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFString>(readProtoField(SFSTRING));
    } else {
        out_value = scanner->readQuotedStringEx("illegal string");
    }
}


void VRMLParserImpl::readMFString(MFString& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFString>(readProtoField(MFSTRING));
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(scanner->readQuotedStringEx("illegal string"));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(scanner->readQuotedStringEx("illegal string"));
            }
        }
    }
}


static inline SFVec2f readSFVec2f(EasyScanner* scanner)
{
    SFVec2f v;
    for(int i=0; i < 2; ++i){
        v[i] = scanner->readDoubleEx("illegal vector element");
    }
    return v;
}


inline void VRMLParserImpl::readSFVec2f(SFVec2f& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFVec2f>(readProtoField(SFVEC2F));
    } else {
        out_value = ::readSFVec2f(scanner);
    }
}


void VRMLParserImpl::readMFVec2f(MFVec2f& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFVec2f>(readProtoField(MFVEC2F));
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(::readSFVec2f(scanner));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(::readSFVec2f(scanner));
            }
        }
    }
}


static inline SFVec2s readSFVec2s(EasyScanner* scanner)
{
    SFVec2s v;
    for(int i=0; i < 2; ++i){
        v[i] = scanner->readFloatEx("illegal vector element");
    }
    return v;
}


inline void VRMLParserImpl::readSFVec2s(SFVec2s& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFVec2f>(readProtoField(SFVEC2F)).cast<SFVec2s::Scalar>();
    } else {
        out_value = ::readSFVec2s(scanner);
    }
}


void VRMLParserImpl::readMFVec2s(MFVec2s& out_value)
{
    if(scanner->readSymbol(F_IS)){
        const MFVec2f& value = stdx::get<MFVec2f>(readProtoField(MFVEC2F));
        const size_t n = value.size();
        out_value.resize(n);
        for(size_t i=0; i < n; ++i){
            out_value[i] = value[i].cast<SFVec2s::Scalar>();
        }
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(::readSFVec2s(scanner));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(::readSFVec2s(scanner));
            }
        }
    }
}


static inline SFVec3f readSFVec3f(EasyScanner* scanner)
{
    SFVec3f v;
    for(int i=0; i < 3; ++i){
        v[i] = scanner->readDoubleEx("illegal vector element");
    }
    return v;
}


inline void VRMLParserImpl::readSFVec3f(SFVec3f& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFVec3f>(readProtoField(SFVEC3F));
    } else {
        out_value = ::readSFVec3f(scanner);
    }
}


void VRMLParserImpl::readMFVec3f(MFVec3f& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFVec3f>(readProtoField(MFVEC3F));
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(::readSFVec3f(scanner));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(::readSFVec3f(scanner));
            }
        }
    }
}


static inline SFVec3s readSFVec3s(EasyScanner* scanner)
{
    SFVec3s v;
    for(int i=0; i < 3; ++i){
        v[i] = scanner->readFloatEx("illegal vector element");
    }
    return v;
}


inline void VRMLParserImpl::readSFVec3s(SFVec3s& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFVec3f>(readProtoField(SFVEC3F)).cast<SFVec3s::Scalar>();
    } else {
        out_value = ::readSFVec3s(scanner);
    }
}


void VRMLParserImpl::readMFVec3s(MFVec3s& out_value)
{
    if(scanner->readSymbol(F_IS)){
        const MFVec3f& value = stdx::get<MFVec3f>(readProtoField(MFVEC3F));
        const size_t n = value.size();
        out_value.resize(n);
        for(size_t i=0; i < n; ++i){
            out_value[i] = value[i].cast<SFVec3s::Scalar>();
        }
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(::readSFVec3s(scanner));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(::readSFVec3s(scanner));
            }
        }
    }
}


static inline SFRotation readSFRotation(EasyScanner* scanner)
{
    SFRotation rotation;
    SFRotation::Vector3& axis = rotation.axis();
    for(int i=0; i < 3; ++i){
        axis[i] = scanner->readDoubleEx("illegal rotaion axis element");
    }
    rotation.angle() = scanner->readDoubleEx("illegal rotaion angle value");

    double size = axis.norm();
    if(size < 1.0e-6){
        scanner->throwException("Rotation axis is the zero vector");
    }
    axis /= size; // normalize

    return rotation;
}


void VRMLParserImpl::readSFRotation(SFRotation& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFRotation>(readProtoField(SFROTATION));
    } else {
        out_value = ::readSFRotation(scanner);
    }
}


void VRMLParserImpl::readMFRotation(MFRotation& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFRotation>(readProtoField(MFROTATION));
    } else {
        out_value.clear();
        if(!scanner->readChar('[')){
            out_value.push_back(::readSFRotation(scanner));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(::readSFRotation(scanner));
            }
        }
    }
}


void VRMLParserImpl::readSFBool(SFBool& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<SFBool>(readProtoField(SFBOOL));
    } else {
        switch(scanner->readSymbolEx("no bool value")){
        case V_TRUE:  out_value = true;  break;
        case V_FALSE: out_value = false; break;
        default: scanner->throwException("no bool value");
        }
    }
}


void VRMLParserImpl::readSFImage(SFImage& out_image)
{
    if(scanner->readSymbol(F_IS)){
        out_image = stdx::get<SFImage>(readProtoField(SFIMAGE));
    } else {
        readSFInt32(out_image.width);
        readSFInt32(out_image.height);
        readSFInt32(out_image.numComponents);

        //! start reading pixel values per component.
        //! numComponents means:
        //!   1:grayscale, 2:grayscale with transparency
        //!   3:RGB components, 4:RGB components with transparency
        SFInt32	pixelValue;
        unsigned char componentValue;
        const SFInt32 comps = out_image.numComponents;

        for(int h=0; h < out_image.height; ++h){
            for(int w=0; w < out_image.width; ++w){
                readSFInt32(pixelValue);
                for(int i=0, shift=8*(comps - 1); i < comps; ++i, shift-=8){
                    // get each component values from left 8 bytes
                    componentValue = (unsigned char)((pixelValue >> shift) & 0x000000FF);
                    out_image.pixels.push_back(componentValue);
                }
            }
        }
    }
}


void VRMLParserImpl::readSFTime(SFTime& out_value)
{
    if(scanner->readSymbol( F_IS )){
        out_value = stdx::get<SFTime>(readProtoField(SFTIME));
    } else {
        out_value = scanner->readDoubleEx("illegal time value");
    }
}


void VRMLParserImpl::readMFTime(MFTime& out_value)
{
    if(scanner->readSymbol(F_IS)){
        out_value = stdx::get<MFTime>(readProtoField(MFTIME));
    } else {
        out_value.clear();
        if(!scanner->readChar('[' )){
            out_value.push_back(SFTime(scanner->readDoubleEx("illegal time value")));
        } else {
            while(!scanner->readChar(']')){
                out_value.push_back(SFTime(scanner->readDoubleEx("illegal time value")));
            }
        }
    }
}


// This API should be obsolete
void VRMLParserImpl::readSFNode(SFNode& out_node, VRMLNodeCategory nodeCategory)
{
    if(scanner->readSymbol(F_IS)){
        out_node = stdx::get<SFNode>(readProtoField(SFNODE));
    } else if(scanner->readSymbol(V_NULL)){
        out_node = 0;
    } else {
        out_node = readNode(nodeCategory);
    }
}


SFNode VRMLParserImpl::readSFNode(VRMLNodeCategory nodeCategory)
{
    if(scanner->readSymbol(F_IS)){
        return stdx::get<SFNode>(readProtoField(SFNODE));
    } else if(scanner->readSymbol(V_NULL)){
        return 0;
    } else {
        return readNode(nodeCategory);
    }
}


void VRMLParserImpl::readMFNode(MFNode& out_nodes, VRMLNodeCategory nodeCategory)
{
    if(scanner->readSymbol(F_IS)){
        out_nodes = stdx::get<MFNode>(readProtoField(MFNODE));
    } else {
        SFNode sfnode;
        out_nodes.clear();
        if(!scanner->readChar('[')){
            readSFNode(sfnode, nodeCategory);
            out_nodes.push_back(sfnode);
        } else {
            while(true) {
                readSFNode(sfnode, nodeCategory);
                if(sfnode){
                    out_nodes.push_back(sfnode);
                }
                bool closed = scanner->readChar(']');
                if(closed){
                    break;
                }
                if(!sfnode && !closed){
                    scanner->throwException("syntax error");
                }
            }
        }
    }
}

void VRMLParserImpl::init()
{
    os_ = &nullout();
    currentProtoInstance = 0;
    protoInstanceActualNodeExtractionMode = true;

    topScanner = std::make_shared<EasyScanner>();
    scanner = topScanner.get();
    setSymbols();
}

void VRMLParserImpl::setSymbols()
{
    static mutex symbolMutex;
    lock_guard<mutex> lock(symbolMutex);
    static shared_ptr<unordered_map<string, int>> symbols(new unordered_map<string, int>{
        // values
        { "TRUE", V_TRUE },
        { "FALSE", V_FALSE },
        { "NULL", V_NULL },

        // types
        { "SFInt32", T_SFINT32 },
        { "MFInt32", T_MFINT32 },
        { "SFFloat", T_SFFLOAT },
        { "MFFloat", T_MFFLOAT },

        { "SFVec2f", T_SFVEC2F },
        { "MFVec2f", T_MFVEC2F },
        { "SFVec3f", T_SFVEC3F },
        { "MFVec3f", T_MFVEC3F },
        { "SFRotation", T_SFROTATION },
        { "MFRotation", T_MFROTATION },
        { "SFTime", T_SFTIME },
        { "MFTime", T_MFTIME },
        { "SFColor", T_SFCOLOR },
        { "MFColor", T_MFCOLOR },
        { "SFString", T_SFSTRING },
        { "MFString", T_MFSTRING },
        { "SFNode", T_SFNODE },
        { "MFNode", T_MFNODE },
        { "SFBool", T_SFBOOL },
        { "SFImage", T_SFIMAGE },

        // Nodes
        { "PROTO", N_PROTO },
        { "Inline", N_INLINE },
        { "Background", N_BACKGROUND },
        { "NavigationInfo", N_NAVIGATION_INFO },
        { "Viewpoint", N_VIEWPOINT },
        { "Group", N_GROUP },
        { "Transform", N_TRANSFORM },
        { "Shape", N_SHAPE },
        { "Appearance", N_APPEARANCE },
        { "Material", N_MATERIAL },
        { "ImageTexture", N_IMAGE_TEXTURE },
        { "TextureTransform", N_TEXTURE_TRANSFORM },
        { "Box", N_BOX },
        { "Cone", N_CONE },
        { "Cylinder", N_CYLINDER },
        { "Sphere", N_SPHERE },
        { "Text", N_TEXT },
        { "FontStyle", N_FONT_STYLE },
        { "IndexedLineSet", N_INDEXED_LINE_SET },
        { "IndexedFaceSet", N_INDEXED_FACE_SET },
        { "Color", N_COLOR },
        { "Coordinate", N_COORDINATE },
        { "TextureCoordinate", N_TEXTURE_COORDINATE },
        { "Normal", N_NORMAL },
        { "CylinderSensor", N_CYLINDER_SENSOR },

        { "PointSet", N_POINTSET },
        { "PixelTexture", N_PIXEL_TEXTURE },
        { "MovieTexture", N_MOVIE_TEXTURE },
        { "ElevationGrid", N_ELEVATION_GRID },
        { "Extrusion", N_EXTRUSION },
        { "Switch", N_SWITCH },
        { "LOD", N_LOD },
        { "Collision", N_COLLISION },
        { "Anchor", N_ANCHOR },
        { "Fog", N_FOG },
        { "Billboard", N_BILLBOARD },
        { "WorldInfo", N_WORLD_INFO },
        { "PointLight", N_POINT_LIGHT },
        { "DirectionalLight", N_DIRECTIONAL_LIGHT },
        { "SpotLight", N_SPOT_LIGHT },

        // unsupported nodes
        { "AudioClip", N_AUDIO_CLIP },
        { "Sound", N_SOUND },
        { "ColorInterpolator", N_COLOR_INTERPOLATOR },
        { "CoordinateInterpolator", N_COORDINATE_INTERPOLATOR },
        { "OrientationInterpolator", N_ORIENTATION_INTERPOLATOR },
        { "NormalInterpolator", N_NORMAL_INTERPOLATOR },
        { "PositionInterpolator", N_POSITION_INTERPOLATOR },
        { "ScalarInterpolator", N_SCALAR_INTERPOLATOR },
        { "PlaneSensor", N_PLANE_SENSOR },
        { "ProximitySensor", N_PROXIMITY_SENSOR },
        { "SphereSensor", N_SPHERE_SENSOR },
        { "TimeSensor", N_TIME_SENSOR },
        { "TouchSensor", N_TOUCH_SENSOR },
        { "VisibilitySensor", N_VISIBILITY_SENSOR },

        // Fields
        { "IS", F_IS },

        { "url", F_URL },

        { "groundAngle", F_GROUND_ANGLE },
        { "groundColor", F_GROUND_COLOR },
        { "skyAngle", F_SKY_ANGLE },
        { "skyColor", F_SKY_COLOR },
        { "backUrl", F_BACK_URL },
        { "bottomUrl", F_BOTTOM_URL },
        { "frontUrl", F_FRONT_URL },
        { "leftUrl", F_LEFT_URL },
        { "rightUrl", F_RIGHT_URL },
        { "topUrl", F_TOP_URL },

        { "avatarSize", F_AVATAR_SIZE },
        { "headlight", F_HEADLIGHT },
        { "speed", F_SPEED },
        { "type", F_TYPE },
        { "visibilityLimit", F_VISIBILITY_LIMIT },

        { "fieldOfView", F_FIELD_OF_VIEW },
        { "jump", F_JUMP },
        { "orientation", F_ORIENTATION },
        { "position", F_POSITION },
        { "description", F_DESCRIPTION },

        { "children", F_CHILDREN },
        { "addChildren", F_ADD_CHILDREN },
        { "removeChildren", F_REMOVE_CHILDREN },
        { "bboxCenter", F_BBOX_CENTER },
        { "bboxSize", F_BBOX_SIZE },

        { "center", F_CENTER },
        { "rotation", F_ROTATION },
        { "scale", F_SCALE },
        { "scaleOrientation", F_SCALE_ORIENTATION },
        { "translation", F_TRANSLATION },

        { "appearance", F_APPEARANCE },
        { "geometry", F_GEOMETRY },

        { "material", F_MATERIAL },
        { "texture", F_TEXTURE },
        { "textureTransform", F_TEXTURE_TRANSFORM },

        { "ambientIntensity", F_AMBIENT_INTENSITY },
        { "diffuseColor", F_DIFFUSE_COLOR },
        { "emissiveColor", F_EMISSIVE_COLOR },
        { "shininess", F_SHININESS },
        { "specularColor", F_SPECULAR_COLOR },
        { "transparency", F_TRANSPARANCY },
        { "direction", F_DIRECTION },

        { "repeatS", F_REPEAT_S },
        { "repeatT", F_REPEAT_T },

        { "size", F_SIZE },

        { "bottom", F_BOTTOM },
        { "bottomRadius", F_BOTTOM_RADIUS },
        { "height", F_HEIGHT },
        { "side", F_SIDE },

        { "radius", F_RADIUS },
        { "top", F_TOP },

        { "string", F_STRING },
        { "fontStyle", F_FONT_STYLE },
        { "length", F_LENGTH },
        { "maxExtent", F_MAX_EXTENT },

        { "family", F_FAMILY },
        { "horizontal", F_HORIZONTAL },
        { "justify", F_JUSTIFY },
        { "language", F_LANGUAGE },
        { "leftToRight", F_LEFT_TO_RIGHT },
        { "spacing", F_SPACING },
        { "style", F_STYLE },
        { "topToBottom", F_TOP_TO_BOTTOM },

        { "color", F_COLOR },
        { "coord", F_COORD },
        { "colorIndex", F_COLOR_INDEX },
        { "colorPerVertex", F_COLOR_PER_VERTEX },
        { "coordIndex", F_COORD_INDEX },

        { "ccw", F_CCW },
        { "convex", F_CONVEX },
        { "solid", F_SOLID },
        { "creaseAngle", F_CREASE_ANGLE },
        { "normalIndex", F_NORMAL_INDEX },
        { "normal", F_NORMAL },
        { "normalPerVertex", F_NORMAL_PER_VERTEX },
        { "texCoordIndex", F_TEX_COORD_INDEX },
        { "texCoord", F_TEX_COORD },

        { "point", F_POINT },
        { "vector", F_VECTOR },

        { "autoOffset", F_AUTO_OFFSET },
        { "diskAngle", F_DISK_ANGLE },
        { "enabled", F_ENABLED },
        { "maxAngle", F_MAX_ANGLE },
        { "minAngle", F_MIN_ANGLE },
        { "offset", F_OFFSET },

        { "image", F_IMAGE },

        { "xDimension", F_X_DIMENSION },
        { "zDimension", F_Z_DIMENSION },
        { "xSpacing", F_X_SPACING },
        { "zSpacing", F_Z_SPACING },

        { "crossSection", F_CROSS_SECTION },
        { "spine", F_SPINE },
        { "beginCap", F_BEGIN_CAP },
        { "endCap", F_END_CAP },

        { "choice", F_CHOICE },
        { "whichChoice", F_WHICH_CHOICE },

        { "collide", F_COLLIDE },
        { "proxy", F_PROXY },

        { "parameter", F_PARAMETER },

        { "visibilityRange", F_VISIBILITY_RANGE },
        { "fogType", F_FOG_TYPE },

        { "axisOfRotation", F_AXIS_OF_ROTATION },

        { "title", F_TITLE },
        { "info", F_INFO },

        { "location", F_LOCATION },
        { "on", F_ON },
        { "intensity", F_INTENSITY },
        { "attenuation", F_ATTENUATION },
        { "beamWidth", F_BEAM_WIDTH },
        { "cutOffAngle", F_CUT_OFF_ANGLE },

        // event type
        { "field", E_FIELD },
        { "exposedField", E_EXPOSED_FIELD },
        { "eventIn", E_EVENTIN },
        { "eventOut", E_EVENTOUT },

        // def & route
        { "DEF", D_DEF },
        { "USE", D_USE },
        { "ROUTE", D_ROUTE },

        // unsupported keywords
        { "Script", U_SCRIPT },
        { "EXTERNPROTO", U_EXTERNPROTO }
    });

    scanner->setSymbols(symbols);
}


void VRMLParserImpl::convertUrl(MFString& urls)
{
    for(MFString::iterator it=urls.begin(); it!=urls.end(); it++){
        stdx::filesystem::path path;
        string chkFile("");
        if(isFileProtocol(*it)){
            path = stdx::filesystem::lexically_normal(removeURLScheme(*it));

            // Relative path check & translate to absolute path 
            if(!exists(path)){
                stdx::filesystem::path parentPath(scanner->filename);
                path = stdx::filesystem::lexically_normal(parentPath.parent_path() / path);
            }
            chkFile = getAbsolutePathString(path);
        } else {
            // Not file protocol implements   
            scanner->throwException("Not file protocol is unsupported");
        }
        *it = chkFile;
    }
}
