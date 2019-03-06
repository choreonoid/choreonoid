/*
 * modelLoader-test.cpp
 *
 *  Created on: 2018/04/24
 *      Author: hattori
 */

#include <cnoid/corba/OpenHRP/3.1/ModelLoader.hh>
#include <cnoid/CorbaUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Body>
#include <cnoid/Sleep>
#include <cnoid/FileUtil>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;

void dumpLink(int i, const LinkInfo& linkInfo);
void dumpTransformedShapeIndex( const TransformedShapeIndex& transformedShapeIndex);
void dumpSensor( const SensorInfo& sensorInfo );
void dumpHwc( const HwcInfo& hwc);
void dumpSegment( const SegmentInfo& segment);
void dumpLight( const LightInfo& light);
void dumpShape(int i, const ShapeInfo& shape);
void dumpAppearance(int i, const AppearanceInfo& appearance);
void dumpMaterial(int i, const MaterialInfo& material);
void dumpTexture(int i, const TextureInfo& texture);
void dumpExtraJoint(const ExtraJointInfo& joint);

int main(int argc, char* argv[])
{
    string filepath;

    if(argc >= 2){
        filepath = getAbsolutePathString(boost::filesystem::path(argv[1]));
    }
    bool flg = true;
    if(argc >=3){
        if(string(argv[2])=="scene"){
            flg = false;
        }
    }

    initializeCorbaUtil();

    ModelLoader_var modelLoader =
            getDefaultNamingContextHelper()->findObject<OpenHRP::ModelLoader>("ModelLoader");

    if(flg){

        BodyInfo_var bodyInfo = modelLoader->getBodyInfo(filepath.c_str());

        cout << "<<< BodyInfo >>>\n";

        cout << "name: " << bodyInfo->name() << "\n";
        cout << "url: " << bodyInfo->url() << "\n";
        StringSequence_var info = bodyInfo->info();
        for(int i=0; i<info->length(); i++){
            cout << "info: " << info[i] << "\n";
        }
        cout << "\n";

        LinkInfoSequence_var links = bodyInfo->links();
        int numLinks = links->length();
        cout << "num links: " << numLinks << "\n";
        for(int i=0; i < numLinks; ++i){
            dumpLink(i, links[i]);
        }
        cout << "\n";

        cout << "<<< ShapeInfo >>>\n";
        ShapeInfoSequence_var shapes = bodyInfo->shapes();
        int numShapes = shapes->length();
        cout << "num shapes: " << numShapes << "\n";
        for(int i=0; i < numShapes; ++i){
            dumpShape(i, shapes[i]);
        }
        cout << "\n";

        cout << "<<< AppearanceInfo >>>\n";
        AppearanceInfoSequence_var appearances = bodyInfo->appearances();
        int numAppearance = appearances->length();
        cout << "num appearance: " << numAppearance << "\n";
        for(int i=0; i < numAppearance; ++i){
            dumpAppearance(i, appearances[i]);
        }

        cout << "\n";

        cout << "<<< MaterialInfo >>>\n";
        MaterialInfoSequence_var materials = bodyInfo->materials();
        int numMaterial = materials->length();
        cout << "num material: " << numMaterial << "\n";
        for(int i=0; i < numMaterial; ++i){
            dumpMaterial(i, materials[i]);
        }
        cout << "\n";

        cout << "<<< TextureInfo >>>\n";
        TextureInfoSequence_var textures = bodyInfo->textures();
        int numTexture = textures->length();
        cout << "num Texture: " << numTexture << "\n";
        for(int i=0; i < numTexture; ++i){
            dumpTexture(i, textures[i]);
        }
        cout << "\n";

        cout << "<<< AllLinkShapeIndexInfo >>>\n";
        AllLinkShapeIndexSequence_var allLinkShapes = bodyInfo->linkShapeIndices();
        int numAllLinkShape = allLinkShapes->length();
        cout << "num AlllinkShape: " << numAllLinkShape << "\n";
        for(int i=0; i<numAllLinkShape; i++){
            const TransformedShapeIndexSequence& shapeIndices = allLinkShapes[i];
            int num = shapeIndices.length();
            for(int j=0; j<num; j++){
                cout << shapeIndices[j].shapeIndex << " ";
            }
            cout << "\n";
        }
        cout << "\n";

        cout << "<<< ExtraJointInfo >>>\n";
        ExtraJointInfoSequence_var extraJoints = bodyInfo->extraJoints();
        int numExtraJoint = extraJoints->length();
        cout << "num ExtraJoint: " << numExtraJoint << "\n";
        for(int i=0; i < numExtraJoint; ++i){
            dumpExtraJoint(extraJoints[i]);
        }

        cout << "<<< BodyInfo  End >>>\n";
        cout.flush();

    }else {
        SceneInfo_var sceneInfo = modelLoader->loadSceneInfo(filepath.c_str());

        cout << "<<< SceneInfo >>>\n";
        cout << "url: " << sceneInfo->url() << "\n";

        TransformedShapeIndexSequence_var shapeIndices = sceneInfo->shapeIndices();
        cout << "TransformedShapeIndices: num= " << shapeIndices->length() << "\n";
        for(CORBA::ULong i=0; i < shapeIndices->length(); ++i){
            dumpTransformedShapeIndex( shapeIndices[i] );
        }

        cout << "<<< ShapeInfo >>>\n";
        ShapeInfoSequence_var shapes = sceneInfo->shapes();
        int numShapes = shapes->length();
        cout << "num shapes: " << numShapes << "\n";
        for(int i=0; i < numShapes; ++i){
            dumpShape(i, shapes[i]);
        }
        cout << "\n";

        cout << "<<< AppearanceInfo >>>\n";
        AppearanceInfoSequence_var appearances = sceneInfo->appearances();
        int numAppearance = appearances->length();
        cout << "num appearance: " << numAppearance << "\n";
        for(int i=0; i < numAppearance; ++i){
            dumpAppearance(i, appearances[i]);
        }

        cout << "\n";

        cout << "<<< MaterialInfo >>>\n";
        MaterialInfoSequence_var materials = sceneInfo->materials();
        int numMaterial = materials->length();
        cout << "num material: " << numMaterial << "\n";
        for(int i=0; i < numMaterial; ++i){
            dumpMaterial(i, materials[i]);
        }
        cout << "\n";

        cout << "<<< TextureInfo >>>\n";
        TextureInfoSequence_var textures = sceneInfo->textures();
        int numTexture = textures->length();
        cout << "num Texture: " << numTexture << "\n";
        for(int i=0; i < numTexture; ++i){
            dumpTexture(i, textures[i]);
        }
        cout << "\n";

        cout << "<<< SceneInfo  End >>>\n";
        cout.flush();

    }
}


template <typename T> void dumpSequence( const T& s, const string& str)
{
    if(s.length() > 0){
        cout << str << ": ";
        for(CORBA::ULong i=0; i < s.length(); ++i){
            cout << s[i] << " ";
        }
        cout << "\n";
    }
}
template void dumpSequence(const DblSequence& s, const string& str);
template void dumpSequence(const FloatSequence& s, const string& str);


template <typename T> void dumpArray3(const T& v, const string& str)
{
    cout << str << ": " << v[0] << ", " << v[1] << ", " << v[2] << "\n";
}
template void dumpArray3( const DblArray3& v, const string& str);
template void dumpArray3( const FloatArray3& v, const string& str);


void dumpDbl4(const DblArray4& v, const string& str)
{
    cout << str << ": " << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << "\n";
}

void dumpDbl9(const DblArray9& v, const string& str)
{
    string space(str.size()+2,' ');
    cout << str << ": " << v[0] << ", " << v[1] << ", " << v[2] << "\n";
    cout << space << v[3] << ", " << v[4] << ", " << v[5] << "\n";
    cout << space << v[6] << ", " << v[7] << ", " << v[8] << "\n";
}

void dumpDbl12(const double* v, const string& str)
{
    string space(str.size()+2,' ');
    cout << str << ": " << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3]<< "\n";
    cout << space << v[4] << ", " << v[5] << ", " << v[6] << ", " << v[7]<< "\n";
    cout << space << v[8] << ", " << v[9] << ", " << v[10] << ", " << v[11]<< "\n";
}

void dumpLink(int i, const LinkInfo& linkInfo)
{
    cout << "<<< LinkInfo: " << linkInfo.name << " (index " << i << ") >>>\n";
    cout << "parentIndex: " << linkInfo.parentIndex << "\n";

    const ShortSequence& childIndices = linkInfo.childIndices;
    if(childIndices.length() > 0){
        cout << "childIndices: ";
        for(CORBA::ULong i=0; i < childIndices.length(); ++i){
            cout << childIndices[i] << " ";
        }
        cout << "\n";
    }

    cout << "jointId: " << linkInfo.jointId << "\n";
    cout << "jointType: " << linkInfo.jointType << "\n";
    cout << "jointValue: " << linkInfo.jointValue << "\n";
    dumpArray3(linkInfo.jointAxis, "jointAxis");
    dumpSequence(linkInfo.ulimit, "ulimit");
    dumpSequence(linkInfo.llimit, "llimit");
    dumpSequence(linkInfo.uvlimit, "uvlimit");
    dumpSequence(linkInfo.lvlimit, "lvlimit");
    dumpSequence(linkInfo.climit, "climit");
    dumpArray3(linkInfo.translation, "translation");
    dumpDbl4(linkInfo.rotation, "rotation");
    cout << "mass: " << linkInfo.mass << "\n";
    dumpArray3(linkInfo.centerOfMass, "centerOfMass");
    dumpDbl9(linkInfo.inertia, "inertia");

    cout << "rotorInertia: " << linkInfo.rotorInertia << "\n";
    cout << "rotorResistor: " << linkInfo.rotorResistor << "\n";
    cout << "gearRatio: " << linkInfo.gearRatio << "\n";
    cout << "torqueConst: " << linkInfo.torqueConst << "\n";
    cout << "encoderPulse: " << linkInfo.encoderPulse << "\n";

    const TransformedShapeIndexSequence& shapeIndices = linkInfo.shapeIndices;
    cout << "TransformedShapeIndices: num= " << shapeIndices.length() << "\n";
    for(CORBA::ULong i=0; i < shapeIndices.length(); ++i){
        dumpTransformedShapeIndex( shapeIndices[i] );
    }

    cout << "AABBmaxDepth: " << linkInfo.AABBmaxDepth << "\n";
    cout << "AABBmaxNum: " << linkInfo.AABBmaxNum << "\n";

    const DblArray12Sequence& inlinedShapeTransforms = linkInfo.inlinedShapeTransformMatrices;
    cout << "InlinedShapeTransformMatrices: num = " << inlinedShapeTransforms.length() << "\n";
    for(int i=0; i<inlinedShapeTransforms.length(); i++){
        dumpDbl12( inlinedShapeTransforms[i], "");
    }

    const SensorInfoSequence& sensorInfos = linkInfo.sensors;
    int numSensors = sensorInfos.length();
    cout << "Sensors: num= " << numSensors << "\n";
    for(int i=0; i < numSensors; i++){
        dumpSensor( sensorInfos[i] );
    }

    const HwcInfoSequence& hwcInfos = linkInfo.hwcs;
    cout << "Hwc: num = " << hwcInfos.length() << "\n";
    for(int i=0; i<hwcInfos.length(); i++){
        dumpHwc( hwcInfos[i] );
    }

    const SegmentInfoSequence& segmentInfos = linkInfo.segments;
    cout << "Segment: num = " << segmentInfos.length() << "\n";
    for(int i=0; i<segmentInfos.length(); i++){
        dumpSegment( segmentInfos[i] );
    }

    const LightInfoSequence& lightInfos = linkInfo.lights;
    cout << "Light: num = " << lightInfos.length() << "\n";
    for(int i=0; i<lightInfos.length(); i++){
        dumpLight( lightInfos[i] );
    }

    const StringSequence& specFiles = linkInfo.specFiles;
    cout << "SpecFiles: num = " << specFiles.length() << "\n";
    for(int i=0; i<specFiles.length(); i++){
        cout << specFiles[i] << "\n";
    }
}


void dumpTransformedShapeIndex( const TransformedShapeIndex& transformedShapeIndex)
{
    dumpDbl12( transformedShapeIndex.transformMatrix, "transformedShapeIndex");
    cout << "inlinedShapeTransformMatrixIndex: " << transformedShapeIndex.inlinedShapeTransformMatrixIndex << "\n";
    cout << "shapeIndex: " << transformedShapeIndex.shapeIndex << "\n";
}


void dumpSensor( const SensorInfo& sensorInfo )
{
    cout << "type: " << sensorInfo.type << "\n";
    cout << "name: \"" << sensorInfo.name << "\"\n";
    cout << "id: " << sensorInfo.id << "\n";
    dumpArray3( sensorInfo.translation, "translation" );
    dumpDbl4( sensorInfo.rotation, "rotation" );
    dumpSequence( sensorInfo.specValues, "spec" );
    cout << "specFile: " << sensorInfo.specFile << "\n";

    const TransformedShapeIndexSequence& shapeIndices = sensorInfo.shapeIndices;
    cout << "TransformedShapeIndices: num= " << shapeIndices.length() << "\n";
    for(CORBA::ULong i=0; i < shapeIndices.length(); ++i){
        dumpTransformedShapeIndex( shapeIndices[i] );
    }

    const DblArray12Sequence& inlinedShapeTransforms = sensorInfo.inlinedShapeTransformMatrices;
    cout << "InlinedShapeTransformMatrices: num = " << inlinedShapeTransforms.length() << "\n";
    for(int i=0; i<inlinedShapeTransforms.length(); i++){
        dumpDbl12( inlinedShapeTransforms[i], "");
    }
}


void dumpHwc( const HwcInfo& hwc)
{
    cout << "name: " << hwc.name << "\n";
    cout << "id: " << hwc.id << "\n";
    dumpArray3( hwc.translation, "translation" );
    dumpDbl4( hwc.rotation, "rotation" );
    cout << "url: " << hwc.url << "\n";

    const TransformedShapeIndexSequence& shapeIndices = hwc.shapeIndices;
    cout << "TransformedShapeIndices: num= " << shapeIndices.length() << "\n";
    for(CORBA::ULong i=0; i < shapeIndices.length(); ++i){
        dumpTransformedShapeIndex( shapeIndices[i] );
    }

    const DblArray12Sequence& inlinedShapeTransforms = hwc.inlinedShapeTransformMatrices;
    cout << "InlinedShapeTransformMatrices: num = " << inlinedShapeTransforms.length() << "\n";
    for(int i=0; i<inlinedShapeTransforms.length(); i++){
        dumpDbl12( inlinedShapeTransforms[i], "");
    }
}

void dumpSegment( const SegmentInfo& segment)
{
    cout << "name: " << segment.name << "\n";
    cout << "mass: " << segment.mass << "\n";
    dumpArray3( segment.centerOfMass, "centerOfmass" );
    dumpDbl9( segment.inertia, "inertia" );
    dumpDbl12( segment.transformMatrix, "transform" );
    cout << "shapeIndices: ";
    for(int i=0; i<segment.shapeIndices.length(); i++){
        cout << segment.shapeIndices[i] << ", ";
    }
    cout << "\n";

}

void dumpLight( const LightInfo& light)
{
    cout << "name: " << light.name << "\n";
    cout << "type: " << light.type << "\n";
    dumpDbl12( light.transformMatrix, "transformMatrix" );
    cout << "ambientIntensity: " << light.ambientIntensity << "\n";
    dumpArray3( light.attenuation, "attenuation" );
    dumpArray3( light.color, "color" );
    cout << "intensity: " << light.intensity << "\n";
    dumpArray3( light.location, "location" );
    cout << "on: " << light.on << "\n";
    cout << "radius: " << light.radius << "\n";
    dumpArray3( light.direction, "direction" );
    cout << "beamWidth: " << light.beamWidth << "\n";
    cout << "cutOffAngle: " << light.cutOffAngle << "\n";
}


void dumpShape(int i, const ShapeInfo& shape)
{
    cout << "index: " << i << "\n";
    cout << "url: " << shape.url << "\n";
    cout << "primitiveType: " << shape.primitiveType << "\n";
    dumpSequence( shape.primitiveParameters, "parameter" );

    const FloatSequence& v = shape.vertices;
    cout << "vertices: num = " << v.length()/3 << "\n";
    int n = min((int)v.length()/3, 10);
    for(int i=0; i<n; i++){
        cout << i << ": " << v[i*3] << ", " << v[i*3+1] << ", " << v[i*3+2] << "\n";
    }

    const LongSequence& t = shape.triangles;
    cout << "triangles: num = " << t.length()/3 << "\n";
    n = min((int)t.length()/3, 10);
    for(int i=0; i<n; i++){
        cout << i << ": " << t[i*3] << ", " << t[i*3+1] << ", " << t[i*3+2] << "\n";
    }

    cout << "appearanceIndex: " << shape.appearanceIndex << "\n";
}


void dumpAppearance(int i, const AppearanceInfo& appearance)
{
    cout << "index: " << i << "\n";
    cout << "materialIndex: " << appearance.materialIndex << "\n";

    const FloatSequence& normals = appearance.normals;
    cout << "normals: num = " << normals.length()/3 << "\n";
    int n = min((int)normals.length()/3, 10);
    for(int i=0; i<n; i++){
        cout << i << ": " << normals[i*3] << ", " << normals[i*3+1] << ", " << normals[i*3+2] << "\n";
    }

    cout << "normalPerVertex: " << appearance.normalPerVertex << "\n";
    const LongSequence& ni = appearance.normalIndices;
    cout << "normalIndices: num = " << ni.length() << "\n";
    n = ni.length() > 10 ? 10 : ni.length();
    for(int i=0; i<n; i++){
        cout << i << ": " << ni[i] << "\n";
    }

    cout << "solid: " << appearance.solid << "\n";
    cout << "creaseAngle: " << appearance.creaseAngle << "\n";

    const FloatSequence& colors = appearance.colors;
    cout << "colors: num = " << colors.length() << "\n";
    n = min((int)colors.length()/3, 10);
    for(int i=0; i<n; i++){
        cout << i << ": " << colors[i*3] << ", " << colors[i*3+1] << ", " << colors[i*3+2] << "\n";
    }

    cout << "colorPerVertex: " << appearance.colorPerVertex << "\n";
    const LongSequence& ci = appearance.colorIndices;
    cout << "colorIndices: num = " << ci.length() << "\n";
    n = min((int)ci.length(), 10);
    for(int i=0; i<n; i++){
        cout << i << ": " << ci[i] << "\n";
    }

    cout << "textureIndex: " << appearance.textureIndex << "\n";
    const FloatSequence& textureCoordinates = appearance.textureCoordinate;
    cout << "textureCoordinates: num = " << textureCoordinates.length() << "\n";
    n = min((int)textureCoordinates.length()/2, 10);
    for(int i=0; i<n; i++){
        cout << i << ": " << textureCoordinates[i*2] << ", " << textureCoordinates[i*2+1] << "\n";
    }

    const LongSequence& tci = appearance.textureCoordIndices;
    cout << "textureCoordIndices: num = " << tci.length() << "\n";
    n = min((int)tci.length(), 10);
    for(int i=0; i<n; i++){
        cout << i << ": " << tci[i] << "\n";
    }

    dumpDbl9( appearance.textransformMatrix, "textransform");
}


void dumpMaterial(int i, const MaterialInfo& material)
{
    cout << "index: " << i << "\n";
    cout << "ambientIntensity: " << material.ambientIntensity << "\n";
    dumpArray3(material.diffuseColor, "diffuseColor" );
    dumpArray3(material.emissiveColor, "emissiveColor" );
    cout << "shininess: " << material.shininess << "\n";
    dumpArray3(material.specularColor, "specularColor" );
    cout << "transdparency: " << material.transparency << "\n";
}


void dumpTexture(int i, const TextureInfo& texture)
{
    cout << "index: " << i << "\n";
    const OctetSequence& image = texture.image;
    cout << "image: length= " << image.length() << "\n";
    int n = min((int)image.length(), 20);
    for(int i=0; i<n; i++){
        cout << hex << (unsigned int)image[i] << " ";
    }
    cout << "\n" << dec;
    cout << "numComponents: " << texture.numComponents << "\n";
    cout << "width: " << texture.width << "  ";
    cout << "height: " << texture.height << "\n";
    cout << "repeatS: " << texture.repeatS << "  ";
    cout << "repeatT: " << texture.repeatT << "\n";
    cout << "url: " << texture.url << "\n";
}


void dumpExtraJoint(const ExtraJointInfo& joint)
{
    cout << "name: " << joint.name << "\n";
    cout << "type: " << joint.jointType << "\n";
    dumpArray3( joint.axis, "Axis" );
    cout << "link: " << joint.link[0] << "  " << joint.link[1] << "\n";
    dumpArray3(joint.point[0], "point0");
    dumpArray3(joint.point[1], "point1");
}
