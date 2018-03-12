/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter {

class UtilityImpl
{
public:

    static void printMessage(const std::string& msg, bool sync = false);

    static void printErrorMessage(const std::string& msg, bool sync = false);

    static void printWarningMessage(const std::string& msg, bool sync = false);

    static cnoid::Mapping* mapping(cnoid::Mapping* mapping, const std::string& key);

    static bool exist(cnoid::Mapping* mapping, const std::string& key);

    static bool value(cnoid::Mapping* mapping, const std::string& key, bool& val);

    static bool value(cnoid::Mapping* mapping, const std::string& key, double& val);

    static bool value(cnoid::Mapping* mapping, const std::string& key, Eigen::Vector3d& val);

    static bool value(cnoid::Mapping* mapping, const std::string& key, Eigen::Matrix3d& val);

    static bool value(cnoid::Mapping* mapping, const std::string& key, std::vector<double>& valAry);

    static bool value(cnoid::Mapping* mapping, const std::string& key, std::vector<int>& valAry);

    static bool value(cnoid::Mapping* mapping, const std::string& key, std::vector<bool>& valAry);

    static bool value(cnoid::Mapping* mapping, const std::string& key, int& val);

    static void splitStringArray(const std::string& line, std::vector<std::string>& retAry);

    static void removeSelfNodeFromParent(cnoid::SgNode* self);

    static std::list<std::string> fileList(const std::string& dir, const std::string& ext, const bool withExt);

    static void removeFiles(const std::string& dir);

    static bool stringArray2Data(const std::vector<std::string>& strAry, std::string& tag, std::vector<double>& valAry);

    static bool stringArray2Data(const std::vector<std::string>& strAry, std::string& tag, std::vector<int>& valAry);

    static bool stringArray2Data(const std::vector<std::string>& strAry, std::string& tag, int& val);

    static void linkArray(const cnoid::Body* body, std::vector<cnoid::Link*>& linkAry);

    static std::string toString(const Eigen::Vector3d& vec, const std::string& fmt = std::string("%.6lf"));

    static std::string toString(const std::vector<int>& ary);

    static std::string toString(const std::vector<double>& ary, const std::string& fmt = "%.6lf");

    static std::string toString(const std::vector<std::string>& ary);

    static bool toIntegerArray(const std::string& str, std::vector<int>& ary);

    static bool toFloatArray(const std::string& str, std::vector<double>& ary);

    static bool toVector3d(const std::string& str, Eigen::Vector3d& vec);

    static void linkAttributeFromBodyMappingInfo(cnoid::Body* body, std::map<const cnoid::Link*, LinkAttribute>& linkAttrMap, bool createDevice = true);

    static bool findLinkAttribute(const std::string& bodyName, const std::string& linkName, const std::map<const cnoid::Link*, LinkAttribute>& linkAttrMap, LinkAttribute& attr);

    static void printSomethingWrongAtRotor(const std::string& key);

protected:

    static void printMessageImpl(const std::string& msg);

    static void printErrorMessageImpl(const std::string& msg);

    static void printWarningMessageImpl(const std::string& msg);

    static void printSomethingWrongAtBody(const std::string& key, const std::string& bodyName);

    static void printSomethingWrongAtLink(const std::string& key, const std::string& linkName);

};
}
