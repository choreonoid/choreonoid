/*!
 * @author Shin'ichiro Nakaoka
 * @author Hisashi Ikari 
*/
#include <boost/python.hpp>
#include <boost/filesystem.hpp>
#include <cnoid/ExecutablePath>
#include <cnoid/FloatingNumberString>
#include <cnoid/FileUtil>
#include <cnoid/AbstractSeq>
#include <cnoid/MultiSeq>
#include <cnoid/MultiValueSeq>
#include <cnoid/MultiSE3Seq>
#include <cnoid/MultiAffine3Seq>
#include <cnoid/Array2D>
#include <cnoid/EigenTypes>

//#include <boost/python/cross_module.hpp>

using namespace boost::python;
using namespace cnoid;

/*!
 * @brief Provides the Util package of Choreonoid.
 *        And we want to provide this classes to the other so(dll) like CNOID_EXPORT.
 * @note  Util is used from so many others. Therefore, export_module(like CNOID_EXPORT) is needed.
 *
 * @reference http://goo.gl/eNOVqh
 * @reference http://goo.gl/nXYCSR
 */
BOOST_PYTHON_MODULE(Util)
{
    /*!
     * @brief It will provide the file utility.
     */
    def("shareDirectory", &cnoid::shareDirectory, return_value_policy<copy_const_reference>());


    /*!
     * @brief It will provide the file utility for getting executable path.
     */
    def("executablePath", &cnoid::executablePath, return_value_policy<copy_const_reference>());


    /*!
     * @brief It will provide the file utility for getting executable path.
     */
    def("executableBasename", &cnoid::executableBasename, return_value_policy<copy_const_reference>());


    /*!
     * @brief It will provide the file utility for getting executable path.
     */
    def("executableTopDirectory", &cnoid::executableTopDirectory, return_value_policy<copy_const_reference>());


    /*!
     * @brief It will provide the float utility.
     */ 
    class_ <FloatingNumberString, boost::noncopyable>("FloatingNumberString", init<const std::string&>())
        .def("set", &FloatingNumberString::set)
        .def("setPositiveValue", &FloatingNumberString::setPositiveValue, return_value_policy<return_by_value>())
        .def("setNonNegativeValue", &FloatingNumberString::setNonNegativeValue, return_value_policy<return_by_value>())
        .def("value", &FloatingNumberString::value, return_value_policy<return_by_value>());


    /*!
     * @brief This is a definition for SE3.
     */
    void (SE3::*set)(const Vector3 translation, const Matrix3& R) = &SE3::set;
    Vector3& (SE3::*translation)() = &SE3::translation;
    Quat& (SE3::*rotation)() = &SE3::rotation;

    class_<SE3, boost::noncopyable>("SE3", init<>()) 
        .def("set", set)
        .def("translation", translation, return_value_policy<return_by_value>())
        .def("rotation", rotation, return_value_policy<return_by_value>());   

 
    /*!
     * @brief It will provide the access of the Frame and Part (Array2D) in the BodyMotion. 
     *        Only definition of the type. Please access in the wrapper.
     */
    class_<AbstractSeq, boost::noncopyable>("AbstractSeq", no_init)
        .def("seqType", &AbstractSeq::seqType, return_value_policy<return_by_value>())
        .def("getFrameRate", &AbstractSeq::getFrameRate, return_value_policy<return_by_value>())
        .def("setFrameRate", &AbstractSeq::setFrameRate)
        .def("getTimeStep", &AbstractSeq::getTimeStep, return_value_policy<return_by_value>())
        .def("setTimeStep", &AbstractSeq::setTimeStep)
        .def("getTimeOfFrame", &AbstractSeq::getTimeOfFrame, return_value_policy<return_by_value>())
        .def("getNumFrames", &AbstractSeq::getNumFrames, return_value_policy<return_by_value>())
        .def("setNumFrames", &AbstractSeq::setNumFrames, (args("n"), args("clearNewElements") = false))
        .def("setTimeLength", &AbstractSeq::setTimeLength, (args("n"), args("clearNewElements") = false))
        .def("getTimeLength", &AbstractSeq::getTimeLength, return_value_policy<return_by_value>())
        .def("seqContentName", &AbstractSeq::seqContentName, return_value_policy<return_by_value>())
        .def("setSeqContentName", &AbstractSeq::setSeqContentName)
        .def("seqMessage", &AbstractSeq::seqMessage, return_value_policy<return_by_value>());


    /*!
     * @brief It will provide the access of the Frame and Part (Array2D) in the BodyMotion. 
     *        Only definition of the type. Please access in the wrapper.
     */
    class_ <AbstractMultiSeq, bases<AbstractSeq>, boost::noncopyable >("AbstractMultiSeq", no_init)
        .def("setDimension", &AbstractMultiSeq::setDimension, (args("numFrames"), args("numParts"), args("clearNewElements") = false))
        .def("setNumParts", &AbstractMultiSeq::setNumParts, (args("numParts"), args("clearNewElements") = false))
        .def("getNumParts", &AbstractMultiSeq::getNumParts, return_value_policy<return_by_value>())
        .def("partIndex", &AbstractMultiSeq::partIndex, return_value_policy<return_by_value>())
        .def("partLabel", &AbstractMultiSeq::partLabel, return_value_policy<return_by_value>());


    /*!
     * @brief It will provide the access of the Frame and Part (Array2D) in the BodyMotion. 
     *        Only definition of the type. Please access in the wrapper.
     */
    typedef MultiSeq< double, std::allocator<double> > MultiSeqDouble;

#ifdef _INVALID_VERSION
    typedef MultiSeq< SE3, std::allocator<SE3> > MultiSeqSE3;
    typedef MultiSeq< Affine3, std::allocator<Affine3> > MultiSeqAffine;
#endif


    // for double concrete type.
    class_<MultiSeqDouble::Frame, boost::noncopyable>("FrameDouble", no_init)
        .def("empty", &MultiSeqDouble::Frame::empty)
        .def("size", &MultiSeqDouble::Frame::size)
        .def("__getitem__", &MultiSeqDouble::Frame::at, return_value_policy<return_by_value>());

    class_<MultiSeqDouble::Part, boost::noncopyable>("PartDouble", no_init)
        .def("empty", &MultiSeqDouble::Part::empty)
        .def("size", &MultiSeqDouble::Part::size)
        .def("__getitem__", &MultiSeqDouble::Part::at, return_value_policy<return_by_value>());


#ifdef _INVALID_VERSION
    // for SE3 concrete type.
    class_<MultiSeqSE3::Frame, boost::noncopyable>("FrameSE3", no_init)
        .def("empty", &MultiSeqSE3::Frame::empty)
        .def("size", &MultiSeqSE3::Frame::size)
        .def("__getitem__", &MultiSeqSE3::Frame::at, return_value_policy<return_by_value>());

    class_<MultiSeqSE3::Part, boost::noncopyable>("PartSE3", no_init)
        .def("empty", &MultiSeqSE3::Part::empty)
        .def("size", &MultiSeqSE3::Part::size)
        .def("__getitem__", &MultiSeqSE3::Part::at, return_value_policy<return_by_value>());
#endif


#ifdef _INVALID_VERSION
    // for Affine concrete type.
    class_<MultiSeqAffine::Frame, boost::noncopyable>("FrameAffine", no_init)
        .def("empty", &MultiSeqAffine::Frame::empty)
        .def("size", &MultiSeqAffine::Frame::size)
        .def("__getitem__", &MultiSeqAffine::Frame::at, return_value_policy<return_by_value>());

    class_<MultiSeqAffine::Part, boost::noncopyable>("PartAffine", no_init)
        .def("empty", &MultiSeqAffine::Part::empty)
        .def("size", &MultiSeqAffine::Part::size)
        .def("__getitem__", &MultiSeqAffine::Part::at, return_value_policy<return_by_value>());
#endif


    /*!
     * @brief It will provide the access of the Frame and Part (Array2D) in the BodyMotion. 
     *        Only definition of the type. Please access in the wrapper.
     */
    // for double concrete type.
    const MultiSeqDouble::Frame (MultiSeqDouble::*frameDouble)(int index) const = &MultiSeqDouble::frame;
    const MultiSeqDouble::Part (MultiSeqDouble::*partDouble)(int index) const = &MultiSeqDouble::part;


#ifdef _INVALID_VERSION
    // for SE3 concrete type.
    const MultiSeqSE3::Frame (MultiSeqSE3::*frameSE3)(int index) const = &MultiSeqSE3::frame;
    const MultiSeqSE3::Part (MultiSeqSE3::*partSE3)(int index) const = &MultiSeqSE3::part;

    // for Affine concrete type.
    const MultiSeqAffine::Frame (MultiSeqAffine::*frameAffine)(int index) const = &MultiSeqAffine::frame;
    const MultiSeqAffine::Part (MultiSeqAffine::*partAffine)(int index) const = &MultiSeqAffine::part;
#endif


    /*!
     * @note MultiSeq is a template class. The template class is not the instance.
     *       Create a concrete class by providing a template parameter to the this MultiSeq.
     *       However, We define as another concrete class and this class (such as MultiValueSeq).
     *       The reason is because the strong type is different.
     *       And please forgive the presence of multiple same method.
     */
    // for double concrete type.
    const double& (MultiSeqDouble::*atDouble)(int rowIndex, int colIndex) const = &MultiSeqDouble::at;
    const MultiSeqDouble::Row (MultiSeqDouble::*rowDouble)(int rowIndex) const = &MultiSeqDouble::row;
    const MultiSeqDouble::Column (MultiSeqDouble::*columnDouble)(int colIndex) const = &MultiSeqDouble::column; 


#ifdef _INVALID_VERSION
    // for SE3 concrete type.
    const SE3& (MultiSeqDouble::*atSE3)(int rowIndex, int colIndex) const = &MultiSeqSE3::at;
    const MultiSeqSE3::Row (MultiSeqSE3::*rowSE3)(int rowIndex) const = &MultiSeqSE3::row;
    const MultiSeqSE3::Column (MultiSeqSE3::*columnSE3)(int colIndex) const = &MultiSeqSE3::column; 

    // for Affine concrete type.
    const Affine3& (MultiSeqAffine::*atAffine)(int rowIndex, int colIndex) const = &MultiSeqAffine::at;
    const MultiSeqAffine::Row (MultiSeqAffine::*rowAffine)(int rowIndex) const = &MultiSeqAffine::row;
    const MultiSeqAffine::Column (MultiSeqAffine::*columnAffine)(int colIndex) const = &MultiSeqAffine::column; 
#endif


    // for double concrete type.
    class_ <MultiSeqDouble, bases<AbstractMultiSeq>, boost::noncopyable >("AbstractMultiSeqDouble", no_init)
        .def("frameRate", &MultiSeqDouble::frameRate, return_value_policy<return_by_value>())
        .def("timeStep", &MultiSeqDouble::timeStep, return_value_policy<return_by_value>())
        .def("numFrames", &MultiSeqDouble::numFrames, return_value_policy<return_by_value>())
        .def("numParts", &MultiSeqDouble::numParts, return_value_policy<return_by_value>())
        .def("timeLength", &MultiSeqDouble::timeLength, return_value_policy<return_by_value>())
        .def("frameOfTime", &MultiSeqDouble::frameOfTime, return_value_policy<return_by_value>())
        .def("timeOfFrame", &MultiSeqDouble::timeOfFrame, return_value_policy<return_by_value>())
        .def("clampFrameIndex", &MultiSeqDouble::clampFrameIndex)
        .def("frame", frameDouble,  return_value_policy<return_by_value>())
        .def("part", partDouble,  return_value_policy<return_by_value>())
        .def("empty", &MultiSeqDouble::empty, return_value_policy<return_by_value>())
        .def("resize", &MultiSeqDouble::resize)
        .def("resizeColumn", &MultiSeqDouble::resizeColumn)
        .def("rowSize", &MultiSeqDouble::rowSize, return_value_policy<return_by_value>())
        .def("resizeRow", &MultiSeqDouble::resizeRow)
        .def("colSize", &MultiSeqDouble::colSize, return_value_policy<return_by_value>())
        .def("at", atDouble, return_value_policy<return_by_value>())
        .def("row", rowDouble, return_value_policy<return_by_value>())
        .def("column", columnDouble, return_value_policy<return_by_value>());


#ifdef _INVALID_VERSION
    // for SE3 concrete type.
    class_ <MultiSeqSE3, bases<AbstractMultiSeq>, boost::noncopyable >("AbstractMultiSeqSE3", no_init)
        .def("frameRate", &MultiSeqDouble::frameRate, return_value_policy<return_by_value>())
        .def("timeStep", &MultiSeqDouble::timeStep, return_value_policy<return_by_value>())
        .def("numFrames", &MultiSeqDouble::numFrames, return_value_policy<return_by_value>())
        .def("numParts", &MultiSeqDouble::numParts, return_value_policy<return_by_value>())
        .def("timeLength", &MultiSeqDouble::timeLength, return_value_policy<return_by_value>())
        .def("frameOfTime", &MultiSeqDouble::frameOfTime, return_value_policy<return_by_value>())
        .def("timeOfFrame", &MultiSeqDouble::timeOfFrame, return_value_policy<return_by_value>())
        .def("clampFrameIndex", &MultiSeqDouble::clampFrameIndex)
        .def("frame", frameSE3,  return_value_policy<return_by_value>())
        .def("part", partSE3,  return_value_policy<return_by_value>())
        .def("empty", &MultiSeqDouble::empty, return_value_policy<return_by_value>())
        .def("resize", &MultiSeqDouble::resize)
        .def("resizeColumn", &MultiSeqDouble::resizeColumn)
        .def("rowSize", &MultiSeqDouble::rowSize, return_value_policy<return_by_value>())
        .def("resizeRow", &MultiSeqDouble::resizeRow)
        .def("colSize", &MultiSeqDouble::colSize, return_value_policy<return_by_value>())
        .def("at", atSE3, return_value_policy<return_by_value>())
        .def("row", rowSE3, return_value_policy<return_by_value>())
        .def("column", columnSE3, return_value_policy<return_by_value>());
#endif


#ifdef _INVALID_VERSION
    // for Affine concrete type.
    class_ <MultiSeqAffine, bases<AbstractMultiSeq>, boost::noncopyable >("AbstractMultiSeqAffine", no_init)
        .def("frameRate", &MultiSeqDouble::frameRate, return_value_policy<return_by_value>())
        .def("timeStep", &MultiSeqDouble::timeStep, return_value_policy<return_by_value>())
        .def("numFrames", &MultiSeqDouble::numFrames, return_value_policy<return_by_value>())
        .def("numParts", &MultiSeqDouble::numParts, return_value_policy<return_by_value>())
        .def("timeLength", &MultiSeqDouble::timeLength, return_value_policy<return_by_value>())
        .def("frameOfTime", &MultiSeqDouble::frameOfTime, return_value_policy<return_by_value>())
        .def("timeOfFrame", &MultiSeqDouble::timeOfFrame, return_value_policy<return_by_value>())
        .def("clampFrameIndex", &MultiSeqDouble::clampFrameIndex)
        .def("frame", frameAffine,  return_value_policy<return_by_value>())
        .def("part", partAffine,  return_value_policy<return_by_value>())
        .def("empty", &MultiSeqDouble::empty, return_value_policy<return_by_value>())
        .def("resize", &MultiSeqDouble::resize)
        .def("resizeColumn", &MultiSeqDouble::resizeColumn)
        .def("rowSize", &MultiSeqDouble::rowSize, return_value_policy<return_by_value>())
        .def("resizeRow", &MultiSeqDouble::resizeRow)
        .def("colSize", &MultiSeqDouble::colSize, return_value_policy<return_by_value>())
        .def("at", atAffine, return_value_policy<return_by_value>())
        .def("row", rowAffine, return_value_policy<return_by_value>())
        .def("column", columnAffine, return_value_policy<return_by_value>());
#endif


    /*!
     * @brief Sequence basic information such as body motion.
     */
    class_< MultiValueSeq, bases< MultiSeqDouble >, boost::noncopyable >("MultiValueSeq", init<>());
    // Parent class has all method about this method of class. 
    // But we can use all parent methods by this class.


#ifdef _INVALID_VERSION
    /*!
     * @brief Sequence basic information such as body motion.
     */
    class_< MultiSE3Seq, bases< MultiSeqSE3 >, boost::noncopyable >("MultiSE3Seq", init<>());


    /*!
     * @brief Sequence basic information such as body motion.
     */
    class_< MultiAffine3Seq, bases< MultiSeqAffine >, boost::noncopyable >("MultiAffineSeq", init<>());
#endif
}
