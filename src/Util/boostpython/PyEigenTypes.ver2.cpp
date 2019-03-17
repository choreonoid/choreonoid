/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"
#include <fmt/format.h>

namespace python = boost::python;
using namespace boost::python;
using namespace cnoid;

namespace {

template<typename VectorType, int dim>
struct pylist_to_Vector_converter {
    pylist_to_Vector_converter(){
        converter::registry::push_back(
            &pylist_to_Vector_converter::convertible,
            &pylist_to_Vector_converter::construct,
            python::type_id<VectorType>());
    }
    static void* convertible(PyObject* pyo){
        if(PySequence_Check(pyo) && PySequence_Size(pyo) == dim){
            return pyo;
        }
        return 0;
    }
    static void construct(PyObject* pyo, python::converter::rvalue_from_python_stage1_data* data){
        VectorType* pv = new(reinterpret_cast<python::converter::rvalue_from_python_storage<VectorType>*>(data)->storage.bytes) VectorType();
        for(python::ssize_t i = 0; i < dim; ++i) {
            (*pv)[i] = python::extract<typename VectorType::Scalar>(PySequence_GetItem(pyo, i));
        }
        data->convertible = pv;
    }
};

template<typename MatrixType, int rows, int cols>
struct pylist_to_Matrix_converter {
    pylist_to_Matrix_converter(){
        converter::registry::push_back(
            &pylist_to_Matrix_converter::convertible,
            &pylist_to_Matrix_converter::construct,
            python::type_id<MatrixType>());
    }
    static void* convertible(PyObject* pyo){
        if(PySequence_Check(pyo)){
            return pyo;
        }
        return 0;
    }
    static void construct(PyObject* pyo, python::converter::rvalue_from_python_stage1_data* data){
        MatrixType* M =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<MatrixType>*>(data)->storage.bytes)
            MatrixType();
        for(python::ssize_t i = 0; i < rows; ++i) {
            python::list row = python::extract<python::list>(PySequence_GetItem(pyo, i));
            for(python::ssize_t j = 0; j < rows; ++j) {
                (*M)(i, j) = python::extract<typename MatrixType::Scalar>(row[j]);
            }
        }
        data->convertible = M;
    }
};

template<typename TransformType>
struct pylist_to_Transform_converter {
    pylist_to_Transform_converter(){
        converter::registry::push_back(
            &pylist_to_Transform_converter::convertible,
            &pylist_to_Transform_converter::construct,
            python::type_id<TransformType>());
    }
    static void* convertible(PyObject* pyo){
        if(PySequence_Check(pyo)){
            int numRows = PySequence_Size(pyo);
            if(numRows == 3 || numRows == 4){
                return pyo;
            }
        }
        return 0;
    }
    static void construct(PyObject* pyo, python::converter::rvalue_from_python_stage1_data* data){
        TransformType* T =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<TransformType>*>(data)->storage.bytes)
            TransformType();
        for(python::ssize_t i = 0; i < 3; ++i) {
            typename TransformType::LinearPart R = T->linear();
            typename TransformType::TranslationPart p = T->translation();
            python::list row = python::extract<python::list>(PySequence_GetItem(pyo, i));
            R(i, 0) = python::extract<typename TransformType::Scalar>(row[0]);
            R(i, 1) = python::extract<typename TransformType::Scalar>(row[1]);
            R(i, 2) = python::extract<typename TransformType::Scalar>(row[2]);
            p[i] = python::extract<typename TransformType::Scalar>(row[3]);
        }
        data->convertible = T;
    }
};


//! \todo replace this function with another python functions
Affine3 getAffine3FromAngleAxis(double angle, const Vector3& vec){
    /*
    Affine3 m;
    m = AngleAxis(angle, vec);
    return m;
    */
    return Affine3(AngleAxis(angle, vec));
}


typedef Eigen::Matrix<int,1,1>::Index Index;

void checkIndex(Index i, Index size) {
    if(i < 0 || i >= size) {
        static string message("Index {0} out of range 0..{1}");
        PyErr_SetString(PyExc_IndexError, fmt::format(message, i, (size - 1)).c_str());
        python::throw_error_already_set();
    }
}

void extractIndex(python::tuple indexTuple, Index rows, Index cols, Index out_index[2]){
    if(python::len(indexTuple) != 2) {
        PyErr_SetString(PyExc_IndexError,"Index must be integer or a 2-tuple");
        python::throw_error_already_set();
    }
    for(int i=0; i < 2; ++i) {
        python::extract<Index> e(indexTuple[i]);
        if(!e.check()){
            PyErr_SetString(
                PyExc_ValueError,
                fmt::format("Unable to convert {}-th index to integer.", i).c_str());
            python::throw_error_already_set();
        }
        out_index[i] = e(); 
    }
    checkIndex(out_index[0], rows);
    checkIndex(out_index[1], cols);
}

std::string getClassName(const python::object& obj){
    return python::extract<std::string>(obj.attr("__class__").attr("__name__"))();
}

template<typename TMatrixBase>
class MatrixBaseVisitor : public python::def_visitor<MatrixBaseVisitor<TMatrixBase> >{

    typedef typename TMatrixBase::Scalar Scalar;
    
public:
    template<class PyClass>
    void visit(PyClass& pyClass) const {
        pyClass
            .def("__neg__", &MatrixBaseVisitor::__neg__)
            .def("__add__", &MatrixBaseVisitor::__add__)
            .def("__iadd__", &MatrixBaseVisitor::__iadd__)
            .def("__sub__", &MatrixBaseVisitor::__sub__)
            .def("__isub__", &MatrixBaseVisitor::__isub__)
            .def("__eq__", &MatrixBaseVisitor::__eq__)
            .def("__ne__", &MatrixBaseVisitor::__ne__)

            .def("__mul__", &MatrixBaseVisitor::__mul__scalar<long>)
            .def("__imul__", &MatrixBaseVisitor::__imul__scalar<long>)
            .def("__rmul__", &MatrixBaseVisitor::__rmul__scalar<long>)
            .def("__div__",&MatrixBaseVisitor::__div__scalar<long>)
            .def("__idiv__",&MatrixBaseVisitor::__idiv__scalar<long>)

            .def("__mul__",&MatrixBaseVisitor::__mul__scalar<Scalar>)
            .def("__rmul__",&MatrixBaseVisitor::__rmul__scalar<Scalar>)
            .def("__imul__",&MatrixBaseVisitor::__imul__scalar<Scalar>)
            .def("__div__",&MatrixBaseVisitor::__div__scalar<Scalar>)
            .def("__idiv__",&MatrixBaseVisitor::__idiv__scalar<Scalar>)
            
            .def("rows", &TMatrixBase::rows)
            .def("cols", &TMatrixBase::cols)
            .add_static_property("Ones", &MatrixBaseVisitor::Ones)
            .add_static_property("Zero", &MatrixBaseVisitor::Zero)
            .add_static_property("Identity", &MatrixBaseVisitor::Identity)

            .def("norm",&TMatrixBase::norm)
            .def("__abs__",&TMatrixBase::norm)
            .def("squaredNorm",&TMatrixBase::squaredNorm)
            .def("normalize",&TMatrixBase::normalize)
            .def("normalized",&TMatrixBase::normalized)
            ;
    }

    static TMatrixBase Ones(){ return TMatrixBase::Ones(); }
    static TMatrixBase Zero(){ return TMatrixBase::Zero(); }
    static TMatrixBase Identity(){ return TMatrixBase::Identity(); }

    static bool __eq__(const TMatrixBase& a, const TMatrixBase& b){
        if(a.rows()!=b.rows() || a.cols()!=b.cols()){
            return false;
        }
        return a.cwiseEqual(b).all();
    }
    static bool __ne__(const TMatrixBase& a, const TMatrixBase& b){ return !__eq__(a,b); }
    static TMatrixBase __neg__(const TMatrixBase& a){ return -a; };
    static TMatrixBase __add__(const TMatrixBase& a, const TMatrixBase& b){ return a + b; }
    static TMatrixBase __sub__(const TMatrixBase& a, const TMatrixBase& b){ return a - b; }
    static TMatrixBase __iadd__(TMatrixBase& a, const TMatrixBase& b){ a += b; return a; };
    static TMatrixBase __isub__(TMatrixBase& a, const TMatrixBase& b){ a -= b; return a; };

    template<typename Scalar2> static TMatrixBase __mul__scalar(const TMatrixBase& a, const Scalar2& scalar){ return a * scalar; }
    template<typename Scalar2> static TMatrixBase __imul__scalar(TMatrixBase& a, const Scalar2& scalar){ a *= scalar; return a; }
    template<typename Scalar2> static TMatrixBase __rmul__scalar(const TMatrixBase& a, const Scalar2& scalar){ return a * scalar; }
    template<typename Scalar2> static TMatrixBase __div__scalar(const TMatrixBase& a, const Scalar2& scalar){ return a / scalar; }
    template<typename Scalar2> static TMatrixBase __idiv__scalar(TMatrixBase& a, const Scalar2& scalar){ a /= scalar; return a; }
};

template<typename TVector>
class VectorVisitor : public python::def_visitor< VectorVisitor<TVector> > {

    friend class python::def_visitor_access;
    typedef typename TVector::Scalar Scalar;
    enum { Dim = TVector::RowsAtCompileTime };

public:    
    template<class PyClass> void visit(PyClass& pyClass) const {

        MatrixBaseVisitor<TVector>().visit(pyClass);

        pyClass
            .def(python::init<Scalar, Scalar, Scalar>())
            .def("__setitem__", &VectorVisitor::__setitem__)
            .def("__getitem__", &VectorVisitor::__getitem__)
            .def("__str__", &VectorVisitor::__str__)
            .def("__repr__", &VectorVisitor::__str__)
            .def("dot", &VectorVisitor::dot)
            .def("__len__", &VectorVisitor::__len__).staticmethod("__len__")
            .def("cross", &VectorVisitor::cross)
            .add_static_property("UnitX", &VectorVisitor::UnitX)
            .add_static_property("UnitY", &VectorVisitor::UnitY)
            .add_static_property("UnitZ", &VectorVisitor::UnitZ)
            .add_property("x", &VectorVisitor::get_x, &VectorVisitor::set_x)
            .add_property("y", &VectorVisitor::get_y, &VectorVisitor::set_y)
            .add_property("z", &VectorVisitor::get_z, &VectorVisitor::set_z)
            ;
    };

    static void __setitem__(TVector& self, Index i, Scalar value){
        checkIndex(i, Dim);
        self[i] = value;
    }
    static Scalar __getitem__(const TVector& self, Index i){
        checkIndex(i, Dim);
        return self[i];
    }
    static Index __len__(){ return Dim; }
    static Scalar dot(const TVector& self, const TVector& other){ return self.dot(other); }
    static TVector cross(const TVector& self, const TVector& other){ return self.cross(other); }
    static TVector UnitX(){ return TVector::UnitX(); }
    static TVector UnitY(){ return TVector::UnitY(); }
    static TVector UnitZ(){ return TVector::UnitZ(); }
    static Scalar get_x(const TVector& self) { return self.x(); }
    static Scalar set_x(TVector& self, Scalar value) { self.x() = value; }
    static Scalar get_y(const TVector& self) { return self.y(); }
    static Scalar set_y(TVector& self, Scalar value) { self.y() = value; }
    static Scalar get_z(const TVector& self) { return self.z(); }
    static Scalar set_z(TVector& self, Scalar value) { self.z() = value; }
            
    static std::string __str__(const python::object& obj){
        const TVector& self = python::extract<TVector>(obj)();
        std::ostringstream oss;
        oss << getClassName(obj) << "(";
        Index i=0;
        while(true){
            oss << self[i++];
            if(i == Dim){
                break;
            }
            oss << ", ";
        }
        oss << ")";
        return oss.str();
    }
};

template<typename TMatrix>
class MatrixVisitor : public python::def_visitor< MatrixVisitor<TMatrix> > {

    friend class python::def_visitor_access;
    typedef typename TMatrix::Scalar Scalar;
    typedef typename Eigen::Matrix<Scalar, TMatrix::RowsAtCompileTime, 1> ColumnVector;
    enum { Rows = TMatrix::RowsAtCompileTime };
    enum { Cols = TMatrix::RowsAtCompileTime };

public:
    template<class PyClass>
    void visit(PyClass& pyClass) const {

        MatrixBaseVisitor<TMatrix>().visit(pyClass);

	pyClass
            .def("transpose", &MatrixVisitor::transpose)
            .def("inverse", &MatrixVisitor::inverse)
            .def("diagonal",&MatrixVisitor::diagonal)
            .def("row", &MatrixVisitor::row)
            .def("col", &MatrixVisitor::col)
            .def("__mul__", &MatrixVisitor::__mul__matrix)
            .def("__imul__",&MatrixVisitor::__imul__matrix)
            .def("__mul__", &MatrixVisitor::__mul__vector)
            .def("__rmul__",&MatrixVisitor::__mul__vector)
            .def("__setitem__", &MatrixVisitor::__setitem__row)
            .def("__getitem__", &MatrixVisitor::__getitem__row)
            .def("__setitem__", &MatrixVisitor::__setitem__element)
            .def("__getitem__", &MatrixVisitor::__getitem__element)
            .def("__str__", &MatrixVisitor::__str__)
            .def("__repr__", &MatrixVisitor::__str__)
            .def("__len__", &MatrixVisitor::__len__).staticmethod("__len__")
            ;
	}
    
    static Index __len__(){ return TMatrix::RowsAtCompileTime; }
    static TMatrix transpose(const TMatrix& M){ return M.transpose(); }
    static ColumnVector diagonal(const TMatrix& M){ return M.diagonal(); }

    static ColumnVector __getitem__row(const TMatrix& M, Index i){
        checkIndex(i, M.rows());
        return M.row(i);
    }
    static void __setitem__row(TMatrix& M, Index i, const ColumnVector& v){
        checkIndex(i, M.rows());
        M.row(i) = v;
    }
    static Scalar __getitem__element(const TMatrix& M, python::tuple indexTuple){
        Index index[2];
        extractIndex(indexTuple, M.rows(), M.cols(), index);
        return M(index[0], index[1]);
    }
    static void __setitem__element(TMatrix& M, python::tuple indexTuple, const Scalar& value){
        Index index[2];
        extractIndex(indexTuple, M.rows(), M.cols(), index);
        M(index[0], index[1]) = value;
    }

    static TMatrix __mul__matrix(const TMatrix& A, const TMatrix& B){ return A * B; }
    static TMatrix __imul__matrix(TMatrix& A, const TMatrix& B){ A *= B; return A; };
    static ColumnVector __mul__vector(const TMatrix& M, const ColumnVector& v){ return M * v; }
    static TMatrix inverse(const TMatrix& M){ return M.inverse(); }
    static ColumnVector row(const TMatrix& M, Index i){ checkIndex(i, M.rows()); return M.row(i); }
    static ColumnVector col(const TMatrix& M, Index i){ checkIndex(i, M.cols()); return M.col(i); }
    
    static std::string __str__(const python::object& obj){
        std::ostringstream oss;
        const TMatrix& M = python::extract<TMatrix>(obj)();
        oss << getClassName(obj) << "(";
        for(Index i=0; i < M.rows(); ++i){
            oss << "\t(";
            Index j=0;
            while(true){
                oss << M(i, j++);
                if(j == Cols){
                    break;
                }
                oss << ", ";
            }
            oss < ")\n";
        }
        oss < ")";
        return oss.str();
    }
};


template<typename TTransform>
class TransformVisitor : public python::def_visitor< TransformVisitor<TTransform> > {

    friend class python::def_visitor_access;
    typedef typename TTransform::Scalar Scalar;
    typedef typename TTransform::MatrixType MatrixType;
    typedef typename TTransform::LinearMatrixType LinearMatrixType;
    typedef typename TTransform::VectorType VectorType;

    static TTransform Identity_;
    
public:
    /*
    TransformVisitor() {
        Identity_.setIdentity();
    }
    */
    
    template<class PyClass>
    void visit(PyClass& pyClass) const {

	pyClass
            .def("__setitem__", &TransformVisitor::__setitem__element)
            .def("__getitem__", &TransformVisitor::__getitem__element)
            .def("matrix", &TransformVisitor::matrix)
            .def("linear", &TransformVisitor::linear)
            .def("rotation", &TransformVisitor::rotation)
            .def("translation", &TransformVisitor::translation)
            .def("inverse", &TransformVisitor::inverse)
            .def("setIdentity", &TTransform::setIdentity).staticmethod("setIdentity")
            .add_static_property("Identity", &TransformVisitor::Identity)
            .add_static_property("Dim", &TransformVisitor::Dim)            
            .add_static_property("HDim", &TransformVisitor::HDim)            
            .add_static_property("Rows", &TransformVisitor::Rows)            
            .def("__str__", &TransformVisitor::__str__)
            .def("__repr__", &TransformVisitor::__str__)
            ;
	}

    static Scalar __getitem__element(const TTransform& T, python::tuple indexTuple){
        Index index[2];
        extractIndex(indexTuple, TTransform::Rows, TTransform::HDim, index);
        return T(index[0], index[1]);
    }
    static void __setitem__element(TTransform& M, python::tuple indexTuple, const Scalar& value){
        Index index[2];
        extractIndex(indexTuple, TTransform::Rows, TTransform::HDim, index);
        M(index[0], index[1]) = value;
    }

    static MatrixType matrix(const TTransform& T){ return T.matrix(); }
    static LinearMatrixType linear(const TTransform& T){ return T.linear(); }
    static LinearMatrixType rotation(const TTransform& T){ return T.rotation(); }    
    static VectorType translation(const TTransform& T){ return T.translation(); }
    static TTransform inverse(const TTransform& T){ return T.inverse(); }

    static TTransform Identity(){ return Identity_; }
    
    static Index Dim() { return TTransform::Dim; }
    static Index HDim() { return TTransform::HDim; }
    static Index Rows() { return TTransform::Rows; }    

    static std::string __str__(const python::object& obj){
        std::ostringstream oss;
        const TTransform& T = python::extract<TTransform>(obj)();
        oss << getClassName(obj) << "(";
        for(Index i=0; i < TTransform::Rows; ++i){
            oss << "\t(";
            Index j=0;
            while(true){
                oss << T(i, j++);
                if(j == TTransform::HDim){
                    break;
                }
                oss << ", ";
            }
            oss < ")\n";
        }
        oss < ")";
        return oss.str();
    }
};

template <typename TTransform> TTransform TransformVisitor<TTransform>::Identity_ = TTransform::Identity();

}


namespace cnoid {

void exportPyEigenTypes()
{
    class_<Vector3>("Vector3", init<>())
        .def(VectorVisitor<Vector3>());

    class_<Vector3f>("Vector3f", init<>())
        .def(VectorVisitor<Vector3f>());

    class_<Matrix3>("Matrix3", init<>())
        .def(MatrixVisitor<Matrix3>());
    
    class_<Matrix4>("Matrix4", init<>())
        .def(MatrixVisitor<Matrix4>());

    class_<Affine3>("Affine3", init<>())
        .def(TransformVisitor<Affine3>());

    class_<Position>("Position", init<>())
        .def(TransformVisitor<Position>());
    
    void (SE3::*SE3_set1)(const Vector3& translation, const Quat& rotation) = &SE3::set;
    void (SE3::*SE3_set2)(const Vector3& translation, const Matrix3& R) = &SE3::set;
    void (SE3::*SE3_set3)(const Position& T) = &SE3::set;
    const Vector3& (SE3::*SE3_translation_const)() const = &SE3::translation;
    const Quat& (SE3::*SE3_rotation_const)() const = &SE3::rotation;

    class_<SE3, boost::noncopyable>("SE3", init<>()) 
        .def("set", SE3_set1)
        .def("set", SE3_set2)
        .def("set", SE3_set3)
        .def("translation", SE3_translation_const, return_value_policy<copy_const_reference>())
        .def("rotation", SE3_rotation_const, return_value_policy<copy_const_reference>());

    python::def("rpyFromRot", cnoid::rpyFromRot);
    Matrix3 (*cnoid_rotFromRpy1)(const Vector3& rpy) = cnoid::rotFromRpy;
    python::def("rotFromRpy", cnoid_rotFromRpy1);
    python::def("omegaFromRot", cnoid::omegaFromRot);

    //! \todo replace the following functions with another python functions
    python::def("angleAxis", getAffine3FromAngleAxis);

    pylist_to_Vector_converter<Vector2, 2>();
    pylist_to_Vector_converter<Vector3, 3>();
    pylist_to_Vector_converter<Vector4, 4>();
    pylist_to_Vector_converter<Vector6, 6>();
    pylist_to_Matrix_converter<Matrix3, 3, 3>();
    pylist_to_Matrix_converter<Matrix4, 4, 4>();
    pylist_to_Transform_converter<Affine3>();
    pylist_to_Transform_converter<Position>();
}

}

