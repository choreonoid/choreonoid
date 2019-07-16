/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using namespace Multicopter;
using fmt::format;

void
stringArray2Coordinate(std::vector<std::string>& strAry, double& org, double& len, int& size)
{
    if( strAry.size() != 4 ){
        return;
    }
    org = stod(strAry[1]);
    len = stod(strAry[2]);    
    size = stoi(strAry[3]);        
}

bool
FluidEnvironment::get(const Eigen::Vector3d& pos, FluidValue& val) const{

    if(_isNull){
        return false;
    }

    if( _bnds.contain(pos) == false ){
        return false;
    }
    
    int ix = static_cast<int>(((pos.x()-_orgPos.x())/_gridLen.x())+0.5);
    int iy = static_cast<int>(((pos.y()-_orgPos.y())/_gridLen.y())+0.5);
    int iz = static_cast<int>(((pos.z()-_orgPos.z())/_gridLen.z())+0.5);

    bool isFluid= isFluidCell(ix, iy, iz);

    if( isFluid == false ){
        val.density = val.viscosity = 0.0;
        val.velocity = Eigen::Vector3d::Zero();
        val.isFluid = isFluid;
        return true;
    }

    val=_valAry(ix,iy,iz);
    val.isFluid = isFluid;

    return true;    
}

bool
FluidEnvironment::isFluidCell(int ix, int iy, int iz) const
{
    if( _valAry(ix,iy,iz).density == 0.0 ){
        return false;
    }
    else{
        return true;
    }
}

bool
FluidEnvironment::load(const string& fileName)
{
    ifstream in;
    in.open(fileName.data());
    if( !in ){
        UtilityImpl::printErrorMessage(format("Air Definition File({}) is not existed", fileName));
        return false;
    }
    
    FluidEnvironment fulEnv;
        
    string buff;
    vector<string> buffAry;
    long int lineCount=0;

    getline(in, buff);
    lineCount++;
    UtilityImpl::splitStringArray(buff, buffAry);
    if( buffAry.size() != 2 ){
        UtilityImpl::printErrorMessage(format("[Line:{}] Air Definition File is not valid", lineCount));
        return false;
    }
    if( buffAry[0] != AIR_DEFINITION_FILE_TAG){
        UtilityImpl::printErrorMessage(format("[Line:{}] Air Definition File is not valid", lineCount));
        return false;
    }
    if( buffAry[1] != AIR_DEFINITION_FILE_VERSION){
        UtilityImpl::printErrorMessage(format("[Line:{}] Air Definition File is not valid", lineCount));
        return false;
    }

    for(int i=0 ; i<3 ; ++i){
        getline(in, buff);
        lineCount++;
        UtilityImpl::splitStringArray(buff, buffAry);
        try{
            stringArray2Coordinate(buffAry, fulEnv._orgPos[i], fulEnv._gridLen[i], fulEnv._gridSize[i]);
            if(fulEnv._gridLen[i] <=0 || fulEnv._gridSize[i]<= 0 ){
                UtilityImpl::printErrorMessage(format("[Line:{}] Air Definition File is not valid", lineCount));
                return false;
            }
        }
        catch(...){
            return false;
        }
    }
    
    Boxd bnds;    
    bnds.setMin(fulEnv._orgPos);    
    bnds.setMax(fulEnv._orgPos.array()+fulEnv._gridLen.array()*fulEnv._gridSize.array().cast<double>());
    if( bnds.isNull() == true ){
        UtilityImpl::printErrorMessage("Air Definition File is not valid");
        return false;
    }
    fulEnv._bnds = bnds;

    getline(in, buff);
    lineCount++;

    long int xgrid=fulEnv._gridSize.x()+1;
    long int ygrid=fulEnv._gridSize.y()+1;
    long int zgrid=fulEnv._gridSize.z()+1;

    fulEnv._valAry.create(xgrid,ygrid,zgrid);
    while(in && getline(in,buff)){
        lineCount++;
        UtilityImpl::splitStringArray(buff, buffAry);
        Vector3i idx;        
        FluidValue dat;
        if( stringArray2Data(buffAry, idx, dat) == false ){
            UtilityImpl::printErrorMessage(format("[Line:{}] Air Definition File is not valid", lineCount));
            return false;
        }
        if(dat.density<=0 || dat.viscosity <= 0){
            UtilityImpl::printErrorMessage(format("[Line:{}] Air Definition File is not valid", lineCount));
            return false;
        }
        if((idx.x()>=xgrid)||(idx.y()>=ygrid)||(idx.z()>=zgrid)){
            UtilityImpl::printErrorMessage(format("[Line:{}] Air Definition File is not valid", lineCount));
            return false;
        }

        FluidValue tmp=fulEnv._valAry(idx.x(), idx.y(), idx.z());
        if(tmp.density != 0 && tmp.viscosity!=0){
            UtilityImpl::printWarningMessage(format("[Line:{}] That index has already been entered", lineCount));
        }
        fulEnv._valAry(idx.x(), idx.y(), idx.z()) = dat;
    }

    if((lineCount-5)!= xgrid*ygrid*zgrid){
        UtilityImpl::printWarningMessage("The number of input data and the number of grid points are defferent.");
    }

    fulEnv._isNull = false;
        
    *this = fulEnv;
    
    return true;
}


FluidEnvironment::FluidEnvironment()
{
   _isNull = true;
}

FluidEnvironment::~FluidEnvironment()
{

}

void
FluidEnvironment::string2Vector3i(const std::string& line, Eigen::Vector3i& ret)
{    
    Tokenizer<CharSeparator<char>> tokens(line, CharSeparator<char>(","));
    
    vector<int> intAry;
    intAry.reserve(3);
        
    for(auto it=begin(tokens) ; it !=end(tokens) ; ++it){
        intAry.push_back(stoi(it->data()));
    }
    if( intAry.size() != 3 ){
        return;
    }
    for(int i=0 ; i<3 ; ++i){
        ret[i] = intAry[i];
    }
}

bool
FluidEnvironment::stringArray2Data(const std::vector<std::string>& strAry, Eigen::Vector3i& idx, FluidValue& val)
{
    if( strAry.size() != 6 ){
        return false;
    }
    try{
        string2Vector3i(strAry[0], idx);
        val.density  = stod(strAry[1]);
        val.velocity.x() = stod(strAry[2]);
        val.velocity.y() = stod(strAry[3]);
        val.velocity.z() = stod(strAry[4]);
        val.viscosity = stod(strAry[5]);
    }
    catch(...){
        return false;
    }
    return true;
}
