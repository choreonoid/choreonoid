/*!
   @file
   @author Hiroyuki Matsushita
*/

#ifndef CNOID_BODY_DEVICE_SDF_SENSOR_CONVERTER_H_INCLUDED
#define CNOID_BODY_DEVICE_SDF_SENSOR_CONVERTER_H_INCLUDED

#include <map>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/DeviceList>
#include <cnoid/AccelerationSensor>
#include <cnoid/RangeSensor>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RateGyroSensor>
#include <cnoid/ForceSensor>
#include <cnoid/MessageView>
#include <sdf/sdf.hh>
#include <boost/lexical_cast.hpp>
#include <boost/function.hpp>
#include "exportdecl.h"

#define DEBUG_SDF_SENSOR_CONVERTER 0

namespace cnoid {

/**
  @if jp
  @brief ジョイント・リンク毎の変換した Choreonoid デバイスを保持する。
  @endif
 */
class CNOID_EXPORT SDFSensorInfo
{
public:
    /**
      @brief Constructor.
     */
    SDFSensorInfo();

    /**
      @brief Destructor.
     */
    ~SDFSensorInfo();

    /**
      @brief Add device.
      @param[in] device Pointer of device instance.
     */
    void addDevice(DevicePtr device);

    /**
      @brief Get devices.
      @return List of device.
     */
    const DeviceList<>& getDevices() const {
        return devices_;
    }

    /**
      @brief Clear devices.
     */
    void clearDevices() {
        devices_.clear();
    }

private:
    DeviceList<> devices_;
};

/**
  @if jp
  @brief SDF センサ要素の Choreonoid デバイス変換および接続を実施する。
  @endif
 */
class CNOID_EXPORT SDFSensorConverter
{
public:
    /**
      @brief Constructor.
     */
    SDFSensorConverter();

    /**
      @brief Destructor.
     */
    ~SDFSensorConverter();

    /**
      @if jp
      @brief SDF のセンサ情報を Choreonoid のデバイスへ変換する。
      @param[in] name リンク名もしくはジョイント名。
      @param[in] ep 第一引数の名前を持つ要素へのポインタ。
      @else
      @brief Convert from SDF sensor to Choreonoid device instance.
      @param[in] name Link name or Joint name. 
      @param[in] ep Pointer of link element or joint element.
      @endif
     */
    void convert(const std::string name, sdf::ElementPtr ep);

    /**
      @if jp
      @brief 変換したデバイスの Choreonoid のモデルへの取り付けを実施する。
      @param[in] body モデルの cnoid::Body ポインタ。
      @attention 本メソッドを実行した場合、取り付け実行後、変換したデバイスを全て消去する。
                 本メソッドは、Choreonoid のリンクツリー構築完了後に呼び出す想定である事から、
                 このような実装となる。
      @endif
     */
    void attachDevices(Body* body);

    /**
      @if jp
      @brief convert メソッドで変換した内部情報のマージを実施する。
             
             SDF ではリンクとジョイントを個別に扱うが、Choreonoid ではリンクとジョ
             イントが同じ扱いである事から、Choreonoid のリンクオブジェクト生成後の
             名前指定 (cnoid::Link setName()) において、SDF でのリンク名もしくは、
             ジョイント名のいずれかを指定する事となる。
             
             センサデバイスを構築する本クラスの attachDevices メソッドは、第二引数
             のリンクの名前 (cnoid::Link name()) をキーに内部情報を検索し構築処理を
             実施する為、前述で指定しなかった名前のデバイス情報が使用されない状態と
             なる。
             
             この状況を回避する為、リンクオブジェクトの名前指定後、本メソッドを呼び
             出し内部情報のマージを実施する必要がある。
      @param[in] src マージ元の名前を指定。
                     マージ元が存在しない場合は何も処理を実施しない。
      @param[in] dst マージ先の名前を指定。
                     マージ先が存在しない場合は新規作成する。
      @attention ルートリンク名にジョイント名ではなくリンク名を使用している場合、
                 本メソッド呼び出し前に setRootLinkName メソッドでルートリンク名を
                 設定する必要がある。
                 前述の条件に合致し、これを実施しない場合、ルートリンクへ設定されて
                 いるデバイスの設定を保証しない。
      @endif
     */
    void mergeInfo(const std::string src, const std::string dst);

    /**
      @if jp
      @brief 変換したデバイス情報を全て消去する。
      @else
      @brief Clear all devices.
      @endif
     */
    void clearAllDevices();

    /**
      @brief Set root link name.
      @param[in] name Root link name.
     */
    void setRootLinkName(std::string name) {
        rootLinkName_ = name;
    }

    /**
      @brief Get root link name.
      @return Root link name.
     */
    const std::string getRootLinkName() const {
        return rootLinkName_;
    }

    /**
      @brief Get SDFormat version string.
      @return SDFormat version string.
     */
    const std::string sdformatVersion() const {
        return sdformatVersion_;
    }

#if (DEBUG_SDF_SENSOR_CONVERTER > 0) /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
    /**
      @brief dumping of sensor devices.
     */
    void dumpDeviceMap();
#endif                               /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */

private:
    enum CameraType { UNKNOWN, ORDINARY, RANGE, POINT_CLOUD };

    std::string rootLinkName_;
    std::map<std::string, SDFSensorInfo*> deviceMap_;
    std::string sdformatVersion_;

    /**
      @brief Get SDFSensorInfo instance.
      @praram[in] key Set link name or joint name.
      @return Pointer of SDFSensorInfo.
     */
    SDFSensorInfo* getSDFSensorInfo(const std::string key);

    /**
      @brief remove SDFSensorInfo instance.
      @praram[in] key link name or joint name.
     */
    void removeSDFSensorInfo(const std::string key);

    /**
      @brief Set device's pose.
      @param[in] device Set pointer of cnoid::DevicePtr.
      @param[in] pose Set pointer of pose element.
     */
    void setDevicePose(DevicePtr device, const sdf::Pose& pose);

    /**
      @brief Create Choreonoid's device instance.
      @param[in] key Set link name or joint name.
      @param[in] ep Set pointer of link element or joint element.
      @param[in] parent Set pointer of parent elemnt of ep.
     */
    void createDevice(const std::string key, sdf::ElementPtr ep, sdf::ElementPtr parent);

    // IMU

    /**
      @brief Setting IMU parameter. (for sdformat version 1.4)
      @param[in] asensor Set pointer of AccelerationSensor.
      @param[in] gsensor Set pointer of RateGyroSensor.
      @param[in] ep Set pointer of sensor element.
      @retval true Success.
      @retval false Fail.
     */
    bool setIMUParameter14(AccelerationSensorPtr asensor, RateGyroSensorPtr gsensor, sdf::ElementPtr ep);

    /**
      @brief Validate IMU noise element. (for sdformat version greater than 1.4)
      @param ep Set pointer of axis (x or y or z) element.
      @retval true Valid.
      @retval false Invalid.
     */
    bool validateIMUNoiseElement(sdf::ElementPtr ep);

    /**
      @brief Setting IMU parameter. (for sdformat version greater than 1.4)
      @param[in] asensor Set pointer of AccelerationSensor.
      @param[in] gsensor Set pointer of RateGyroSensor.
      @param[in] ep Set pointer of sensor element.
      @retval true Success.
      @retval false Fail.
     */
    bool setIMUParameter(AccelerationSensorPtr asensor, RateGyroSensorPtr gsensor, sdf::ElementPtr ep);

    /**
      @if jp
      @brief Create imu device. (SDF sensor of type imu)
      Choreonoid デバイスの変換に際し、AccelerationSensor と RateGyroSensor の 2 つのデバイスを生成する。
      また、名前の重複を避けるため、それぞれ要素の名前の後ろに _accel, _rate を付加した名前を使用する。
      @param[in] key センサが設けられているリンク名・ジョイント名を指定する。
      @param[in] name センサの名前を指定する。
      @param[in] ep センサ要素のポインタを指定する。
      @else
      @brief Create imu device. (SDF sensor of type imu)
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
      @endif
     */
    void createIMUDevice(const std::string key, const std::string name, sdf::ElementPtr ep);

    // Camera

    /**
      @brief Validate camera element.
      @param[in] cp Pointer of camera element.
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @retval true Valid.
      @retval false Invalid.
     */
    bool validateCameraElement(sdf::ElementPtr cp, const std::string key, const std::string name);

    /**
      @brief Create device of type of camera.
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
      @param[in] type Camera type. (SDFSensorConverter::ORDINARY, RANGE, POINT_CLOUD)
     */
    void createCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep,
                            const SDFSensorConverter::CameraType type);

    /**
      @if jp
      @brief Create multicamera device. (SDF sensor of type multicamera)
      Choreonoid でのデバイス名指定の際、要素の name 属性が空、もしくは値が __default__ の場合、
      指定する名前に 0 から始まる一意の番号を、アンダースコアで繋ぎ付加する。
      @param[in] key センサが設けられているリンク名・ジョイント名を指定する。
      @param[in] name センサの名前を指定する。
      @param[in] ep センサ要素のポインタを指定する。
      @else
      @brief Create multicamera device. (SDF sensor of type multicamera)
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
      @endif
     */
    void createMultiCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep);

    /**
      @brief Create depth device. (SDF sensor of type depth_device)
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
     */
    void createDepthCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep);

    /**
      @brief Create camera device. (SDF sensor of type camera)
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
     */
    void createOrdinaryCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep);

    // ForceSensor

    /**
      @brief Create force torque device. (SDF sensor of type force_torque)
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
     */
    void createForceTorqueDevice(const std::string key, const std::string name, sdf::ElementPtr ep);

    // RangeSensor

    /**
      @if jp
      @brief Convert from SDF angle to Choreonoid angle.
      Choreonoid の設定しきい値が 170 度までとなっている為、変換の結果が 170 度を越えた場合、
      変換失敗とし Choreonoid のメッセージビューへエラーメッセージを出力する。
      @param[out] angle Store returning value.
      @param[in] min SDF min angle.
      @param[in] max SDF max angle.
      @param[in] defaultValue default angle.
      @param[in] key Set link name or joint name.
      @param[in] name Set sensor name. (name attribute of <sensor>)
      @retval true Convert success.
      @retval false Convert fail.
      @else
      @brief Convert from SDF angle to Choreonoid angle.
      @param[out] angle Store returning value.
      @param[in] min SDF min angle.
      @param[in] max SDF max angle.
      @param[in] defaultValue default angle.
      @param[in] key Set link name or joint name.
      @param[in] name Set sensor name. (name attribute of <sensor>)
      @retval true Convert success.
      @retval false Convert fail.
      @endif
     */
    bool convertAngle(double* angle, double min, double max, const double defaultValue, const std::string key,
                      const std::string name);

    /**
      @brief Create ray device. (SDF sensor of type ray)
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
     */
    void createRayDevice(const std::string key, const std::string name, sdf::ElementPtr ep);

    /**
      @brief Create GPU ray device. (SDF sensor of type gpu_ray)
      @param[in] key link name or joint name.
      @param[in] name Sensor name.
      @param[in] ep Pointer of sensor element.
     */
    void createGPURayDevice(const std::string key, const std::string name, sdf::ElementPtr ep);
};

}

#endif    /* CNOID_BODY_DEVICE_SDF_SENSOR_CONVERTER_H_INCLUDED */
