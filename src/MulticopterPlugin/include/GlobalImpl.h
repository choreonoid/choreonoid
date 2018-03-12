/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter{

    enum Axis : uint8_t{
        X,
        Y,
        Z
    };

    template<typename T> T
    clamp(const T& val, const T& min, const T& max) noexcept
    {
        if( val < min ){
            return min;
        }
        if( val > max ){
            return max;
        }
        return val;
    }

    template<typename T, typename U> T
    clampCast(const U& val) noexcept
    {
        static_assert(std::is_integral<T>(), "val type must be interger.");

        if( val < static_cast<U>(std::numeric_limits<T>::min()) ){
            return static_cast<U>(std::numeric_limits<T>::min());
        }
        if( val > static_cast<U>(std::numeric_limits<T>::max()) ){
            return static_cast<U>(std::numeric_limits<T>::max());
        }
        return val;
    }

    template<typename T, typename U> T
    normalize(const U& val, const U& min = std::numeric_limits<U>::min(), const U& max = std::numeric_limits<U>::max()) noexcept
    {
        static_assert(std::is_floating_point<T>(), "returned type must be float or double.");

        T p = (static_cast<T>(val)-static_cast<T>(min))/(static_cast<T>(max)-static_cast<T>(min));
        return clamp(p, static_cast<T>(0.0), static_cast<T>(1.0));
    }

    template<typename T> T
    degreeToRadian(const T& deg) noexcept
    {
        return static_cast<T>(M_PI * deg / 180.0);
    }

    template<typename T> T
    radianToDegree(const T& rad) noexcept
    {
        return static_cast<T>(180.0 * rad / M_PI);
    }

    template<typename T>
    bool isInteger(const T& val)
    {
        return !(val - std::trunc(val));
    }

#ifdef QT_VERSION

    inline bool
    qtCheckStateToBool(const Qt::CheckState& stat) noexcept
    {
        return (stat == Qt::CheckState::Unchecked) ? false : true;
    }

    inline Qt::CheckState
    boolToQtCheckState(const bool stat) noexcept
    {
        return (stat == true) ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    }
#endif

    template<typename T>
    std::vector<T*> fromCnoidDeviceList(const cnoid::DeviceList<T>& devList){
        std::vector<T*> devAry;
        devAry.reserve(devList.size());
        for(auto&& dev : devList){
            devAry.emplace_back(dev);
        }
        return std::move(devAry);
    }
}
