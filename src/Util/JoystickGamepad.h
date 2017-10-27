/**
   \file
   \author Ikumi Susa
*/

#ifndef CNOID_UTIL_JOYSTICK_GAMEPAD_H
#define CNOID_UTIL_JOYSTICK_GAMEPAD_H

#include <cnoid/Joystick>
#include <iostream>

namespace cnoid{

class Gamepad : public Joystick{
public:
    inline double getStickLX(){ return getPosition(0); }
    inline double getStickLY(){ return -getPosition(1); }
    inline double getStickRX(){ return getPosition(2); }
    inline double getStickRY(){ return -getPosition(3); }
    inline double getTriggerL2(){ return (getPosition(6) + 1.0) / 2.0; }
    inline double getTriggerR2(){ return (getPosition(7) + 1.0) / 2.0; }
    inline double getKeyLX(){ return getPosition(4); }
    inline double getKeyLY(){ return -getPosition(5); }
    inline int getKeyLeft() { return getPosition(4) == -1 ? 1 : 0; }
    inline int getKeyRight(){ return getPosition(4) == 1 ? 1 : 0; }
    inline int getKeyUp()   { return getPosition(5) == -1 ? 1 : 0; }
    inline int getKeyDown() { return getPosition(5) == 1 ? 1 : 0; }
    inline int getKeyAX(){ return getButtonState(0); }
    inline int getKeyBO(){ return getButtonState(1); }
    inline int getKeyXS(){ return getButtonState(2); }
    inline int getKeyYT(){ return getButtonState(3); }
    inline int getKeyL1(){ return getButtonState(4); }
    inline int getKeyR1(){ return getButtonState(5); }
    inline int getKeyStickL(){ return getButtonState(8); }
    inline int getKeyStickR(){ return getButtonState(9); }
    inline int getKeyBackShare()   { return getButtonState(6); }
    inline int getKeyStartOptions(){ return getButtonState(7); }
    inline int getKeyHome()        { return getButtonState(10); } // or maybe 12
    inline int getKeyPS4Pad()      { return getButtonState(13) == 1; }

    void print(){
        std::cout << "-------------------" << std::endl;
        std::cout << "LX " << getStickLX() << " ";
        std::cout << "LY " << getStickLY() << " ";
        std::cout << "RX " << getStickRX() << " ";
        std::cout << "RY " << getStickRY() << " ";
        std::cout << "L2 " << getTriggerL2() << " ";
        std::cout << "R2 " << getTriggerR2() << " ";
        std::cout << "KLX " << getKeyLX() << " ";
        std::cout << "KLY " << getKeyLY() << " ";
        std::cout << "KL " << getKeyLeft() << " ";
        std::cout << "KR " << getKeyRight() << " ";
        std::cout << "KU " << getKeyUp() << " ";
        std::cout << "KD " << getKeyDown() << " ";
        std::cout << "KA " << getKeyAX() << " ";
        std::cout << "KB " << getKeyBO() << " ";
        std::cout << "KX " << getKeyXS() << " ";
        std::cout << "KY " << getKeyYT() << " ";
        std::cout << "L1 " << getKeyL1() << " ";
        std::cout << "R1 " << getKeyR1() << " ";
        std::cout << "KL " << getKeyStickL() << " ";
        std::cout << "KR " << getKeyStickR() << " ";
        std::cout << "KBS " << getKeyBackShare() << " ";
        std::cout << "KSO " << getKeyStartOptions() << " ";
        std::cout << "KH " << getKeyHome() << " ";
        std::cout << "KP " << getKeyPS4Pad() << " ";
        std::cout << std::endl;
    }
};

}

#endif
