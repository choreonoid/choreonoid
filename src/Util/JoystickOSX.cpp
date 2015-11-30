/**
   Joystick class implementation for Mac OS X.
   This implementation is based on the joystick reader program
   distributed by developed by ysflight.com.
*/

#include "Joystick.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <vector>
#include <string>
#include <iostream>
#include <sys/errno.h>
#include <sysexits.h>
#include <IOKit/hid/IOHIDLib.h>

using namespace std;
using namespace cnoid;

namespace {

const int MaxNumJoysticks = 4;
const int YsJoyReaderMaxNumAxis = 6;
const int YsJoyReaderMaxNumButton = 32;
const int YsJoyReaderMaxNumHatSwitch = 4;

class YsJoyReaderElement
{
public:
    int exist;
    IOHIDElementRef elem;
    int value;
    YsJoyReaderElement() {
        exist = 0;
        elem = NULL;
        value = 0;
    }
};
    
class YsJoyReaderAxis : public YsJoyReaderElement
{
public:
    int min,max;
    int scaledMin,scaledMax;
    int calibCenter,calibMin,calibMax;

    YsJoyReaderAxis() {
        min = 0;
        max = 0;
        scaledMin = 0;
        scaledMax = 0;
        calibCenter = 0;
        calibMin = 0;
        calibMax = 0;
    }
    double GetCalibratedValue(void) const {
        double calib;
        if(calibCenter < value && calibMax != calibCenter){
            calib = (double)(value - calibCenter)/(double)(calibMax - calibCenter);
        } else if(value<calibCenter && calibMin!=calibCenter){ 
            calib = (double)(value - calibCenter)/(double)(calibCenter - calibMin);
        } else {
            return 0.0;
        }
        if(calib>1.0){
            calib = 1.0;
        }
        if(calib<-1.0){
            calib = -1.0;
        }
        return calib;
    }
    void CaptureCenter(void) {
        calibCenter = value;
    }
    void BeginCaptureMinMax(void) {
        calibMin = calibCenter + 1000;
        calibMax = calibCenter - 1000;
    }
    void CaptureMinMax(void) {
        if(value < calibMin) {
            calibMin = value;
        }
        if(value > calibMax) {
            calibMax = value;
        }
    }
    void CenterFromMinMax(void){
        calibCenter = (calibMin + calibMax) / 2;
    }
};

class YsJoyReaderButton : public YsJoyReaderElement
{
public:
    YsJoyReaderButton() { }
};

class YsJoyReaderHatSwitch : public YsJoyReaderElement
{
public:
    int valueNeutral;
    int value0Deg;
    int value90Deg;
    int value180Deg;
    int value270Deg;
        
    YsJoyReaderHatSwitch() {
        valueNeutral = 0;
        value0Deg = 1;
        value90Deg = 3;
        value180Deg = 5;
        value270Deg = 7;
    }
    int GetDiscreteValue(void) const {
        if(value == valueNeutral){
            return 0;
        } else if(value==value0Deg){
            return 1;
        } else if(value==value90Deg){
            return 3;
        } else if(value==value180Deg) {
            return 5;
        } else if(value270Deg==value) {
            return 7;
        } else if(value0Deg<value && value<value90Deg){
            return 2;
        } else if(value90Deg<value && value<value180Deg){
            return 4;
        } else if(value180Deg<value && value<value270Deg){
            return 6;
        } else if(value270Deg<value) {
            return 8;
        }
        return 0;
    }
};

void CFSetCopyCallBack(const void *value,void *context)
{
    CFArrayAppendValue((CFMutableArrayRef)context,value);
}
    
class  YsJoyReader
{
public:
    static IOHIDManagerRef hidManager;
    static CFMutableArrayRef devArray;

    int joyId;
    IOHIDDeviceRef hidDev;
    char regPath[512];

    YsJoyReaderAxis axis[YsJoyReaderMaxNumAxis];
    YsJoyReaderButton button[YsJoyReaderMaxNumButton];
    YsJoyReaderHatSwitch hatSwitch[YsJoyReaderMaxNumHatSwitch];

    YsJoyReader()
        {
            hidDev = NULL;
        }

    int SetUpInterface(int joyId, IOHIDDeviceRef hidDev)
        {
            this->joyId = joyId;

            if(hidDev != NULL) {
                CFArrayRef elemAry = IOHIDDeviceCopyMatchingElements(hidDev, NULL, 0);
                int nElem = (int)CFArrayGetCount(elemAry);
                int isMouse = 0;
                int isJoystick = 0;
                int isKeyboard = 0;
                int isGamePad = 0;

                printf("This HID Device has %d elements.\n", nElem);

                for(int j=0; j < nElem; j++){
                    IOHIDElementRef elem = (IOHIDElementRef)CFArrayGetValueAtIndex(elemAry, j);
                    IOHIDElementType elemType = IOHIDElementGetType(elem);
                    unsigned int usage = IOHIDElementGetUsage(elem);
                    unsigned int usagePage = IOHIDElementGetUsagePage(elem);

                    printf("Element %3d",j);
                    switch(elemType){
                    case kIOHIDElementTypeInput_ScanCodes:
                        printf(" ScanCode  ");
                        break;
                    case kIOHIDElementTypeInput_Misc:
                        printf(" Misc      ");
                        break;
                    case kIOHIDElementTypeInput_Button:
                        printf(" Button    ");
                        break;
                    case kIOHIDElementTypeInput_Axis:
                        printf(" Axis      ");
                        break;
                    case kIOHIDElementTypeOutput:
                        printf(" Output    ");
                        break;
                    case kIOHIDElementTypeFeature:
                        printf(" Feature   ");
                        break;
                    case kIOHIDElementTypeCollection:
                        printf(" Collection");
                        break;
                    }

                    printf("  Usage %3d  UsagePage %3d\n", usage, usagePage);

                    if(kHIDPage_GenericDesktop == usagePage){
                        switch(usage){
                        case kHIDUsage_GD_Mouse:
                            printf("    Can function as mouse\n");
                            isMouse=1;
                            break;
                        case kHIDUsage_GD_Keyboard:
                            printf("    Can function as Keyboard\n");
                            isKeyboard=1;
                            break;
                        case kHIDUsage_GD_Joystick:
                            printf("    Can function as Joystick\n");
                            isJoystick=1;
                            break;
                        case kHIDUsage_GD_GamePad:
                            printf("    Can function as GamePad\n");
                            isGamePad=1;
                            break;
                        }
                    }
                }
                
                if(0!=isJoystick){
                    int nAxis=0;
                    int nHat=0;
                    for(int j=0; j<nElem; j++) {
                        IOHIDElementRef elem = (IOHIDElementRef)CFArrayGetValueAtIndex(elemAry, j);
                        IOHIDElementType elemType = IOHIDElementGetType(elem);
                        unsigned int usage = IOHIDElementGetUsage(elem);
                        unsigned int usagePage = IOHIDElementGetUsagePage(elem);
                        // The following two returned 0 and 255
                        // IOHIDElementGetPhysicalMin(elem);
                        // IOHIDElementGetPhysicalMax(elem);
                        int min = IOHIDElementGetLogicalMin(elem);
                        int max = IOHIDElementGetLogicalMax(elem);
                        int scaledMin = min;
                        int scaledMax = max;
                        
                        if(elemType == kIOHIDElementTypeInput_Misc ||
                           elemType == kIOHIDElementTypeInput_Button ||
                           elemType == kIOHIDElementTypeInput_Axis ||
                           elemType == kIOHIDElementTypeInput_ScanCodes) {
                            switch(usagePage){
                            case kHIDPage_GenericDesktop:
                                switch(usage){
                                case kHIDUsage_GD_Mouse:
                                    break;
                                case kHIDUsage_GD_Keyboard:
                                    break;
                                case kHIDUsage_GD_Joystick:
                                    break;
                                case kHIDUsage_GD_GamePad:
                                    break;
                                case kHIDUsage_GD_X:
                                    printf("    This element is for X-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Y:
                                    printf("    This element is for Y-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Z:
                                    printf("    This element is for Z-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Rx:
                                    printf("    This element is for Rx-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Ry:
                                    printf("    This element is for Ry-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Rz:
                                    printf("    This element is for Rz-Axis (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Slider:
                                    printf("    This element is for Slider (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    AddAxis(nAxis++,elem,min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Wheel:
                                    printf("    This element is for Wheel (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    break;
                                case kHIDUsage_GD_Hatswitch:
                                    printf("    This element is for Hatswitch (%d->%d) Scaled(%d->%d)\n",min,max,scaledMin,scaledMax);
                                    if(nHat<YsJoyReaderMaxNumHatSwitch){
                                        hatSwitch[nHat].exist=1;
                                        hatSwitch[nHat].elem=elem;
                                        CFRetain(elem);
                                        nHat++;
                                    }
                                    break;
                                }
                                break;
                            case kHIDPage_Button:
                                printf("    This element is for Button %d\n",usage-1);
                                usage--;
                                if(0 <= usage && usage < YsJoyReaderMaxNumButton){
                                    button[usage].exist=1;
                                    button[usage].elem=elem;
                                    CFRetain(elem);
                                }
                                break;
                            }
                        }
                    }
                    CFRelease(elemAry);
                    this->hidDev=hidDev;
                    return 1;
                }
                CFRelease(elemAry);
            }
            return 0;
        }
        
    void Read(void)
        {
            int i;
            IOHIDValueRef valueRef;
            for(i=0; i < YsJoyReaderMaxNumAxis; i++){
                if(axis[i].exist!=0){
                    IOHIDDeviceGetValue(hidDev,axis[i].elem,&valueRef);
                    axis[i].value=IOHIDValueGetIntegerValue(valueRef);
                }
            }
            for(i=0; i<YsJoyReaderMaxNumButton; i++){
                if(button[i].exist!=0){
                    IOHIDDeviceGetValue(hidDev,button[i].elem,&valueRef);
                    button[i].value=IOHIDValueGetIntegerValue(valueRef);
                }
            }
            for(i=0; i<YsJoyReaderMaxNumHatSwitch; i++)	{
                if(hatSwitch[i].exist!=0){
                    IOHIDDeviceGetValue(hidDev,hatSwitch[i].elem,&valueRef);

                    double scaled=IOHIDValueGetScaledValue(valueRef,kIOHIDValueScaleTypePhysical);
                    if(scaled<-0.001 || 359.999<scaled)	{
                        hatSwitch[i].value=0;
                    } else {
                        hatSwitch[i].value=1+(int)((scaled+22.5)/45.0);
                    }
                }
            }
        }
        
    void ReleaseInterface(void)
        {
            if(hidDev!=NULL) {
                // Honestly, I don't know what to do.
                //
                // Should I do
                //   CFRelease(hidDev);
                // ?
                //
                // This hidDev was copied from a copy of IOHIDManager's device list.
                // Who owns it?  Why did I have to make a copy?
                // 
                // The Creare Rule implies that I have the ownership.
                // http://developer.apple.com/mac/library/documentation/CoreFoundation/Conceptual/CFMemoryMgmt/Concepts/Ownership.html#//apple_ref/doc/uid/20001148-SW1
                //
                // Then, I suppose I should release it.  Am I right?
                CFRelease(hidDev);
                hidDev=NULL;
            }
        }            
        
    int WriteCalibInfoFile(FILE *fp) const
        {
            fprintf(fp,"BGNJOY %d\n",joyId);
            for(int i=0; i < YsJoyReaderMaxNumAxis; i++){
                if(0 != axis[i].exist){
                    fprintf(fp,"AXSINF %d %d %d %d\n",i,axis[i].calibCenter,axis[i].calibMin,axis[i].calibMax);
                }
            }
#ifdef YSJOYREADER_USE_HAT_CALIBRATION
            for(i=0; i < YsJoyReaderMaxNumHatSwitch; i++){
                if(0 != hatSwitch[i].exist){
                    fprintf(fp,"HATINF %d %d %d %d %d %d\n",
                            i,
                            hatSwitch[i].valueNeutral,
                            hatSwitch[i].value0Deg,
                            hatSwitch[i].value90Deg,
                            hatSwitch[i].value180Deg,
                            hatSwitch[i].value270Deg);
                }
            }
#endif
            fprintf(fp,"ENDJOY\n");
            return 1;
        }

    int ReadCalibInfoFile(FILE *fp)
        {
            char str[256];
            while(fgets(str,255,fp)!=NULL){
                if(strncmp(str,"AXSINF",6) == 0){
                    int axisId,cen,min,max;
                    sscanf(str,"%*s %d %d %d %d",&axisId,&cen,&min,&max);
                    if(0<=axisId && axisId<YsJoyReaderMaxNumAxis){
                        axis[axisId].calibCenter=cen;
                        axis[axisId].calibMin=min;
                        axis[axisId].calibMax=max;
                    }
                }
#ifdef YSJOYREADER_USE_HAT_CALIBRATION
                else if(strncmp(str,"HATINF",6)==0) {
                    int hatId;
                    int valueNeutral=0,value0Deg=1,value90Deg=3,value180Deg=5,value270Deg=7;
                    sscanf(str,"%*s %d %d %d %d %d %d",&hatId,&valueNeutral,&value0Deg,&value90Deg,&value180Deg,&value270Deg);
                    if(0<=hatId && hatId<YsJoyReaderMaxNumHatSwitch){
                        hatSwitch[hatId].valueNeutral=valueNeutral;
                        hatSwitch[hatId].value0Deg=value0Deg;
                        hatSwitch[hatId].value90Deg=value90Deg;
                        hatSwitch[hatId].value180Deg=value180Deg;
                        hatSwitch[hatId].value270Deg=value270Deg;
                    }
                }
#endif
                else if(strncmp(str,"ENDJOY",6)==0){
                    return 1;
                }
            }
            return 0;
        }
            
protected:
    void AddAxis(int axisId,IOHIDElementRef elem,int min,int max,int scaledMin,int scaledMax)
        {
            if(0<=axisId && axisId<YsJoyReaderMaxNumAxis){
                axis[axisId].exist=1;
                axis[axisId].elem=elem;
                axis[axisId].min=min;
                axis[axisId].max=max;
                axis[axisId].scaledMin=scaledMin;
                axis[axisId].scaledMax=scaledMax;

                axis[axisId].calibCenter=(min+max)/2;
                axis[axisId].calibMin=min;
                axis[axisId].calibMax=max;

                CFRetain(elem);
            }
        }

public:
    static int SetUpJoystick(int &nJoystick, YsJoyReader joystick[], int maxNumJoystick)
        {
            nJoystick=0;
            if(NULL==hidManager){
                hidManager=IOHIDManagerCreate(kCFAllocatorDefault,kIOHIDOptionsTypeNone);
            }
            if(NULL!=hidManager){
                IOHIDManagerSetDeviceMatching(hidManager,NULL);  // Just enumrate all devices
                IOHIDManagerOpen(hidManager,kIOHIDOptionsTypeNone);
                
                CFSetRef copyOfDevices=IOHIDManagerCopyDevices(hidManager);
                if(NULL!=devArray){
                    CFRelease(devArray);
                    devArray=NULL;
                }
                devArray=CFArrayCreateMutable(kCFAllocatorDefault,0,&kCFTypeArrayCallBacks);
                CFSetApplyFunction(copyOfDevices,CFSetCopyCallBack,(void *)devArray);

                CFIndex nDev=CFArrayGetCount(devArray);

                printf("%d devices found\n",(int)nDev);

                CFRelease(copyOfDevices);

                for(int i=0; i < nDev && nJoystick < maxNumJoystick; i++){
                    IOHIDDeviceRef hidDev=(IOHIDDeviceRef)CFArrayGetValueAtIndex(devArray,i);
                    if(joystick[nJoystick].SetUpInterface(nJoystick,hidDev)!=0)	{
                        nJoystick++;
                        // CFRelease(hidDev);  // Doesn't it destroy integrity of devArray?
                    }
                }
            }
            return nJoystick;
        }
};

IOHIDManagerRef YsJoyReader::hidManager = NULL;
CFMutableArrayRef YsJoyReader::devArray = NULL;
    
int YsJoyReaderSetUpJoystick(int &nJoystick, YsJoyReader joystick[], int maxNumJoystick)
{
    return YsJoyReader::SetUpJoystick(nJoystick, joystick, maxNumJoystick);
}
}

extern "C" FILE* YsJoyReaderOpenJoystickCalibrationFileC(const char mode[]);

namespace {
FILE *YsJoyReaderOpenJoystickCalibrationFile(const char mode[])
{
    return YsJoyReaderOpenJoystickCalibrationFileC(mode);
}

int YsJoyReaderSaveJoystickCalibrationInfo(int nJoystick, YsJoyReader joystick[])
{
    FILE *fp;
    fp = YsJoyReaderOpenJoystickCalibrationFile("w");

    if(fp!=NULL){
        for(int i=0; i<nJoystick; i++){
            joystick[i].WriteCalibInfoFile(fp);
        }
        fclose(fp);
        return 1;
    }
    return 0;
}

int YsJoyReaderLoadJoystickCalibrationInfo(int nJoystick,YsJoyReader joystick[])
{
    FILE *fp;
    fp = YsJoyReaderOpenJoystickCalibrationFile("r");

    if(fp!=NULL){
        char str[256];
        while(fgets(str,255,fp)!=NULL){
            if(strncmp(str,"BGNJOY",6)==0)	{
                int joyId;
                sscanf(str,"%*s %d",&joyId);
                if(0<=joyId && joyId<nJoystick){
                    joystick[joyId].ReadCalibInfoFile(fp);
                }
            }
        }
        fclose(fp);
        return 1;
    }
    return 0;
}
}
    
namespace cnoid {

class JoystickImpl
{
public:
    int fd;
    YsJoyReader m_dev[MaxNumJoysticks];
    vector<double> axes;
    vector<bool> buttons;
    string errorMessage;
    Signal<void(int id, bool isPressed)> sigButton;
    Signal<void(int id, double position)> sigAxis;

    JoystickImpl(const char* device);
    bool readCurrentState();
};
}


Joystick::Joystick()
{
    impl = new JoystickImpl("0");
}


Joystick::Joystick(const char* dev)
{
    impl = new JoystickImpl(dev);
}


JoystickImpl::JoystickImpl(const char *dev)
{
    fd = atoi(dev);
    int numJoystick;
  
    YsJoyReaderSetUpJoystick(numJoystick, m_dev, MaxNumJoysticks);
    std::cout << "numJoystick:" << numJoystick << std::endl;
    if(fd >= numJoystick) {
        fd = -1;
        return;
    }
    YsJoyReaderLoadJoystickCalibrationInfo(numJoystick, m_dev);
  
    int nbuttons = 0;
    int naxes = 0;
    
    for(int j=0; j < YsJoyReaderMaxNumAxis; j++){
        if(m_dev[fd].axis[j].exist != 0){
            naxes++;
        }
    }
    for(int j=0; j < YsJoyReaderMaxNumButton; j++){
        if(m_dev[fd].button[j].exist != 0){
            nbuttons++;
        }
    }
    std::cout << "axes:" << naxes << ", buttons:" << nbuttons << std::endl;
    
    axes.resize(naxes, 0.0);
    buttons.resize(nbuttons, false);

    readCurrentState();
}


Joystick::~Joystick()
{
    delete impl;
}


bool Joystick::isReady() const
{
    return (impl->fd >= 0);
}

const char* Joystick::errorMessage() const
{
    return impl->errorMessage.c_str();
}


int Joystick::numAxes() const
{
    return impl->axes.size();
}


int Joystick::numButtons() const
{
    return impl->buttons.size();
}


bool Joystick::readCurrentState()
{
    return impl->readCurrentState();
}


bool JoystickImpl::readCurrentState()
{
    if(fd < 0){
        return false;
    }
    
    m_dev[fd].Read();

    for(int j=0; j < YsJoyReaderMaxNumAxis; j++){
        if(m_dev[fd].axis[j].exist != 0){
            axes[j] = m_dev[fd].axis[j].GetCalibratedValue();
        }
    }
    for(int j=0; j < YsJoyReaderMaxNumButton; j++){
        if(m_dev[fd].button[j].exist != 0){
            buttons[j] = m_dev[fd].button[j].value;
        }
    }
    return false;
}


double Joystick::getPosition(int axis) const
{
    if(axis < impl->axes.size()){
        return impl->axes[axis];
    }
    return 0.0;
}


bool Joystick::getButtonState(int button) const
{
    if(button < impl->buttons.size()){
        return impl->buttons[button];
    }
    return false;
}


SignalProxy<void(int id, bool isPressed)> Joystick::sigButton()
{
    return impl->sigButton;
}


SignalProxy<void(int id, double position)> Joystick::sigAxis()
{
    return impl->sigAxis;
}
