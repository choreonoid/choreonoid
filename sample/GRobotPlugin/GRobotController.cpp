/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GRobotController.h"
#include <boost/asio/write.hpp>
#include <cmath>
#include <iostream>

#ifndef _WIN32
#include <sys/time.h>
#endif

using namespace std;
namespace asio = boost::asio;

namespace {

const bool MEASURE_TIMER_INTERVALS = true;

#ifdef _WIN32
const char* defaultPortDevice = "COM0";
#else
const char* defaultPortDevice = "/dev/ttyUSB0";
#endif

inline double degree(double rad) { return (180.0 * rad / M_PI); }
    
const int NJOINTS = 20;

const char jointIdToMotorIdMap[] = {
    8 /* R_HIP_Y */, 10 /* R_HIP_R */,  9 /* R_HIP_P */, 11 /* R_KNEE_P */, 12 /* R_ANKLE_P */, 13 /* R_ANKLE_R */,
    14 /* L_HIP_Y */, 16 /* L_HIP_R */, 15 /* L_HIP_P */, 17 /* L_KNEE_P */, 18 /* L_ANKLE_P */, 19 /* L_ANKLE_R */,
    0 /* CHEST_P */,  1 /* NECK_Y */,
    2 /* R_SHOULDER_P */, 3 /* R_SHOULDER_R */, 4 /* R_ELBOW_P */,
    5 /* R_SHOULDER_P */, 6 /* R_SHOULDER_R */, 7 /* R_ELBOW_P */ };

const unsigned char PoseCommand[] = {
    0x53 ,0x6c ,0xfa,0xaf ,0x00 ,0x00 ,0x1e ,0x05,
    0x14,
    0x01 ,0x00 ,0x00 ,0x64 ,0x00,
    0x02 ,0x00 ,0x00 ,0x64 ,0x00,
    0x03 ,0x00 ,0x00 ,0x64 ,0x00,
    0x04 ,0x00 ,0x00 ,0x64 ,0x00,
    0x05 ,0x00 ,0x00 ,0x64 ,0x00,
    0x06 ,0x00 ,0x00 ,0x64 ,0x00,
    0x07 ,0x00 ,0x00 ,0x64 ,0x00,
    0x08 ,0x00 ,0x00 ,0x64 ,0x00,
    0x09 ,0x00 ,0x00 ,0x64 ,0x00,
    0x0a ,0x00 ,0x00 ,0x64 ,0x00,
    0x0b ,0x00 ,0x00 ,0x64 ,0x00,
    0x0c ,0x00 ,0x00 ,0x64 ,0x00,
    0x0d ,0x00 ,0x00 ,0x64 ,0x00,
    0x0e ,0x00 ,0x00 ,0x64 ,0x00,
    0x0f ,0x00 ,0x00 ,0x64 ,0x00,
    0x10 ,0x00 ,0x00 ,0x64 ,0x00,
    0x11 ,0x00 ,0x00 ,0x64 ,0x00,
    0x12 ,0x00 ,0x00 ,0x64 ,0x00,
    0x13 ,0x00 ,0x00 ,0x64 ,0x00,
    0x14 ,0x00 ,0x00 ,0x64 ,0x00,
    0x1b
};

}


GRobotController::GRobotController()
    : portDevice_(defaultPortDevice),
      port(io),
      poseCommand(sizeof(PoseCommand)),
      tmpJointAngles(NJOINTS, 0.0),
      jointAngles(NJOINTS, 0.0)
{
    init();
}


GRobotController::GRobotController(const GRobotController& org)
    : portDevice_(org.portDevice_),
      port(io),
      poseCommand(sizeof(PoseCommand)),
      tmpJointAngles(NJOINTS, 0.0),
      jointAngles(NJOINTS, 0.0)
{
    init();
}    


void GRobotController::init()
{
    transitionTime = 0.1;
    memcpy(&poseCommand[0], PoseCommand, sizeof(PoseCommand));
    mode = ON_DEMAND_POSE_SENDING;

#ifndef _WIN32
    sem_init(&semTimer, 0, 0);
#endif
}


GRobotController::~GRobotController()
{
    if(poseSendingThread.joinable()){
        poseSendingMutex.lock();
        mode = EXIT_POSE_SENDING;
        poseSendingMutex.unlock();
        poseSendingCondition.notify_all();
        poseSendingThread.join();
    }
    
    closeSerialPort();

#ifndef _WIN32
    sem_destroy(&semTimer);
#endif
}


int GRobotController::numJoints() const
{
    return NJOINTS;
}
    

const std::string& GRobotController::portDevice() const
{
    return portDevice_;
}


void GRobotController::setPortDevice(const std::string& device)
{
    portDevice_ = device;
}


bool GRobotController::openSerialPort()
{
    if(!port.is_open()){
        boost::system::error_code error;
        port.open(portDevice_, error);
        if(!error){
            port.set_option(asio::serial_port_base::baud_rate(115200));
            port.set_option(asio::serial_port_base::character_size(8));
            port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
            port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
            port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
            return true;
        }
    }
    return port.is_open();
}


void GRobotController::closeSerialPort()
{
    if(port.is_open()){
        boost::system::error_code error;
        port.close(error);
    }
}


bool GRobotController::sendData(unsigned char* data, int len)
{
    boost::system::error_code error;
    asio::write(port, asio::buffer(data, len), boost::asio::transfer_all(), error);
    if(error){
        closeSerialPort();
    }
    return !error;
}


bool GRobotController::receiveData(char* buf, int len)
{
    boost::system::error_code error;
    int n = 0;
    for(int i=0; i < 100; ++i){
        int c = port.read_some(asio::buffer(buf, len), error);
        if(error){
            break;
        }
        n += c;
        if(n == len){
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return (n == len);
}


bool GRobotController::checkConnection()
{
    static unsigned char command[] = { 0x41, 0x00 };
    char buf;
    
    if(sendData(command, 2)){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(receiveData(&buf, 1) && buf == 0x07){
            return true;
        }
    }

    return false;
}


bool GRobotController::makeConnection()
{
    if(port.is_open()){
        return true;
    }
    if(openSerialPort() && checkConnection()){
        return true;
    }
    closeSerialPort();
    return false;
}
        

char GRobotController::calcSum(unsigned char* data, int len)
{
    char sum = 0;
    for(int i=0; i < len; ++i){
        sum ^= data[i];
    }
    return sum;
}


void GRobotController::switchServo(int jointId, bool on)
{
    unsigned char command[] = { 0x53, 0x09, 0xfa, 0xaf, 0x01, 0x00, 0x24, 0x01, 0x01, 0x01, 0x26 };

    command[4]  = jointIdToMotorIdMap[jointId];
    command[9]  = on ? 1 : 0;
    command[10] = calcSum(&command[4], 6);
    
    sendData(command, 11);
}


void GRobotController::setJointAngleCommand(int jointId, double q, double ttime)
{
    int motorId = jointIdToMotorIdMap[jointId];
    unsigned char* command = &poseCommand[motorId * 5 + 10];

    short qcom = static_cast<short>(degree(q) * 10.0);
    command[0] = qcom & 0xff;
    command[1] = (qcom >> 8) & 0xff;

    unsigned short t = static_cast<unsigned short>(ttime * 100.0);
    command[2] = t & 0xff;
    command[3] = (t >> 8) & 0xff;
}


#if _WIN32

void GRobotController::initializeTimer()
{
 
    timerQueue = CreateTimerQueue();
    isTimerAvailable = (timerQueue != NULL);
    isTimerActive = false;
}


bool GRobotController::startTimer(double interval)
{
    if(MEASURE_TIMER_INTERVALS){
        QueryPerformanceFrequency(&pcfreq);
        QueryPerformanceCounter(&pc0);
    }

    if(isTimerAvailable){
        int msecInterval = static_cast<int>(floor(interval * 1000.0 + 0.5));
        if(CreateTimerQueueTimer(&timerHandle, timerQueue, (WAITORTIMERCALLBACK)timerCallback, (PVOID)this,
                                 msecInterval, msecInterval, WT_EXECUTEINIOTHREAD)){
            isTimerActive = true;
        }
    }
    
    return isTimerActive;
}

    
void GRobotController::stopTimer()
{
    if(isTimerActive){
        if(DeleteTimerQueueTimer(timerQueue, timerHandle, (HANDLE)-1)){
            isTimerActive = false;
        }
    }
}


void GRobotController::finalizeTimer()
{
    if(isTimerAvailable){
        DeleteTimerQueueEx(timerQueue, (HANDLE)-1);
        timerQueue = NULL;
    }
    isTimerActive = false;
    isTimerAvailable = false;
}


VOID WINAPI GRobotController::timerCallback(PVOID lpParameter, BOOL TimerOrWaitFired)
{
    reinterpret_cast<GRobotController*>(lpParameter)->onMotionFrameSendingRequest();
}


#else


void GRobotController::initializeTimer()
{
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sa.sa_sigaction = timerHandler;
    sigemptyset(&sa.sa_mask);
    if(sigaction(SIGRTMIN, &sa, NULL) == -1){
        return;
    }

    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = this;

    sev.sigev_notify_attributes = 0;

    isTimerAvailable = (timer_create(CLOCK_MONOTONIC, &sev, &timerID) >= 0);
    isTimerActive = false;
}


bool GRobotController::startTimer(double interval)
{
    if(MEASURE_TIMER_INTERVALS){
        gettimeofday(&tv0, 0);
    }

    if(isTimerAvailable){
        struct itimerspec tspec;
        tspec.it_interval.tv_sec = 0;
        tspec.it_interval.tv_nsec = floor(interval * 1.0e9 + 0.5);
        tspec.it_value.tv_sec = 0;
        tspec.it_value.tv_nsec = floor(interval * 1.0e9 + 0.5);
        
        isTimerActive = (timer_settime(timerID, 0, &tspec, 0) >= 0);
    }

    return isTimerActive;
}

    
void GRobotController::stopTimer()
{
    if(isTimerAvailable){
        struct itimerspec tspec;
        tspec.it_interval.tv_sec = 0;
        tspec.it_interval.tv_nsec = 0;
        tspec.it_value.tv_sec = 0;
        tspec.it_value.tv_nsec = 0;
        timer_settime(timerID, 0, &tspec, 0);
        isTimerActive = false;
    }
}


void GRobotController::finalizeTimer()
{
    timer_delete(timerID);
    isTimerAvailable = false;
}


void GRobotController::timerHandler(int sig, siginfo_t* si, void* uc)
{
    GRobotController* self = static_cast<GRobotController*>(si->si_value.sival_ptr);
    if(self){
        sem_post(&self->semTimer); // wake up the continuos pose sending loop
    }
}


#endif


void GRobotController::poseSendingLoop()
{
    initializeTimer();
        
    while(true){
        poseSendingMutex.lock();
        int m = mode;
        poseSendingMutex.unlock();

        if(m == ON_DEMAND_POSE_SENDING){
            doOnDemandPoseSending();
        } else if(m == CONTINUOUS_POSE_SENDING){
            doContinuousPoseSending();
        } else {
            break;
        }
    }

    finalizeTimer();
}


void GRobotController::doOnDemandPoseSending()
{
    while(true){
        {
            std::unique_lock<std::mutex> lock(poseSendingMutex);
            poseSendingCondition.wait(lock);

            if(mode != ON_DEMAND_POSE_SENDING){
                break;
            }

            for(int i=0; i < NJOINTS; ++i){
                setJointAngleCommand(i, jointAngles[i], transitionTime);
            }
        }

        poseCommand[109] = calcSum(&poseCommand[4], 105);
        sendData(&poseCommand[0], 110);
    }
}


#ifdef _WIN32
void GRobotController::doContinuousPoseSending()
{
    if(isTimerActive){
        stopTimer();
    }
    poseSendingMutex.lock();
    double ts = motions[currentMotionId].timeStep;
    poseSendingMutex.unlock();

    if(!startTimer(ts)){
        poseSendingMutex.lock();
        mode = ON_DEMAND_POSE_SENDING;
        poseSendingMutex.unlock();
        return;
    }

    while(true){
        {
            unique_lock<mutex> lock(poseSendingMutex);
            poseSendingCondition.wait(lock);
            if(mode != CONTINUOUS_POSE_SENDING){
                break;
            }
        }
    }

    stopTimer();
}


bool GRobotController::onMotionFrameSendingRequest()
{
#ifdef _WIN32
    LARGE_INTEGER pc1;
    if(MEASURE_TIMER_INTERVALS){
        QueryPerformanceCounter(&pc1);
    }
#else
    struct timeval tv1;
    if(MEASURE_TIMER_INTERVALS){
        gettimeofday(&tv1, 0);
    }
#endif
    
    bool doContinue = true;
    
    poseSendingMutex.lock();

    MotionInfo& motion = motions[currentMotionId];
        
    if(currentFrame >= motion.numFrames){
        doContinue = false;
        mode = ON_DEMAND_POSE_SENDING;
        poseSendingMutex.unlock();
        poseSendingCondition.notify_all();

    } else {
        double* angles = &motion.angles[currentFrame * NJOINTS];
        for(int i=0; i < NJOINTS; ++i){
            setJointAngleCommand(i, angles[i], motion.timeStep);
        }
        currentFrame++;

        poseSendingMutex.unlock();

        poseCommand[109] = calcSum(&poseCommand[4], 105);
        sendData(&poseCommand[0], 110);
    }

    if(MEASURE_TIMER_INTERVALS){
#ifdef _WIN32
        cout << "time: " << (double)(pc1.QuadPart - pc0.QuadPart) / pcfreq.QuadPart << endl;
#else
        cout << "time: " << ((tv1.tv_sec - tv0.tv_sec) + (tv1.tv_usec - tv0.tv_usec) / 1000000.0) << endl;
#endif
    }
    
    return doContinue;
}

#else

void GRobotController::doContinuousPoseSending()
{
    if(isTimerActive){
        stopTimer();
    }
    poseSendingMutex.lock();
    double ts = motions[currentMotionId].timeStep;
    poseSendingMutex.unlock();

    if(!startTimer(ts)){
        poseSendingMutex.lock();
        mode = ON_DEMAND_POSE_SENDING;
        poseSendingMutex.unlock();
        return;
    }

    while(true){

        sem_wait(&semTimer);

        if(mode != CONTINUOUS_POSE_SENDING){
            break;
        }

        struct timeval tv1;
        if(MEASURE_TIMER_INTERVALS){
            gettimeofday(&tv1, 0);
        }
    
        poseSendingMutex.lock();

        MotionInfo& motion = motions[currentMotionId];
        
        if(currentFrame >= motion.numFrames){
            mode = ON_DEMAND_POSE_SENDING;
            poseSendingMutex.unlock();
            break;

        } else {
            double* angles = &motion.angles[currentFrame * NJOINTS];
            for(int i=0; i < NJOINTS; ++i){
                setJointAngleCommand(i, angles[i], motion.timeStep);
            }
            currentFrame++;

            poseSendingMutex.unlock();

            poseCommand[109] = calcSum(&poseCommand[4], 105);
            sendData(&poseCommand[0], 110);
        }

        if(MEASURE_TIMER_INTERVALS){
            cout << "time: " << ((tv1.tv_sec - tv0.tv_sec) + (tv1.tv_usec - tv0.tv_usec) / 1000000.0) << endl;
        }
    }

    stopTimer();

    while(sem_trywait(&semTimer) == 0);
}

#endif


void GRobotController::requestToSendPose(double transitionTime)
{
    if(makeConnection()){

        poseSendingMutex.lock();

        if(!poseSendingThread.joinable()){
            poseSendingThread = std::thread(std::bind(&GRobotController::poseSendingLoop, this));
        }

        if(mode == ON_DEMAND_POSE_SENDING){
            jointAngles = tmpJointAngles;
            this->transitionTime = transitionTime;
            poseSendingMutex.unlock();
            poseSendingCondition.notify_all();
        } else {
            poseSendingMutex.unlock();
        }
    }
}


void GRobotController::switchServos(bool on)
{
    poseSendingMutex.lock();
    if(makeConnection()){
        for(int i=0; i < NJOINTS; ++i){
            switchServo(i, on);
        }
    }
    poseSendingMutex.unlock();
}


bool GRobotController::setMotion(double* angles, int numFrames, double timeStep, int id)
{
    poseSendingMutex.lock();
    if(mode == CONTINUOUS_POSE_SENDING){
        poseSendingMutex.unlock();
        return false;
    }
    
    if(id >= static_cast<int>(motions.size())){
        motions.resize(id + 1);
    }
    MotionInfo& motion = motions[id];
    if(motion.angles){
        delete[] motion.angles;
    }
    motion.angles = new double[NJOINTS * numFrames];
    std::copy(angles, angles + NJOINTS * numFrames, motion.angles);
    motion.numFrames = numFrames;
    motion.timeStep = timeStep;

    poseSendingMutex.unlock();
    return true;
}


bool GRobotController::startMotion(double time, int id)
{
    bool ready = false;

    poseSendingMutex.lock();

    if(!poseSendingThread.joinable()){
        poseSendingThread = std::thread(std::bind(&GRobotController::poseSendingLoop, this));
    }
    
    if(mode == CONTINUOUS_POSE_SENDING){
        poseSendingMutex.unlock();
        return false;
    }
    
    currentMotionId = id;
    MotionInfo& info = motions[id];
    currentFrame = static_cast<int>(time / info.timeStep);
    mode = CONTINUOUS_POSE_SENDING;

    poseSendingMutex.unlock();
    poseSendingCondition.notify_all();

    return true;
}


bool GRobotController::isPlayingMotion()
{
    bool isPlaying = false;
    poseSendingMutex.lock();
    isPlaying = (mode == CONTINUOUS_POSE_SENDING);
    poseSendingMutex.unlock();
    return isPlaying;
}
    

void GRobotController::stopMotion()
{
    poseSendingMutex.lock();
    mode = ON_DEMAND_POSE_SENDING;
    poseSendingMutex.unlock();
    poseSendingCondition.notify_all();
}
