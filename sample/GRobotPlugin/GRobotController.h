/*!
  @file
  @author Shin'ichiro Nakaoka
*/
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <boost/asio/serial_port.hpp>
#if (BOOST_VERSION < 106600)
namespace boost { namespace asio {
typedef io_service io_context;
} }
#endif

#ifndef _WIN32
#include <signal.h>
#include <semaphore.h>
#endif

class GRobotController
{
public:
    GRobotController();
    GRobotController(const GRobotController& org);
    virtual ~GRobotController();

    int numJoints() const;

    void setPortDevice(const std::string& device);
    const std::string& portDevice() const;
    void switchServos(bool on);

    void setJointAngle(int jointId, double q) {
        if(jointId < static_cast<int>(tmpJointAngles.size())){
            tmpJointAngles[jointId] = q;
        }
    }

    void requestToSendPose(double transitionTime);

    bool setMotion(double* angles, int numFrames, double timeStep, int id = 0);
    bool startMotion(double time = 0.0, int id = 0);
    bool isPlayingMotion();
    void stopMotion();

private:
    std::string portDevice_;
    boost::asio::io_context io;
    boost::asio::serial_port port;
    
    std::vector<unsigned char> poseCommand;
    double transitionTime;
    std::vector<double> tmpJointAngles;
    std::vector<double> jointAngles;

    std::thread poseSendingThread;
    std::mutex poseSendingMutex;
    std::condition_variable poseSendingCondition;
    enum { ON_DEMAND_POSE_SENDING, CONTINUOUS_POSE_SENDING, EXIT_POSE_SENDING } mode;
    
    struct MotionInfo {
        MotionInfo() {
            angles = 0;
            numFrames = 0;
        }
        ~MotionInfo(){
            if(angles){
                delete[] angles;
            }
        }
        double* angles;
        double timeStep;
        int numFrames;
    };
    
    std::vector<MotionInfo> motions;
    int currentMotionId;
    int currentFrame;
    double timeStep;

    bool isTimerAvailable;
    bool isTimerActive;
#ifdef _WIN32
    HANDLE timerHandle;
    HANDLE timerQueue; 
    LARGE_INTEGER pc0, pc1, pcfreq;
#else
    sem_t semTimer;
    timer_t timerID;
    struct timeval tv0;
#endif

    void init();
    bool openSerialPort();
    void closeSerialPort();
    bool sendData(unsigned char* data, int len);
    bool receiveData(char* buf, int len);
    bool checkConnection();
    bool makeConnection();
    char calcSum(unsigned char* data, int len);
    void switchServo(int jointId, bool on);
    void setJointAngleCommand(int jointId, double q, double ttime);

    void initializeTimer();
    bool startTimer(double interval);
    void stopTimer();
    void finalizeTimer();
    
#if _WIN32
    static VOID CALLBACK timerCallback(PVOID lpParameter, BOOL TimerOrWaitFired);
#else
    static void timerHandler(int sig, siginfo_t* si, void* uc);
#endif
    
    void poseSendingLoop();
    void doOnDemandPoseSending();
    void doContinuousPoseSending();
    bool onMotionFrameSendingRequest();
};        
