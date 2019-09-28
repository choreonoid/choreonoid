/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BALANCER_PLUGIN_WAIST_BALANCER_H
#define CNOID_BALANCER_PLUGIN_WAIST_BALANCER_H

#include <cnoid/BodyMotion>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/LinkTraverse>
#include <cnoid/CompositeIK>
#include <cnoid/stdx/optional>
#include <vector>

namespace cnoid {

    class PoseProvider;

    class WaistBalancer
    {
      public:
        WaistBalancer();

        void setMessageOutputStream(std::ostream& os) {
            os_ = &os;
        }

        void setBody(const BodyPtr& body);
        const BodyPtr& body() const;
                
        void setWaistLink(Link* waistLink);
        void setNumIterations(int n);
        int numIterations() const;
        void setTimeRange(double lower, double upper);
        void setFullTimeRange();
        void setTimeMargins(double timeToStartBalancer, double preInitialDuration, double postFinalDuration);
        void setGravity(double g);
        void setDynamicsTimeRatio(double r);

        enum BoundaryConditionType {
            KEEP_POSITIONS = 0,
            ZERO_VELOCITY = 1,
            NUM_BOUNDARY_CONDITION_TYPES = 2,
            DEFAULT_BOUNDARY_CONDITION = 0
        };
        static const char* boundaryConditionTypeNameOf(int type);
        static int boundaryConditionTypeOf(const std::string& name);
        void setBoundaryConditionType(int type);

        enum BoundarySmootherType {
            NO_SMOOTHER = 0,
            CUBIC_SMOOTHER = 1,
            QUINTIC_SMOOTHER = 2,
            NUM_BOUNDARY_SMOOTHER_TYPES = 3,
            DEFAULT_BOUNDARY_SMOOTHER = 2
        };
        static const char* boundarySmootherTypeNameOf(int type);
        static int boundarySmootherTypeOf(const std::string& name);
        void setBoundarySmoother(int type, double smoothingTime);

        void enableBoundaryCmAdjustment(bool on, double transitionTime = 1.0);

        enum InitialWaistTrajectoryMode { PLAIN_TRAJECTORY, ORG_TRAJECTORY };
        void setInitialWaistTrajectoryMode(int mode);

        void enableWaistHeightRelaxation(bool on);
            
        bool apply(PoseProvider* provider, BodyMotion& motion, bool putAllLinkPositions = false);

      private:

        std::vector<double> q0;
        Vector3 p0;
        Matrix3 R0;

        int initialWaistTrajectoryMode;
        double timeRangeLower;
        double timeRangeUpper;
        double targetBeginningTime;
        double targetEndingTime;
        double timeToStartBalancer;
        double preInitialDuration;
        double postFinalDuration;
        double dynamicsTimeRatio;

        BodyPtr body_;
        Link* baseLink;
        LinkTraverse fkTraverse;
        PoseProvider* provider;
        bool isCalculatingInitialWaistTrajectory;

        std::vector<stdx::optional<double>> jointPositions;
            
        int numIterations_;
        int beginningFrame;
        int endingFrame;
        int numFilteredFrames;
        int frameToStartBalancer;
        double frameRate;
        double timeStep; // original delta time
        double dt; // delta time (as the dynamics)
        double dt2; // delta time square (as the dyanmics)
        double g;
        double mg;
        double m;
        double inertial_g;
        Vector3 cm; // center of mass
        Vector3 P0; // prev momentum
        Vector3 L0; // prev angular momentum
        Vector3 dP;
        Vector3 dL;
        Vector3 zmp; // calculated ZMP
        Vector3 desiredZmp;
        Vector3 zmpDiff;

        struct Coeff {
            double a;
            double b;
            Vector3 d;
        };
        std::vector<Coeff> coeffSeq;

        std::vector<Vector3> totalCmTranslations;

        Link* waistLink;
        int waistLinkIndex;
            
        int boundaryConditionType;
        int boundarySmootherType_;
        bool doBoundarySmoother;
        double boundarySmoothingTime;
        std::function<void(int begin, int direction)> boundarySmootherFunction;
        int numBoundarySmoothingFrames;
        bool doProcessFinalBoundary;

        bool isBoundaryCmAdjustmentEnabled;
        double boundaryCmAdjustmentTransitionTime;
        double boundaryCmAdjustmentTransitionHalfLength;
        bool doBoundaryCmAdjustment;
        std::vector<Vector3> boundaryCmAdjustmentTranslations;

        // for the waist height relaxation
        bool isWaistHeightRelaxationEnabled;
        bool doStoreOriginalWaistFeetPositionsForWaistHeightRelaxation;
        bool doWaistHeightRelaxation;
        struct WaistFeetPos {
            Vector3 p_Waist;
            Matrix3 R_Waist;
            Vector3 p_Foot[2];
            Matrix3 R_Foot[2];
        };
        std::vector<WaistFeetPos> waistFeetPosSeq;
        std::vector<double> waistDeltaZseq;
        CompositeIK waistFeetIK;
        Link* rightKneePitchJoint;
        Link* leftKneePitchJoint;

        std::ostream* os_;

        std::ostream& os() { return *os_; }

        bool apply2(BodyMotion& motion, bool putAllLinkPositions);
        void calcBoundaryCmAdjustmentTrajectory();
        bool calcBoundaryCmAdjustmentTrajectorySub(int begin, int direction);
        bool calcWaistTranslationWithCmAboveZmp(
            int frame, const Vector3& zmp, Vector3& out_translation);
        void initBodyKinematics(int frame, const Vector3& cmTranslation);
        void updateCmAndZmp(int frame);
        bool updateBodyKinematics1(int frame);
        void updateBodyKinematics2();
        bool calcCmTranslations();
        void initWaistHeightRelaxation();
        void relaxWaistHeightTrajectory();
        void applyCubicBoundarySmoother(int begin, int direction);
        void applyQuinticBoundarySmoother(int begin, int direction);
        bool applyCmTranslations(BodyMotion& motion, bool putAllLinkPositions);

        inline double timeOfFrame(int frame) {
            return std::max(targetBeginningTime, std::min(targetEndingTime, (frame / frameRate)));
        }
    };
}

#endif
