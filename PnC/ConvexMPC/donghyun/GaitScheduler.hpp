/*!
 * @file GaitScheduler.h
 * @brief Logic for fixed-gait timing
 */

#include <iostream>
#include <Eigen/Dense>

// #include "cppTypes.h"
// #include "../../user/MIT_Controller/MIT_UserParameters.h"

/**
 * Enumerated gait types. Preplanned gaits are defined.
 */
enum class GaitType {
  STAND,
  STAND_CYCLE,
  STATIC_WALK,
  AMBLE,
  TROT_WALK,
  TROT,
  TROT_RUN,
  PACE,
  BOUND,
  ROTARY_GALLOP,
  TRAVERSE_GALLOP,
  PRONK,
  THREE_FOOT,
  CUSTOM,
  TRANSITION_TO_STAND
};

/**
 * Timing data for a gait
 */

struct GaitData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GaitData() { zero(); }

  // Zero out all of the data
  void zero();

  // The current GaitType
  GaitType _currentGait;

  // Next GaitType to transition into
  GaitType _nextGait;

  // Gait name string
  std::string gaitName;

  // Gait descriptors
  double periodTimeNominal;      // overall period time to scale
  double initialPhase;           // initial phase to offset
  double switchingPhaseNominal;  // nominal phase to switch contacts
  int overrideable;

  // Enable flag for each foot
  Eigen::Vector4i gaitEnabled;  // enable gait controlled legs

  // Time based descriptors
  Eigen::Vector4d periodTime;           // overall foot scaled gait period time
  Eigen::Vector4d timeStance;           // total stance time
  Eigen::Vector4d timeSwing;            // total swing time
  Eigen::Vector4d timeStanceRemaining;  // stance time remaining
  Eigen::Vector4d timeSwingRemaining;   // swing time remaining

  // Phase based descriptors
  Eigen::Vector4d switchingPhase;  // phase to switch to swing
  Eigen::Vector4d phaseVariable;   // overall gait phase for each foot
  Eigen::Vector4d phaseOffset;     // nominal gait phase offsets
  Eigen::Vector4d phaseScale;      // phase scale relative to variable
  Eigen::Vector4d phaseStance;     // stance subphase
  Eigen::Vector4d phaseSwing;      // swing subphase

  // Scheduled contact states
  Eigen::Vector4i contactStateScheduled;  // contact state of the foot
  Eigen::Vector4i contactStatePrev;       // previous contact state of the foot
  Eigen::Vector4i touchdownScheduled;     // scheduled touchdown event flag
  Eigen::Vector4i liftoffScheduled;       // scheduled touchdown event flag

  // UserParameters moved here
  double gait_type;
  double gait_period_time;
  double gait_switching_phase;
  int gait_override;
  double gait_max_leg_angle;
  double gait_max_stance_time;
  double gait_min_stance_time;

};

/**
 * Utility to process GaitData and schedule foot steps and swings.
 */
class GaitScheduler {
 public:
  // Constructors for the GaitScheduler
  GaitScheduler(double _dt); //MIT_UserParameters* _userParameters,
  ~GaitScheduler(){};

  // Initialize the Gait Scheduler
  void initialize();

  // Iteration step for scheduler logic
  void step();

  // Creates a new gait from predefined library
  void modifyGait();
  void createGait();
  void calcAuxiliaryGaitData();

  // Prints the characteristic info and curret state
  void printGaitInfo();

  // Struct containing all of the gait relevant data
  GaitData gaitData;

  // Natural gait modifiers
  double period_time_natural = 0.5;
  double switching_phase_natural = 0.5;
  double swing_time_natural = 0.25;

 private:
  // The quadruped model
  // Quadruped<T>& _quadruped;
  // MIT_UserParameters* userParameters;

  // Control loop timestep change
  double dt;

  // Phase change at each step
  double dphase;

  // Choose how often to print info, every N iterations
  int printNum = 5;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;
};


