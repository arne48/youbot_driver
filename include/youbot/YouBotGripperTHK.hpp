#ifndef YOUBOTGRIPPERTHK_HPP
#define YOUBOTGRIPPERTHK_HPP

/*License Copyright etc.*/

#include <vector>
#include <sstream>
#include <string>
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/ConfigFile.hpp"
#include "generic/Exceptions.hpp"
#include "youbot/YouBotGripper.hpp"
#include "youbot/YouBotJoint.hpp"
#include "youbot/EthercatMaster.hpp"
#include "youbot/EthercatMasterInterface.hpp"
#include "youbot/EthercatMasterWithThread.hpp"
#include "youbot/EthercatMasterWithoutThread.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>

#include <boost/thread.hpp>

namespace youbot {

/// The number of manipulator joints
#define GRIPPERJOINTS 1
///////////////////////////////////////////////////////////////////////////////
/// It groups the manipulator joints and the gripper together
///////////////////////////////////////////////////////////////////////////////
class YouBotGripperTHK {
  public:
    YouBotGripperTHK(const std::string name, const std::string configFilePath = "../config/");

    virtual ~YouBotGripperTHK();

    ///does the sine commutation of the arm joints
    void doJointCommutation();

    ///calibrate the reference position of the arm joints
    void calibrateGripperTHK(const bool forceCalibration = false);

    void calibrateGripper(const bool forceCalibration = false);

    void softClose(float TargetPosition, float fMaxVelocity, float fAbortCurrent);

    void softCloseThread(float TargetPosition, float fMaxVelocity, float fAbortCurrent);

    ///return a joint form the arm1
    ///@param armJointNumber 1-5 for the arm joints
    YouBotJoint& getArmJoint(const unsigned int armJointNumber);

    YouBotGripper& getArmGripper();

    ///commands positions or angles to all manipulator joints
    ///all positions will be set at the same time
    ///@param JointData the to command positions
    virtual void setJointData(const std::vector<JointAngleSetpoint>& JointData);

    ///gets the position or angle of all manipulator joints which have been calculated from the actual encoder value
    ///These values are all read at the same time from the different joints
    ///@param data returns the angles by reference
    virtual void getJointData(std::vector<JointSensedAngle>& data);

    ///commands velocities to all manipulator joints
    ///all velocities will be set at the same time
    ///@param JointData the to command velocities
    virtual void setJointData(const std::vector<JointVelocitySetpoint>& JointData);

    ///gets the velocities of all manipulator joints which have been calculated from the actual encoder values
    ///These values are all read at the same time from the different joints
    ///@param data returns the velocities by reference
    virtual void getJointData(std::vector<JointSensedVelocity>& data);

    ///commands current to all manipulator joints
    ///all current values will be set at the same time
    ///@param JointData the to command current
    virtual void setJointData(const std::vector<JointCurrentSetpoint>& JointData);

    ///gets the motor currents of all manipulator joints which have been measured by a hal sensor
    ///These values are all read at the same time from the different joints
    ///@param data returns the actual motor currents by reference
    virtual void getJointData(std::vector<JointSensedCurrent>& data);

    ///commands torque to all manipulator joints
    ///all torque values will be set at the same time
    ///@param JointData the to command torque
    virtual void setJointData(const std::vector<JointTorqueSetpoint>& JointData);

    ///gets the joint torque of all manipulator joints which have been calculated from the current
    ///These values are all read at the same time from the different joints
    ///@param data returns the actual joint torque by reference
    virtual void getJointData(std::vector<JointSensedTorque>& data);


  private:
    YouBotGripperTHK(const YouBotGripperTHK & source);

    YouBotGripperTHK & operator=(const YouBotGripperTHK & source);

    ///does the commutation of the arm joints with firmware 2.0
    void commutationFirmware200();

    ///does the commutation of the arm joints with firmware 1.48 and below
    void commutationFirmware148();

    void initializeJoints();

    boost::scoped_ptr<ConfigFile> configfile;

    boost::ptr_vector<YouBotJoint> joints;

    boost::scoped_ptr<YouBotGripper> gripper;

    int controllerType;

    EthercatMasterInterface& ethercatMaster;

    bool useGripper;

    EthercatMasterWithThread* ethercatMasterWithThread;

    int alternativeControllerType;

    std::vector<std::string> supportedFirmwareVersions;

    std::string actualFirmwareVersionAllJoints;

};

} // namespace youbot
#endif // YOUBOTGRIPPERTHK_HPP
