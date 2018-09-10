#ifndef BULLETWRAPPER_H
#define BULLETWRAPPER_H
//#include "btBulletDynamicsCommon.h"
#include "bullet/b3RobotSimulatorClientAPI_NoGUI.h"
#include <memory>
#include "Eigen/Core"
#include <map>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

typedef std::vector<double> DblVec;
typedef std::vector<int> IntVec;

namespace BulletWrapper {

class Configuration {
public:

  virtual void SetDOFValues(const DblVec& dofs) = 0;
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const = 0;
  virtual DblVec GetDOFValues() = 0;
  virtual int GetDOF() const = 0;
  virtual DblMatrix PositionJacobian(std::string link_name, const Eigen::Vector3f& pt) = 0;
  virtual DblMatrix RotationJacobian(std::string link_name) = 0;
//  virtual bool DoesAffect(const std::string& link) = 0;
//  virtual vector<OpenRAVE::KinBodyPtr> GetBodies() = 0;
//  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() = 0;
//  virtual void GetAffectedLinks(std::vector<std::string>& links, bool only_with_geom, std::vector<int>& link_inds) = 0;
  virtual DblVec RandomDOFValues() = 0;
  virtual ~Configuration() {}

//    struct Saver {
//    virtual ~Saver(){}
//  };
//  typedef boost::shared_ptr<Saver> SaverPtr;
//  struct GenericSaver : public Saver {
//    DblVec dofvals;
//    Configuration* parent;
//    GenericSaver(Configuration* _parent) : dofvals(_parent->GetDOFValues()), parent(_parent) {}
//    ~GenericSaver() {
//      parent->SetDOFValues(dofvals);
//    }
//  }; // inefficient

//  virtual SaverPtr Save() {
//    return SaverPtr(new GenericSaver(this));
//  }


};

class BulletWorld : public Configuration
{
    std::shared_ptr<b3RobotSimulatorClientAPI_NoGUI> m_world_;
    int m_robot_id, m_num_joints;
    std::map<std::string, int> m_jointNameToId;

public:
    BulletWorld();
    void setGravity(Eigen::Vector3d gravity);
    int loadURDF(std::string file_name, Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0), Eigen::Vector4d orn = Eigen::Vector4d(0, 0, 0, 1), bool is_fixed_base = false, bool use_multi_body = true);
    int getNumJoints(int body_unique_id);
    bool jointStates();

    // Configuration interface
    void SetDOFValues(const DblVec &dofs);

    void GetDOFLimits(DblVec &lower, DblVec &upper) const;
    DblVec GetDOFValues();
    int GetDOF() const;
    DblMatrix PositionJacobian(std::string link_name, const Eigen::Vector3f &pt = Eigen::Vector3f(0, 0, 0));
    DblMatrix RotationJacobian(std::string link_name) ;
    DblVec RandomDOFValues();
    void SetDOFValue(const std::string motorId, double desiredAngle, double maxTorque, double kp, double kd);
    double GetDOFValue(std::string jointName);
    void ResetJointState(const std::string jointName, const double state);

    DblMatrix PositionJacobian(std::string link_name, const DblVec &pt);

    void SetDOFValues(const std::vector<std::string> joint_names, const DblVec &dofs);

    DblVec GetDOFValues(std::vector<std::string> joint_names);
};
}

#endif // BULLETWRAPPER_H
