#include "BulletWrapper.h"
#include <iostream>
#include <iostream>
using std::ostream;

namespace BulletWrapper {


template<typename T>
ostream& operator<< (ostream& out, const std::vector<T>& v) {
    out << "{";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last)
            out << ", ";
    }
    out << "}";
    return out;
}


//template<typename T>
////template<typename T1>
//ostream& operator<< (ostream& out, const std::map<std::string, T>& v) {
//    out << "{";
//    size_t last = v.size() - 1;
//    for(size_t i = 0; i < v.size(); ++i) {
//        out << v[i].first << " : " << v[i].second;
//        if (i != last)
//            out << "\n ";
//    }
//    out << "}";
//    return out;
//}

template<class key_t, class value_t>
ostream& operator<<(ostream& os, const std::map<key_t, value_t>& m) {
    for (typename std::map<key_t, value_t>::const_iterator it = m.begin();
            it != m.end(); it++) {
        os << it->first << " : " << it->second << "\n";
    }
    return os;
}

BulletWorld::BulletWorld():m_world_(new b3RobotSimulatorClientAPI_NoGUI())
{

    m_world_->connect(eCONNECT_DIRECT);
    m_world_->setTimeStep(0.2);


}

void BulletWorld::setGravity(Eigen::Vector3d gravity){
    m_world_->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));
}

int BulletWorld::loadURDF(std::string file_name, Eigen::Vector3d pos, Eigen::Vector4d orn, bool is_fixed_base, bool use_multi_body)
{
    b3RobotSimulatorLoadUrdfFileArgs args(btVector3(pos[0], pos[1], pos[2]), btQuaternion(orn[0], orn[1], orn[2], orn[3]));
    args.m_forceOverrideFixedBase = is_fixed_base;
    args.m_useMultiBody = use_multi_body;
    m_robot_id =  m_world_->loadURDF(file_name, args);
//    m_world_->stepSimulation();

    m_num_joints = getNumJoints(m_robot_id);

    for (int i=0;i<m_num_joints;i++)
    {
        b3JointInfo jointInfo;
        m_world_->getJointInfo(m_robot_id,i,&jointInfo);
        if (jointInfo.m_jointName[0])
        {
            m_jointNameToId[jointInfo.m_jointName] = jointInfo.m_jointIndex;
//            std::cout << "load:  "<< jointInfo.m_jointName << ": "<<jointInfo.m_jointIndex<< "\n";
        }
    }

//    std::cout << m_jointNameToId << "\n";
//    m_world_->stepSimulation();

    return m_robot_id;
}

int BulletWorld::getNumJoints(int body_unique_id)
{
    return m_world_->getNumJoints(body_unique_id);

}
void BulletWorld::SetDOFValues(const DblVec &dofs){

}

void BulletWorld::SetDOFValues(const std::vector<std::string> joint_names, const DblVec &dofs)
{
    //    b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
    //    controlArgs.m_maxTorqueValue = maxTorque;
    //    controlArgs.m_kd = kd;
    //    controlArgs.m_kp = kp;
    //    controlArgs.m_targetPosition = desiredAngle;
    //    m_world_->setJointMotorControl(m_robot_id, m_jointNameToId[motorName],controlArgs);

    assert(joint_names.size() == dofs.size());
    for(int i = 0; i< dofs.size(); i++){
        //        SetDOFValue(sb, dofs.at(i), 1000, 0.3, 1);
        ResetJointState(joint_names.at(i), dofs.at(i));

    }
}

void BulletWorld::SetDOFValue(const std::string motorId, double desiredAngle, double maxTorque, double kp, double kd){
    b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
    controlArgs.m_maxTorqueValue = maxTorque;
    controlArgs.m_kd = kd;
    controlArgs.m_kp = kp;
    controlArgs.m_targetPosition = desiredAngle;
    m_world_->setJointMotorControl(m_robot_id, m_jointNameToId[motorId],controlArgs);
    //    m_world_->stepSimulation();
}

//void BulletWorld::SetDOFValues(const DblVec &dofs)
//{
////    b3RobotSimulatorJointMotorArrayArgs args();

////    int *jointIndices = new int[dofs.size()];
////    double *targetVelocitiess = new double[dofs.size()];
////    double *targetPositionss = new double[dofs.size()];
////    double *forcess = new double[dofs.size()];
////    double *kpss = new double[dofs.size()];
////    double *kdss = new double[dofs.size()];


//    b3RobotSimulatorJointMotorArrayArgs args(CONTROL_MODE_PD, m_num_joints);

//    args.m_jointIndices = new int[dofs.size()];
//    args.m_targetPositions = new double[dofs.size()];
//    args.m_targetVelocities = new double[dofs.size()];
//    args.m_kps = new double[dofs.size()];
//    args.m_kds = new double[dofs.size()];
//    args.m_forces = new double[dofs.size()];


//    for(int i=0; i< dofs.size(); i++){
//        args.m_jointIndices[i] = i;
//        args.m_targetVelocities[i] = 0;
//        args.m_targetPositions[i] = dofs.at(i);
//        args.m_forces[i] = 1000;
//        args.m_kps[i] = 10;
//        args.m_kds[i] = 0.3;


//    }
////    m_world_->setJointMotorControlArray(m_robot_id, CONTROL_MODE_PD, m_num_joints, jointIndices, targetVelocitiess, targetPositionss,
////                                        forcess, kpss, kdss);

//   bool setState = m_world_->setJointMotorControlArray(m_robot_id, args);

//   std::cout << setState << std::endl;
//}


void BulletWorld::ResetJointState(const std::string jointName, const double state){
    m_world_->resetJointState(m_robot_id, m_jointNameToId[jointName], state);
}
void BulletWorld::GetDOFLimits(DblVec &lower, DblVec &upper) const
{

}

double BulletWorld::GetDOFValue(std::string jointName)
{
    b3JointSensorState state;
    m_world_->getJointState(m_robot_id, m_jointNameToId[jointName], &state);
    return state.m_jointPosition;


}

DblVec BulletWorld::GetDOFValues(std::vector<std::string> joint_names){
//    DblVec jState(joint_names.size());
//    for(int i=0; i< joint_names.size(); i++){

////        std::string sb = "lbr_iiwa_joint_" + std::to_string(i);
//        std::string sb(joint_names.at(i));

////        jState.at(i) = GetDOFValue(joint_names.at(i));
//        jState.at(i) = GetDOFValue(sb);

////        std::cout << sb << " : ";

////        std::cout << "jState: " << joint_names.at(i) << " : " << jState.at(i) << std::endl;
//    }
//    return jState;

    DblVec jState(m_num_joints);
    for(int i=0; i< m_num_joints; i++){

        std::string sb = "lbr_iiwa_joint_" + std::to_string(i);

        jState.at(i) = GetDOFValue(sb);

//        std::cout << "jState: " << sb << " : " << jState.at(i) << std::endl;
    }
    return jState;
}

DblVec BulletWorld::GetDOFValues(){
    DblVec jState(m_num_joints);
    for(int i=0; i< m_num_joints; i++){

        std::string sb = "lbr_iiwa_joint_" + std::to_string(i);

        jState.at(i) = GetDOFValue(sb);

//        std::cout << "jState: " << sb << " : " << jState.at(i) << std::endl;
    }
    return jState;
}

//DblVec BulletWorld::GetDOFValues()
//{
//    DblVec jState(m_num_joints);
//    for(int i=0; i< m_num_joints; i++){

//        std::string sb = "lbr_iiwa_joint_" + std::to_string(i);

//        jState.at(i) = GetDOFValue(sb);

////        std::cout << "jState: " << sb << " : " << jState.at(i) << std::endl;
//    }
//    return jState;

//    //    b3JointStates2 state;

//    //    state.m_actualStateQ.resize(m_num_joints);
//    //    state.m_actualStateQdot.resize(m_num_joints);
//    //    state.m_jointReactionForces.resize(m_num_joints);
//    //    state.m_bodyUniqueId = m_robot_id;
//    //    state.m_numDegreeOfFreedomQ = m_num_joints;
//    //    state.m_numDegreeOfFreedomU = m_num_joints;


//    //    m_world_->getJointStates(m_robot_id, state);

//    //    DblVec joint_state(state.m_actualStateQ.size());

//    //    for(int i=0; i< state.m_actualStateQ.size(); i++){
//    //        std::cout << "q: " << i << " : " << state.m_actualStateQ[i]<< std::endl;
//    //        joint_state.at(i) = state.m_actualStateQ[i];
//    //    }

//}

int BulletWorld::GetDOF() const
{

}

//DblVec BulletWorld::getZeroVec(const int size){
//    for()
//}

DblMatrix BulletWorld::PositionJacobian(std::string link_name, const Eigen::Vector3f &pt){

    DblVec pt1 = {pt[0], pt[1], pt[2]};
    return PositionJacobian(link_name, pt1);

//    DblVec jointPositions = GetDOFValues();
//    DblVec jointVelocities(jointPositions.size(), 0);
//    DblVec jointAccelerations(jointPositions.size(), 0);

//    DblVec st = GetDOFValues();
//    for (int i = 0; i< st.size(); i++){
//        jointVelocities[i] = st.at(i);
//        jointAccelerations[i] = 0;

//    }

////    std::cout << "jointPositions : " << jointPositions << std::endl;
////    std::cout << "jointVelocities : " << jointVelocities << std::endl;
////    std::cout << "jointAccelerations : " << jointAccelerations << std::endl;

//    double* linearJacobian = NULL;
//    double* angularJacobian = NULL;


//    int dofCountOrg = 0;
//    for (int j = 0; j < m_num_joints; j++)
//    {
//        struct b3JointInfo info;
//        m_world_->getJointInfo(m_robot_id, j, &info);
//        switch (info.m_jointType)
//        {
//        case eRevoluteType:
//        {
//            dofCountOrg+=1;
//            break;
//        }
//        case ePrismaticType:
//        {
//            dofCountOrg+=1;
//            break;
//        }
//        case eSphericalType:
//        {
//            std::cerr << "Spherirical joints are not supported in the pybullet binding";
//        }
//        case ePlanarType:
//        {
//            std::cerr << "Planar joints are not supported in the pybullet binding";
//        }
//        default:
//        {
//            //fixed joint has 0-dof and at the moment, we don't deal with planar, spherical etc
//        }
//        }
//    }
//    DblMatrix result(3, dofCountOrg ? dofCountOrg : m_num_joints);
//    if (dofCountOrg && (pt1.size() == 3) && (jointPositions.size() == dofCountOrg) &&
//                (jointVelocities.size() == dofCountOrg) && (jointAccelerations.size() == dofCountOrg))
//        {
//            int byteSizeDofCount = sizeof(double) * dofCountOrg;
//            linearJacobian = (double*)malloc(3 * byteSizeDofCount);
//            angularJacobian = (double*)malloc(3 * byteSizeDofCount);

//            m_world_->getBodyJacobian(m_robot_id, m_jointNameToId[link_name], &pt1[0], &jointPositions[0], &jointVelocities[0],
//                    &jointAccelerations[0], linearJacobian, angularJacobian);



//            for (int r = 0; r < 3; ++r) {

//                for (int c = 0; c < dofCountOrg; ++c) {
//                    int element = r * dofCountOrg + c;
////                    std::cout << linearJacobian[element] << " ";
//                    result(r, c) = linearJacobian[element];
//                }
////                std::cout << "\n";
//            }
//        }

//    return result;
}

DblMatrix BulletWorld::PositionJacobian(std::string link_name, const DblVec &pt1)

{

    DblVec jointPositions = GetDOFValues();
    DblVec jointVelocities(jointPositions.size(), 0);
    DblVec jointAccelerations(jointPositions.size(), 0);

    DblVec st = GetDOFValues();
    for (int i = 0; i< st.size(); i++){
        jointVelocities[i] = st.at(i);
        jointAccelerations[i] = 0;

    }

//    std::cout << "jointPositions : " << jointPositions << std::endl;
//    std::cout << "jointVelocities : " << jointVelocities << std::endl;
//    std::cout << "jointAccelerations : " << jointAccelerations << std::endl;

    double* linearJacobian = NULL;
    double* angularJacobian = NULL;


    int dofCountOrg = 0;
    for (int j = 0; j < m_num_joints; j++)
    {
        struct b3JointInfo info;
        m_world_->getJointInfo(m_robot_id, j, &info);
        switch (info.m_jointType)
        {
        case eRevoluteType:
        {
            dofCountOrg+=1;
            break;
        }
        case ePrismaticType:
        {
            dofCountOrg+=1;
            break;
        }
        case eSphericalType:
        {
            std::cerr << "Spherirical joints are not supported in the pybullet binding";
        }
        case ePlanarType:
        {
            std::cerr << "Planar joints are not supported in the pybullet binding";
        }
        default:
        {
            //fixed joint has 0-dof and at the moment, we don't deal with planar, spherical etc
        }
        }
    }
    DblMatrix result(3, dofCountOrg ? dofCountOrg : m_num_joints);
    if (dofCountOrg && (pt1.size() == 3) && (jointPositions.size() == dofCountOrg) &&
                (jointVelocities.size() == dofCountOrg) && (jointAccelerations.size() == dofCountOrg))
        {
            int byteSizeDofCount = sizeof(double) * dofCountOrg;
            linearJacobian = (double*)malloc(3 * byteSizeDofCount);
            angularJacobian = (double*)malloc(3 * byteSizeDofCount);

            m_world_->getBodyJacobian(m_robot_id, m_jointNameToId[link_name], &pt1[0], &jointPositions[0], &jointVelocities[0],
                    &jointAccelerations[0], linearJacobian, angularJacobian);



            for (int r = 0; r < 3; ++r) {

                for (int c = 0; c < dofCountOrg; ++c) {
                    int element = r * dofCountOrg + c;
//                    std::cout << linearJacobian[element] << " ";
                    result(r, c) = linearJacobian[element];
                }
//                std::cout << "\n";
            }
        }

    return result;

}

DblMatrix BulletWorld::RotationJacobian(std::string link_name)
{

}

DblVec BulletWorld::RandomDOFValues()
{

}
}
