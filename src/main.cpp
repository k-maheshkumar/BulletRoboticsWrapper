#include <iostream>
#include "BulletWrapper.h"
#include "Eigen/Core"
#include <iostream>
using std::ostream;

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

int main(int argc, char** argv)
{

    BulletWrapper::BulletWorld world;
    world.setGravity( Eigen::Vector3d(0 , 0, -10));
    int robot = world.loadURDF("../data/kuka_iiwa/model.urdf");

    int num_joints = world.getNumJoints(robot);

    std::cout << "No of Joints : " << num_joints << std::endl;

    double ab = 0.5;
    std::vector<double> robot_state = {ab, ab, ab, ab, ab, ab, ab};

    std::vector<std::string> joint_names(robot_state.size());

    for(int i = 0; i< robot_state.size(); i++){
        joint_names.at(i) = "lbr_iiwa_joint_" + std::to_string(i+1);
    }
    std::cout<<"joint names: "<< joint_names << std::endl;

    world.SetDOFValues(joint_names, robot_state);

    std::cout << "robot state: " << world.GetDOFValues() << std::endl;

    DblVec pt ={0.5, 0.5, 0.5};


    DblMatrix jac = world.PositionJacobian("lbr_iiwa_joint_7", pt);

    std::cout << jac << std::endl;

    std::cout << std::endl;

    Eigen::Vector3f pt1 = Eigen::Vector3f(0.5, 0.5, 0.5);
    DblMatrix jac1 = world.PositionJacobian("lbr_iiwa_joint_7", pt1);
    std::cout << jac1 << std::endl;


    return 0;


}
