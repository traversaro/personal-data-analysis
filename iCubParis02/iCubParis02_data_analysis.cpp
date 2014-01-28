#include <kdl_codyco/utils.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>
#include <dirl/dataset/DynamicDatasetFile.hpp>
#include <dirl/dynamicRegressorGenerator.hpp>
#include <cstdlib>
#include <iCub/iDynTree/iCubTree.h>

#include <yarp/os/Property.h>

using namespace KDL;
using namespace KDL::CoDyCo;
using namespace dirl;

#define MODEL_DOFS 32
#define DATASET_DOFS 41

bool convertJointsFromDatasetToModel(KDL::JntArray & q_dataset, KDL::JntArray & q_model)
{
    assert(q_dataset.columns() == MODEL_DOFS);
    assert(q_model.columns() == DATASET_DOFS);
    //Joints in dataset are : Torso (3), head(6), left arm (16), right_arm (16)
    //Joints in the mode are : Torso (3), head(3), left arm (7), right arm (7), left_leg (6), right_leg(6)
    
    q_model.SetToZero();
    
    //Torso should be inverted
    q_model(0) = q_dataset(2);
    q_model(1) = q_dataset(1);
    q_model(2) = q_dataset(0);
    
    //Copyng head
    for(int i = 0; i < 3; i++ ) {
        q_model(3+i) = q_dataset(3+i);
    }
    
    //Copyng left arm
    for(int i = 0; i < 7; i++ ) {
        q_model(3+3+i) = q_dataset(3+6+i);
    }
    
    //Copyng right arm 
    for(int i = 0; i < 7; i++ ) {
        q_model(3+3+7+i) = q_dataset(3+6+16+i);
    }
    
   
    
}

int main(int argc, char ** argv)
{
    yarp::os::Property opt;
    
    opt.fromCommand(argc,argv);
   
    if( !opt.check("dataset") ) {
        std::cout << "iCubParis02_data_analysis: " << std::endl;
        std::cout << "usage: ./iCubParis02_data_analysis --dataset dataset.csv" << std::endl;
        return 0;
    }
    
    std::string dataset_file_name(opt.find("dataset").asString());
    
    iCub::iDynTree::iCubTree_version_tag tag;
    //iCubParis02 is a v2 robot
    tag.head_version = 2;
    tag.legs_version = 2;
    
    iCub::iDynTree::iCubTree icub_tree_model(tag);
    
    std::vector<std::string> ft_names;
    ft_names.push_back("l_arm_ft_sensor");
    ft_names.push_back("r_arm_ft_sensor");
    
    std::vector<std::string> fake_names;
    fake_names.push_back("torso");
    fake_names.push_back("imu_frame");
    fake_names.push_back("l_gripper");
    fake_names.push_back("r_gripper");    
    fake_names.push_back("l_sole");
    fake_names.push_back("r_sole");
    fake_names.push_back("l_wrist_1");
    fake_names.push_back("r_wrist_1");
    
    DynamicRegressorGenerator ft_regressor_generator(icub_tree_model.getKDLTree(),
                                                  "root_link",
                                                  ft_names,
                                                  true,
                                                  fake_names,
                                                  icub_tree_model.getKDLUndirectedTree().getSerialization());
    
    DynamicRegressorGenerator torque_regressor_generator(icub_tree_model.getKDLTree(),
                                                  "root_link",
                                                  ft_names,
                                                  true,
                                                  fake_names,
                                                  icub_tree_model.getKDLUndirectedTree().getSerialization());
    
    DynamicRegressorGenerator reverse_torque_regressor_generator(icub_tree_model.getKDLTree(),
                                                  "root_link",
                                                  ft_names,
                                                  true,
                                                  fake_names,
                                                  icub_tree_model.getKDLUndirectedTree().getSerialization());
                                                
    std::vector<std::string> l_arm_subtree, r_arm_subtree;
    
    l_arm_subtree.push_back("l_arm");
    r_arm_subtree.push_back("r_arm");
    
    //Adding three regressors to the generator:
    //    (1) 6 for ft sensors (with offset)
    //    (2) 1 for forward torque regressor
    //    (3) 1 for backward torque regressor
    int ret = ft_regressor_generator.addSubtreeRegressorRows(l_arm_subtree);
    assert(ret == 0);
    
    ret = torque_regressor_generator.addTorqueRegressorRows("l_elbow");
    assert(ret == 0);
    
    reverse_torque_regressor_generator.addTorqueRegressorRows("l_elbow",true,ft_names);
    assert(ret == 0);
    
    
    //Loading dataset
    DynamicDatasetFile test_dataset;
  
    test_dataset.loadFromFile(dataset_file_name);
    
    KDL::JntArray q, dq, ddq;
    q.resize(regressor_generator.getNrOfDOFs());
    q.SetToZero();
    
    dq = q;
    ddq = q;
        
    const double g = 9.8;
    KDL::Twist gravity(KDL::Vector(0.0,0.0,g),KDL::Vector(0.0,0.0,0.0));
    
    Eigen::MatrixXd ft_regressor(ft_regressor_generator.getNrOfOutputs(),ft_regressor_generator.getNrOfParameters());
    Eigen::VectorXd ft_kt(ft_regressor_generator.getNrOfOutputs());
    
    Eigen::MatrixXd torque_regressor(torque_regressor_generator.getNrOfOutputs(),torque_regressor_generator.getNrOfParameters());
    Eigen::VectorXd torque_kt(torque_regressor_generator.getNrOfOutputs());
    
    
    Eigen::MatrixXd reverse_torque_regressor(reverse_torque_regressor_generator.getNrOfOutputs(),reverse_torque_regressor_generator.getNrOfParameters());
    Eigen::VectorXd reverse_torque_kt(reverse_torque_regressor_generator.getNrOfOutputs());
     
    
    const int l_elbow_local_id = 3;
    
    Eigen::MatrixXd base_parameters_subspace; 
    
    Eigen::MatrixXd static_base_parameters_subspace;
    
    regressor_generator.computeNumericalIdentifiableSubspace(base_parameters_subspace,)
    
    int i;
    for(i=0; i < test_dataset.getNrOfSamples(); i++ ) {
        //Joints in dataset are : Torso (3), head(6), left arm (16), right_arm (16)
        //Depending on the dataset we use different parts, here we use only the  right arm, the torso and the head (which however should be still)
        DynamicSample sample;
        bool ret = test_dataset.getSample(i,sample);
        if( !ret ) return -1;
        assert(sample.getNrOfDOFs() == DATASET_DOFS);
       
        convertJointsFromDatasetToModel(sample.getJointPosition(),q);
        //Considering velocity zero for now
        
        ft_regressor_generator.setRobotState(q,dq,ddq,gravity);
        for(int i=0; i < ft_regressor_generator.getNrOfWrenchSensors(); i++ ) {
            ft_regressor_generator.setFTSensorMeasurement(i,sample.getWrenchMeasure(i));
        }
        ft_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex("l_elbow"),sample.getTorqueMeasure(l_elbow_local_id));
        ft_regressor_generator.computeRegressor(ft_regressor,ft_kt);
        
        torque_regressor_generator.setRobotState(q,dq,ddq,gravity);
        for(int i=0; i < torque_regressor_generator.getNrOfWrenchSensors(); i++ ) {
            torque_regressor_generator.setFTSensorMeasurement(i,sample.getWrenchMeasure(i));
        }
        torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex("l_elbow"),sample.getTorqueMeasure(l_elbow_local_id));
        torque_regressor_generator.computeRegressor(torque_regressor,torque_kt);
        
        reverse_torque_regressor_generator.setRobotState(q,dq,ddq,gravity);
        for(int i=0; i < reverse_torque_regressor_generator.getNrOfWrenchSensors(); i++ ) {
            reverse_torque_regressor_generator.setFTSensorMeasurement(i,sample.getWrenchMeasure(i));
        }
        reverse_torque_regressor_generator.setTorqueSensorMeasurement(icub_tree_model.getDOFIndex("l_elbow"),sample.getTorqueMeasure(l_elbow_local_id));
        reverse_torque_regressor_generator.computeRegressor(reverse_torque_regressor,reverse_torque_kt);

        //Compute estimation
        
        
    }
   
    std::cout << "iCubParis02_analysis: got " << i << " samples "  << std::endl;
    
    return 0;
}