#include <or_trac_ik/or_trac_ik.hpp>
#include <limits>       // std::numeric_limits
#include <iostream>
#include <ros/ros.h>

//#include <kdl/frames_io.hpp>
//#include <kdl/kinfam_io.hpp>


typedef OpenRAVE::RobotBase::RobotStateSaver RobotStateSaver;

namespace or_helper
{
    bool HasFlag(int key, OpenRAVE::IkFilterOptions options)
    {
        return (key & (static_cast<int>(options))) != 0;
    }

    bool IsSupported(int key)
    {
        return key == 0x0 ||
               key == OpenRAVE::IKFO_CheckEnvCollisions ||
               key == OpenRAVE::IKFO_IgnoreSelfCollisions ||
               key == (OpenRAVE::IKFO_CheckEnvCollisions | OpenRAVE::IKFO_IgnoreSelfCollisions);
    }
}


//initialize the kdl chain from openrave manip
void TracIK::InitKDLChain()
{
    _kdl_chain = KDL::Chain();

    //for each dof, add a joint to the kdl chain
    for (int i = 0; i < _numdofs; i++)
    {
        OpenRAVE::KinBody::JointPtr p_joint = _pRobot->GetJointFromDOFIndex(_indices[i]);
        //ROS_INFO_STREAM("segment from joint: " << getSegmentTransformFromJoint(p_joint));
        //_kdl_chain.addSegment(KDL::Segment(toKDLJoint(p_joint)));//, getSegmentTransformFromJoint(p_joint)));
        _kdl_chain.addSegment(KDL::Segment(toKDLJoint(p_joint), getSegmentTransformFromJoint(p_joint)));
    }

//    //TEST CHAIN
//    // PRINT TO MAKE SURE WE GET THE SAME AS READING URDF
//    std::vector<KDL::Segment> chain_segs = _kdl_chain.segments;
//    for(unsigned int i = 0; i < chain_segs.size(); ++i) {
//      ROS_INFO_STREAM("kdl joint origin: " << chain_segs[i].getJoint().JointOrigin() << "  axis: " << chain_segs[i].getJoint().JointAxis());
//      ROS_INFO_STREAM("kdl segment frame\n" << chain_segs[i].getFrameToTip());
//    }
//
//
//    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(_kdl_chain);
// 
//    // Create joint array
//    unsigned int nj = _kdl_chain.getNrOfJoints();
//    KDL::JntArray jointpositions = KDL::JntArray(nj);
// 
//    // Assign some values to the joint positions
//    // these are based on ada's home config
//    jointpositions(0) = -1.65549603;
//    jointpositions(1) = -1.48096311;
//    jointpositions(2) = 0.19731201;
//    jointpositions(3) = -1.10550746;
//    jointpositions(4) = 1.67789602;
//    jointpositions(5) = 3.39982207;
// 
//    // Create the frame that will contain the results
//    KDL::Frame cartpos;    
// 
//    // Calculate forward position kinematics
//    bool kinematics_status;
//    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
//    if(kinematics_status>=0){
//        ROS_INFO_STREAM("cartesian pose:\n" << cartpos);
//    }else{
//        ROS_INFO_STREAM("Error: could not calculate forward kinematics");
//    } 

}


KDL::Vector toKDLVec3(const OpenRAVE::Vector& vec)
{
    return KDL::Vector(vec.x, vec.y, vec.z);
}


KDL::Rotation toKDLQuat(const OpenRAVE::Vector& vec)
{
    return KDL::Rotation::Quaternion(vec[1], vec[2], vec[3], vec[0]);
}

KDL::Frame toKDLFrame(const OpenRAVE::Transform& transform)
{
    return KDL::Frame(toKDLQuat(transform.rot), toKDLVec3(transform.trans));
}


KDL::Joint toKDLJoint(const OpenRAVE::KinBody::JointPtr p_joint)
{
    KDL::Joint::JointType kdl_type;

    //figure out joint type
    OpenRAVE::KinBody::JointType or_j_type = p_joint->GetType();
    //TODO: handle other joint types?
    if (or_j_type == OpenRAVE::KinBody::JointRevolute)
    {
        kdl_type = KDL::Joint::RotAxis;
    } else if (or_j_type == OpenRAVE::KinBody::JointSlider || or_j_type == OpenRAVE::KinBody::JointPrismatic) {
        kdl_type = KDL::Joint::TransAxis;
    } else {
        ROS_FATAL_STREAM("Error: Unknown conversion to kdl for joint type " << or_j_type);
    }

    //get origin and axis from parent joint
    KDL::Vector origin = toKDLVec3(p_joint->GetInfo()._vanchor);
    KDL::Vector axis = toKDLVec3(p_joint->GetInfo()._vaxes[0]);  //TODO can we always just take 0? urdf_loader only sets that one

    return KDL::Joint(p_joint->GetName(), origin, axis, kdl_type);
}

KDL::Frame getSegmentTransformFromJoint(const OpenRAVE::KinBody::JointPtr p_joint)
{
    return toKDLFrame(p_joint->GetInternalHierarchyLeftTransform() * p_joint->GetInternalHierarchyRightTransform());
}


bool TracIK::Init(OpenRAVE::RobotBase::ManipulatorConstPtr pmanip)
{
    _pmanip = boost::const_pointer_cast<OpenRAVE::RobotBase::Manipulator>(pmanip);
    _pmanip_base = _pmanip->GetBase();
    _pRobot = _pmanip->GetRobot();
    _indices = _pmanip->GetArmIndices();
    _numdofs = _pmanip->GetArmDOF();

    InitKDLChain();

    KDL::JntArray l_limits(_numdofs), u_limits(_numdofs);
    for (int i = 0; i < _numdofs; i++)
    {
        //if(limits[ii].first<-2*M_PI){
        OpenRAVE::KinBody::JointPtr p_joint = _pRobot->GetJointFromDOFIndex(_indices[i]);
        if (p_joint->IsCircular(0))
        {
            l_limits(i)=std::numeric_limits<float>::lowest();
            u_limits(i)=std::numeric_limits<float>::max();
        } else {
            l_limits(i) = p_joint->GetLimit().first;
            u_limits(i) = p_joint->GetLimit().second;
        }
    }

    _tracik_solver_ = new TRAC_IK::TRAC_IK(_kdl_chain, l_limits, u_limits);//, maxtime, eps, type);

    return true;
}


TracIK::TracIK(OpenRAVE::EnvironmentBasePtr penv) :
                IkSolverBase(penv)
{
    __description = ":Interface Author: Shervin Javdani";

    //_target.reserve(7);
    //_initialized = false;
}


TracIK::~TracIK()
{
  delete _tracik_solver_;
}


OpenRAVE::RobotBase::ManipulatorPtr TracIK::GetManipulator() const
{
    return _pmanip;
}

int TracIK::GetNumFreeParameters() const
{
    return 0;
}

bool TracIK::GetFreeParameters(std::vector<double> &v) const
{
    v.clear();
    return true;
}

KDL::JntArray toKDLJntArray(const std::vector<double>& vec)
{
    KDL::JntArray to_ret(vec.size());
    for (size_t i=0; i < vec.size(); i++)
    {
        to_ret(i) = vec[i];
    }
    return to_ret;
}

std::vector<double> toStdVec(const KDL::JntArray& arr)
{
    std::vector<double> vec(arr.rows());
    toStdVec(arr, vec);
    return vec;
}

void toStdVec(const KDL::JntArray& arr, std::vector<double>& vec)
{
    vec.resize(arr.rows());
    for (size_t i=0; i < vec.size(); i++)
    {
        vec[i] = arr(i);
    }
}

bool TracIK::Solve(const OpenRAVE::IkParameterization& params, const std::vector<double>& q0, int filter_options, boost::shared_ptr<std::vector<double> > result)
{
    //reinitialize robot in case bounds or anything else changed
    delete _tracik_solver_;
    Init(_pmanip);


    //target transform is transform between the base link and what is specified by params
    KDL::Frame target_transform= toKDLFrame(_pmanip_base->GetTransform().inverse() * params.GetTransform6D());
    KDL::JntArray tracik_result(q0.size());

    int tracik_return = _tracik_solver_->CartToJnt(toKDLJntArray(q0), target_transform, tracik_result);
    //if tracik said no solution, return now
    if (tracik_return <= 0)
    {
        return false;
    }


    toStdVec(tracik_result, *(result.get()));

    bool checkSelfCollision = !or_helper::HasFlag(filter_options, OpenRAVE::IKFO_IgnoreSelfCollisions);
    bool checkEnvCollision = or_helper::HasFlag(filter_options, OpenRAVE::IKFO_CheckEnvCollisions);

    if (!or_helper::IsSupported(filter_options))
    {
        RAVELOG_WARN("Unsupported filter option %#x. Supported options are:"
                     " %#x, %#x, %#x and %#x.\n",
            filter_options,
            0,
              OpenRAVE::IKFO_CheckEnvCollisions,
              OpenRAVE::IKFO_IgnoreSelfCollisions,
              OpenRAVE::IKFO_CheckEnvCollisions
            | OpenRAVE::IKFO_IgnoreSelfCollisions
        );
    }

    if(checkSelfCollision || checkEnvCollision)
    {
        //RobotStateSaver const saver(_pRobot, OpenRAVE::KinBody::Save_ActiveDOF | OpenRAVE::KinBody::Save_LinkTransformation);
        RobotStateSaver const saver(_pRobot, OpenRAVE::KinBody::Save_ActiveDOF | OpenRAVE::KinBody::Save_LinkTransformation);
        _pRobot->SetDOFValues(*result.get(), true, _pmanip->GetArmIndices());

        if(checkSelfCollision && _pRobot->CheckSelfCollision())
        {
            RAVELOG_DEBUG("No IK solution, robot in self collision.\n");
            return false;
        }

        if(checkEnvCollision && GetEnv()->CheckCollision(_pRobot))
        {
            RAVELOG_DEBUG("Warning: no IK solution, robot colliding with environment.\n");
            return false;
        }
    }

    return true;
}

bool TracIK::Solve(const OpenRAVE::IkParameterization&, const std::vector<double>&, const std::vector<double>&, int, boost::shared_ptr<std::vector<double> >)
{
    RAVELOG_ERROR("Function bool Solve(const OpenRAVE::IkParameterization&, const std::vector<double>&, const std::vector<double>&, int, boost::shared_ptr<std::vector<double> >)not implemented in TracIK.\n");
    return false;
}

bool TracIK::SolveAll(const OpenRAVE::IkParameterization& param, int filterOptions, std::vector<std::vector<double> >& returnValues)
{
    std::vector<double> randSample(_numdofs, 0.0);
    std::vector<double>* solution = new std::vector<double>();
    for (int j = 0; j < _numdofs; j++)
    {
        solution->push_back(0.0);
    }
    boost::shared_ptr<std::vector<double> > solutionPtr(solution);
    bool foundOne = false;
    // Try 1K random samples as starting points
    for (size_t randomSamples = 0; randomSamples < 1000; randomSamples++)
    {
        // Sample between -PI and PI
        for (int j = 0; j < _numdofs; j++)
        {
            randSample[j] = 2.0 * (((double)(rand()) / (double)(RAND_MAX)) * M_PI - M_PI * 0.5);
        }

        for (int j = 0; j < _numdofs; j++)
        {
            (*solution)[j] = 0.0;
        }
        // Solve for this random seed
        if (Solve(param, randSample, filterOptions, solutionPtr) )
        {
            bool isNear = false;
            // Check all the return values to see if we already got a solution like this one
            for (size_t s = 0; s < returnValues.size(); s++)
            {
                bool nearSolution = true;
                const std::vector<double>& currentSol = returnValues[s];
                for (int j = 0; j < _numdofs; j++)
                {
                    double diff = fabs(currentSol[j] - (*solution)[j]);

                    if (diff > 1e-1)
                    {
                        nearSolution = false;
                        break;
                    }
                }

                if (nearSolution)
                {
                    isNear = true;
                    break;
                }
            }

            // If this is far enough away from other solutions, accept it.
            if (!isNear)
            {
                foundOne = true;
                returnValues.push_back(*solution);
            }
        }
    }

    return foundOne;
    //RAVELOG_ERROR("WARNING: Function bool SolveAll(const OpenRAVE::IkParameterization&, const std::vector<double>&, int, std::vector<std::vector<double> >&)not implemented in TracIK.\n");
    //return false;
}

bool TracIK::SolveAll(const OpenRAVE::IkParameterization& param, const std::vector<double>& q0, int filterOptions, std::vector<std::vector<double> >& returnValues)
{
    return SolveAll(param, filterOptions, returnValues);
}

