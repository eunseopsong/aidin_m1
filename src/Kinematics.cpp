#include <Kinematics.h>

Vector4d Variable::dummy(0, 0, 0, 1);
// extern Robot aidin;

//// Lie Algebra
/// @brief Rotation Matrix x
/// @param theta Roll angle
/// @return R
///     Rotation matrix with X axis
Matrix<double, 3, 3> LieAlgebra::Rotx(double theta)
{
    Matrix<double, 3, 3> R;
    R << 1, 0, 0,
        0, cos(theta), -sin(theta),
        0, sin(theta), cos(theta);
    return R;
}

/// @brief Rotation Matrix y
/// @param theta Pitch angle
/// @return R
///     Rotation matrix with Y axis
Matrix<double, 3, 3> LieAlgebra::Roty(double theta)
{
    Matrix<double, 3, 3> R;
    R << cos(theta), 0, sin(theta),
        0, 1, 0,
        -sin(theta), 0, cos(theta);
    return R;
}

/// @brief Rotation Matrix z
/// @param theta Yaw angle
/// @return R
///     Rotation matrix with Z axis
Matrix<double, 3, 3> LieAlgebra::Rotz(double theta)
{
    Matrix<double, 3, 3> R;
    R << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    return R;
}

// RpToTrans
/// @brief Set transition matrix
/// @param R Rotation matrix
/// @param p Transition vector
/// @return T
///     Transition matrix
Matrix<double, 4, 4> LieAlgebra::RpToTrans(Matrix<double, 3, 3> R, Vector3d p)
{
    Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;
    T.block<1, 4>(3, 0) = Variable::dummy.transpose();
    return T;
}

// RPY
/// @brief Rotation with the order of Yaw, Pitch, Roll
/// @param orn_d Euler angle, RPY (Unit : rad)
/// @return RPY
///     Rotation matrix
Matrix<double, 3, 3> LieAlgebra::RPY(Vector3d orn_d)
{
    Matrix<double, 3, 3> Rx_d;
    Matrix<double, 3, 3> Ry_d;
    Matrix<double, 3, 3> Rz_d;

    Rx_d = LieAlgebra::Rotx(orn_d(0));
    Ry_d = LieAlgebra::Roty(orn_d(1));
    Rz_d = LieAlgebra::Rotz(orn_d(2));

    Matrix<double, 3, 3> RPY;
    RPY = (Rx_d * Ry_d) * Rz_d;
    return RPY;
}

// YPR
/// @brief Rotation with the order of Roll, Pitch, Yaw
/// @param orn_d Euler angle, RPY (Unit : rad)
/// @return YPR
///     Rotation matrix
Matrix<double, 3, 3> LieAlgebra::YPR(Vector3d orn_d)
{
    Matrix<double, 3, 3> Rx_d;
    Matrix<double, 3, 3> Ry_d;
    Matrix<double, 3, 3> Rz_d;

    Rx_d = LieAlgebra::Rotx(orn_d(0));
    Ry_d = LieAlgebra::Roty(orn_d(1));
    Rz_d = LieAlgebra::Rotz(orn_d(2));

    Matrix<double, 3, 3> YPR;
    YPR = Rz_d * Ry_d * Rx_d;
    return YPR;
}

//// Forward Kinematics Matrices
// T01
/// @brief Transition matrix from body to scapular
/// @param index The order of LEG
/// @return T
///     Transition matrix
Matrix<double, 4, 4> ForwardKinematic::T01_matrix(int index)
{
    Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
    double bx;
    double by = pow(-1, index) * YBODY;
    if (index == 0 || index == 1)
        bx = XBODY;
    else
        bx = -XBODY;
    Vector3d pos(bx, by, 0);
    T = LieAlgebra::RpToTrans(MatrixXd::Identity(3, 3), pos);
    return T;
}

// // T12
// Matrix<double, 4, 4> ForwardKinematic::T12_matrix(int index)
// {
//     Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
//     Vector3d pos(0, 0, 0);
//     double scap_angle = State::joint_angles(0, index);
//     T = LieAlgebra::RpToTrans(LieAlgebra::Rotx(scap_angle), pos);
//     return T;
// }

// // T23
// //// sign of link is different
// Matrix<double, 4, 4> ForwardKinematic::T23_matrix(int index)
// {
//     Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
//     double hip_angle = State::joint_angles(1, index);
//     // double l1 = LINKSCAP * pow(-1, index + 1);
//     double l1 = LINKSCAP * pow(-1, index);
//     Vector3d pos(0, l1, 0);
//     // T = LieAlgebra::RpToTrans(LieAlgebra::Roty(hip_angle), pos);
//     T = LieAlgebra::RpToTrans(LieAlgebra::Roty(-hip_angle), pos);
//     return T;
// }

// // T34
// //// It doesn't consider AIDIN8 coordinates
// Matrix<double, 4, 4> ForwardKinematic::T34_matrix(int index)
// {
//     Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
//     // double knee_angle = State::joint_angles(2, index);
//     double knee_angle = pi - State::joint_angles(1, index) + State::joint_angles(2, index);
//     Vector3d pos(0, 0, LINKHIP);
//     // T = LieAlgebra::RpToTrans(LieAlgebra::Roty(knee_angle), pos);
//     T = LieAlgebra::RpToTrans(LieAlgebra::Roty(-knee_angle), pos);
//     return T;
// }

// // T45
// //// sign of link is different
// Matrix<double, 4, 4> ForwardKinematic::T45_matrix(int index)
// {
//     Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
//     // double l1 = LINKSCAP * pow(-1, index + 1);
//     // double l1 = LINKSCAP * pow(-1, index);
//     Vector3d pos(0, 0, LINKKNEE);
//     T = LieAlgebra::RpToTrans(MatrixXd::Identity(3, 3), pos);
//     return T;
// }

// // T05
// Matrix<double, 4, 4> ForwardKinematic::T05_matrix(int index){
//     Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
//     Matrix<double, 4, 4> T01, T12, T23, T34, T45;
//     T01 = ForwardKinematic::T01_matrix(index);
//     T12 = ForwardKinematic::T12_matrix(index);
//     T23 = ForwardKinematic::T23_matrix(index);
//     T34 = ForwardKinematic::T34_matrix(index);
//     T45 = ForwardKinematic::T45_matrix(index);

//     T = T01 * T12 * T23 * T34 * T45;
//     return T;
// }

// // T15
// Matrix<double, 4, 4> ForwardKinematic::T15_matrix(int index)
// {
//     Matrix<double, 4, 4> T = MatrixXd::Zero(4, 4);
//     Matrix<double, 4, 4>  T12, T23, T34, T45;
//     T12 = ForwardKinematic::T12_matrix(index);
//     T23 = ForwardKinematic::T23_matrix(index);
//     T34 = ForwardKinematic::T34_matrix(index);
//     T45 = ForwardKinematic::T45_matrix(index);

//     T = T12 * T23 * T34 * T45;
//     return T;
// }

// // RPY Homogenous T01 Matrix
// //// Tbody size is (1x3), so it should be changed to (3x1) size
// Matrix<double, 4, 4> ForwardKinematic::rpyT01_matrix(int index)
// {
//     Matrix<double, 3, 3> aR;
//     Matrix<double, 4, 4> rpyT01;
//     Vector3d Tbody, aBody;

//     aR = LieAlgebra::YPR(State::act_orn);
//     // Tbody = (ForwardKinematic::T01_matrix(index)).block<3, 1>(0, 3).transpose();
//     Tbody = (ForwardKinematic::T01_matrix(index)).block<3, 1>(0, 3);
//     aBody = aR * Tbody;
//     rpyT01 = LieAlgebra::RpToTrans(aR, aBody);
//     return rpyT01;
// }

// // RPY Homogenous T05 Matrix
// Matrix<double, 4, 4> ForwardKinematic::rpyT05_matrix(int index)
// {
//     Matrix<double, 4, 4> rpyT05, rpyT01, T12, T23, T34, T45;
//     rpyT01 = ForwardKinematic::rpyT01_matrix(index);
//     T12 = ForwardKinematic::T12_matrix(index);
//     T23 = ForwardKinematic::T23_matrix(index);
//     T34 = ForwardKinematic::T34_matrix(index);
//     T45 = ForwardKinematic::T45_matrix(index);
//     rpyT05 = rpyT01 * T12 * T23 * T34 * T45;
//     return rpyT05;
// }

// Rot
//// It isn't needed.
// Matrix<double, 12, 12> ForwardKinematic::get_Rot()
// {
//     MatrixXd Rot = MatrixXd::Zero(12, 12);
//     for (int i = 0; i < 4; i++)
//     {
//         Rot.block<3, 3>(3 * i, 3 * i) = LieAlgebra::YPR(State::act_orn);
//     }
//     return Rot;
// }

/// @brief Get local foot position with forward kinematics
/// @param _LEG The order of LEG
/// @param _theta Joint angle of LEG
/// @return Value
///     Foot position from scapular
Vector3d ForwardKinematic::getFootPosToScapOfLocal(int _LEG, Vector3d _theta)
{
    ////// This matrix is same with T15_matrix, but it is more optimized thing
    int _sign = 1;
    //// In case, LEG is RF or RB
    if( _LEG == 1 || _LEG == 3 )
        _sign = -1;

    return {
        -LINKHIP*sin(_theta(1)) + LINKKNEE*sin(_theta(2)), 
        _sign*LINKSCAP*cos(_theta(0)) + ( -LINKHIP*cos(_theta(1)) + LINKKNEE*cos(_theta(2)) )*sin(_theta(0)), 
        _sign*LINKSCAP*sin(_theta(0)) - ( -LINKHIP*cos(_theta(1)) + LINKKNEE*cos(_theta(2)) )*cos(_theta(0))
    };
}

/// @brief Calculate foot position with scapular frame
/// @param jointAngles Joint angle of all legs
void ForwardKinematic::calculateFootPosInScapFrame(Matrix<double, 3, 4> jointAngles)
{
    p_hf.setZero();
    for (int i_leg = 0; i_leg < 4; i_leg++)
    {
        p_hf.col(i_leg) = getFootPosToScapOfLocal(i_leg, jointAngles.col(i_leg));
    }
}

/// @brief Get foot position with scapular frame
/// @return p_hf
///     Foot position with scapular frame
Matrix<double, 3, 4> ForwardKinematic::getFootPosInScapFrame()
{
    return p_hf;
}

//// Inverse Kinematic
/// @brief Initialize parameters
void InverseKinematic::InitKinematics()
{
    minangle = 14.0/180.0*pi; // Mininum L2 and L3 angle is 15 degree
    Vector3d _minPos, _maxPos;
    _minPos << 0, pi/2, pi/2 - minangle;
    MinPos = ForwardKinematic::getFootPosToScapOfLocal(0, _minPos);
    _maxPos << 0, pi, 0;
    MaxPos = ForwardKinematic::getFootPosToScapOfLocal(0, _maxPos);
    Rmin = 1.001*sqrt( dotProduct(MinPos, MinPos) );
    Rmax = 0.999*sqrt( dotProduct(MaxPos, MaxPos) );
    Zbound = LINKSCAP;
}
// Body IK
//// I don't know what is pos, Rb.
// Matrix<double, 3, 4> InverseKinematic::BodyIK(Vector3d pos, Matrix<double, 3, 3> Rb, Matrix<double, 3, 4> p_bf)
// {
//     Matrix<double, 3, 4> p_hf1;
    
//     Matrix<double, 3, 3> Rwb;
//     Rwb = MatrixXd::Identity(3, 3);


//     Matrix<double, 4, 4> T_wb;
//     T_wb = LieAlgebra::RpToTrans(Rb, pos);

//     for (int i = 0; i < 4; i++)
//     {
//         Matrix<double, 4, 4> T_wh;
//         T_wh = LieAlgebra::RpToTrans(Rwb, aidin.p_bh_offset.col(i));

//         Matrix<double, 4, 4> T_bh;
//         T_bh = T_wb * T_wh;

//         Vector3d p_bhi;
//         p_bhi = T_bh.block<3, 1>(0, 3);

//         p_hf1.col(i) = p_bf.col(i) - p_bhi;
//     }
//     return p_hf1;
// }

#if SELECTKINEMATICS == 1
/// @brief Calculate inverse kinematics of each leg
/// @param _LEG The order of LEG
/// @param _des_p_hf Desired foot position with XYZ
/// @param _theta Previous joint angle of LEG
/// @return _turnCount
///     Multi-turn count of each leg
/// @return _errorCount
///     Difference with XYZ target and calculated joint angle
/// @return _desJointAngles
///     Calculated joint angle with inverse kinematics
Matrix<double, 3, 1> InverseKinematic::LegIK(int _LEG, const Ref<Matrix<double, 3, 1>> _des_p_hf, Ref<Matrix<int, 4, 1>> _turnCount, Ref<Matrix<double, 4, 1>> _errorCount, const Ref<Vector3d> _theta)
{
    //// p0 : Origin of scapular link with local frame
    //// p1 : End of scapular link from p0 with local frame
    //// p2 : End of hip link from p0 with local frame
    //// p3 : End of knee link from p0 with local frame

    Matrix<double, 3, 1> _desJointAngles;    _desJointAngles.setZero();
    int _error = 0;
    
    //// _footPos == p3 ////
    Vector3d _footPos = _des_p_hf;
    double _footPosMag = sqrt( dotProduct(_footPos, _footPos) );
    Vector3d _footPosUnit = _footPos/_footPosMag;

    Vector3d _YZvec;        _YZvec.setZero();
    Vector3d _YZvecUnit;    _YZvecUnit.setZero();
    Vector3d _currentPos;   _currentPos.setZero();
    
    int _sign = -1;
    ////// If leg is RF or RB
    if( _LEG == 1 || _LEG == 3 )
        _sign = 1;

    double _L1 = _sign*LINKSCAP;
    double _L2 = LINKHIP;
    double _L3 = LINKKNEE;


    ////// Out of the workspace
    ////// If foot position is out of the workspace, then change foot position to inside workspace
    ////// Outside the workspace
    if( Rmax < _footPosMag )
    {
		// std::cout << "No solution, out of workspace. It could be outside" << std::endl;
		_footPos = _footPosUnit*Rmax;
		_error = -1;
    }
    ////// Inside the workspace
	if( Rmin > _footPosMag )
	{
		// std::cout << "No solution, out of workspace. It could be inside" << std::endl;
        ////// If footPos is origin, then change footPos to previous point
        if( _footPosMag == 0 )
            _footPos = ForwardKinematic::getFootPosToScapOfLocal(_LEG, _theta);
        _footPosMag = sqrt( dotProduct(_footPos, _footPos) );
        _footPosUnit = _footPos/_footPosMag;
		_footPos = _footPosUnit*Rmin;
		_error = -1;
	}
    ////// Inside the frontal cylinder 
    if( (_footPos(1)*_footPos(1) + _footPos(2)*_footPos(2)) < LINKSCAP*LINKSCAP )
    {
        // std::cout << "No solution, out of scapular workspace. It could be inside" << std::endl;
        ////// If footPos is on X axis, then change footPos of YZ plane to previous point
        if( _footPos(1) == 0 && _footPos(2) == 0 )
        {
            _currentPos = ForwardKinematic::getFootPosToScapOfLocal(_LEG, _theta);
            _footPos(1) = _currentPos(1);
            _footPos(2) = _currentPos(2);
        }
        _YZvec << 0, _footPos(1), _footPos(2);
        _YZvecUnit = _YZvec/sqrt( dotProduct(_YZvec, _YZvec) );
        _footPos(1) = LINKSCAP*_YZvecUnit(1);
        _footPos(2) = LINKSCAP*_YZvecUnit(2);

        ////// If footPos is on outside of the workspace after changing footPos, then change footPos to point of inner workspace
        _footPosMag = sqrt( dotProduct(_footPos, _footPos) );
        if( Rmax < _footPosMag )
        {
            if( _footPos(0) >= 0 )
                _footPos(0) = sqrt( Rmax*Rmax - LINKSCAP*LINKSCAP );
            else
                _footPos(0) = -sqrt( Rmax*Rmax - LINKSCAP*LINKSCAP );
        }
        ////// If footPos is on inside of the workspace after changing footPos, then change footPos to point of inner workspace
        else if( Rmin > _footPosMag )
        {
            if( _footPos(0) >= 0 )
                _footPos(0) = sqrt( Rmin*Rmin - LINKSCAP*LINKSCAP );
            else
                _footPos(0) = -sqrt( Rmin*Rmin - LINKSCAP*LINKSCAP );
        }
        _error = -1;
    }
    ////// Out of the scapular range
    if( (_sign*_footPos(1) > 0) && (fabs(_footPos(2)) < Zbound) )
    {
        // std::cout << "No solution, out of range from -90 to +90" << std::endl;
        ////// Change Z footPos to inner workspace area
        if( _footPos(2) < 0 )
            _YZvec << 0, _footPos(1), -Zbound;
        else if( _footPos(2) > 0 )
            _YZvec << 0, _footPos(1), Zbound;
        else
        {
            if( _theta(0) > 0 )
                _YZvec << 0, _footPos(1), -_sign*Zbound;
            else
                _YZvec << 0, _footPos(1), _sign*Zbound;
        }
        ////// If footPos of YZ plane is on outside of the workspace after changing footPos, the change Y footPos to inner workspace area
        if( sqrt( dotProduct(_YZvec, _YZvec) ) > Rmax )
            _YZvec(1) = _sign*sqrt( Rmax*Rmax - Zbound*Zbound );
        _footPos(1) = _YZvec(1);
        _footPos(2) = _YZvec(2);
        ////// If footPos is on outside of the workspace after changing footPos, the change X, Y footPos to inner workspace area with XY vector direction
        if( Rmax < sqrt( dotProduct(_footPos, _footPos) ) )
        {
            Vector3d _XYvec(_footPos(0), _footPos(1), 0);
            Vector3d _XYvecUnit = _XYvec/sqrt( dotProduct(_XYvec, _XYvec) );
            if( _footPos(0) >= 0 )
            {
                _footPos(0) = sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(0);
                _footPos(1) = sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(1);
            }
            else
            {
                _footPos(0) = -sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(0);
                _footPos(1) = -sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(1);
            }
        }
        _error = -1;        
    }

    ////// Calculate scapular angle
    ////// Distance from p1 to p3 with yz plane
    double _sqrtComponent = _footPos(1)*_footPos(1) + _footPos(2)*_footPos(2) - _L1*_L1;
    if( _sqrtComponent < 0.0 )
        _sqrtComponent = 0;
    double _scapAngle = 0;
    if( _footPos(1) == 0 )
    {
        // Downside scapular
        if( _footPos(2) <= 0 )
            _scapAngle = pi/2.0 - atan2(sqrt(_sqrtComponent), _L1);
        // Upside scapular
        else
            _scapAngle = -(pi/2.0 + atan2(sqrt(_sqrtComponent), _L1));
    }
    else
    {
        ////// LF, LB //////
        if( _LEG == 0 || _LEG == 2 )
        {
            ////// If footPos is in the 3rd quadrant of YZ plane, p3 is lower than p01
            if( _footPos(1) < 0 && _footPos(2) < 0 )
                _scapAngle = (pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
            ////// If footPos is in the 2nd quadrant of YZ plane, p3 is higher than p01
            else if( _footPos(1) < 0 && _footPos(2) > 0 )
                _scapAngle = -(pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
            else
            {
                double _angleBelow = (pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
                double _angleAbove = -(pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
                ////// If previous angle of scapular is downside, then angleBelow is more considered
                if( _theta(0) <= 0 )
                {
                    if( fabs(_theta(0) - (_angleBelow - 0.0001)) < fabs(_theta(0) - _angleAbove) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
                ////// If previous angle of scapular is upside, then angleAbove is more considered
                else
                {
                    if( fabs(_theta(0) - _angleBelow) < fabs(_theta(0) - (_angleAbove + 0.0001)) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
            }
        }
        ////// RF, RB //////
        else
        {
            ////// If footPos is in the 4th quadrant of YZ plane, p3 is lower than p01
            if( _footPos(1) > 0 && _footPos(2) < 0 )
                _scapAngle = (pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
            ////// If footPos is in the 1st quadrant of YZ plane, p3 is higher than p01
            else if( _footPos(1) > 0 && _footPos(2) > 0 )
                _scapAngle = -(pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
            else
            {
                double _angleBelow = (pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
                double _angleAbove = -(pi - atan2(sqrt(_sqrtComponent), _L1)) + atan2(_footPos(2), _footPos(1));
                ////// If two angle are out of range, then change two angle to inner range
                if( _angleBelow > pi )
                    _angleBelow = _angleBelow - 2*pi;
                if( _angleAbove < -pi )
                    _angleAbove = _angleAbove + 2*pi;
                
                ////// If previous angle of scapular is upside, then angleAbove is more considered
                if( _theta(0) < 0 )
                {
                    if( fabs(_theta(0) - _angleBelow) < fabs(_theta(0) - (_angleAbove - 0.0001)) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
                ////// If previous angle of scapular is downside, then angleBelow is more considered
                else
                {
                    if( fabs(_theta(0) - (_angleBelow + 0.0001)) < fabs(_theta(0) - _angleAbove) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
            }
        }
    }

    ////// Calculate knee angle between Hip & Knee links
    ////// Use the 2nd cosine rule formula
    ////// Triangle is consisted of p1 & p2 & p3
    ////// kneeAngleGap is angle between hip & knee links
    double _kneeAngleGap = acos( ( _footPos(1)*_footPos(1) + _footPos(2)*_footPos(2) - _L1*_L1 + _footPos(0)*_footPos(0) - _L2*_L2 - _L3*_L3 )/( -2*_L2*_L3 ) );

    ////// Calculate Z axis direction of plane with p1 & p3
    Vector3d _p1(0, -_sign*LINKSCAP*cos(_scapAngle), -_sign*LINKSCAP*sin(_scapAngle));
    Vector3d _p13 = _footPos - _p1;
    Vector3d _Xvec(-_sign, 0, 0);
    Vector3d _ZvecUnit = crossProduct(_Xvec, _p1);
    double _ZvecP13 = dotProduct(_ZvecUnit, _p13);

    ////// Calculate hip angle
    double _hipAngle = 0;
    ////// If p3 is lower than p1
    if( _ZvecP13 < 0 )
        _hipAngle = -( atan2(-_footPos(0), sqrt(_sqrtComponent)) + atan2(-LINKKNEE*sin(_kneeAngleGap), -LINKHIP + LINKKNEE*cos(_kneeAngleGap)) );
    ////// If p3 is higher than p1
    else if( _ZvecP13 > 0 )
        _hipAngle = -( atan2(-_footPos(0), -sqrt(_sqrtComponent)) + atan2(-LINKKNEE*sin(_kneeAngleGap), -LINKHIP + LINKKNEE*cos(_kneeAngleGap)) );
    ////// If p3 is same with p1
    else
    {
        ////// If X footPos is backward of p1, then knee joint is upper than p1
        if( _footPos(0) <= 0 )
            _hipAngle = pi/2.0 - acos( (LINKHIP*LINKHIP + _footPos(0)*_footPos(0) - LINKKNEE*LINKKNEE)/(2*LINKHIP*abs(_footPos(0))) );
        ////// If X footPos is forward of p1, then knee joint is lower than p1
        else
            _hipAngle = 3.0*pi/2.0 - acos( (LINKHIP*LINKHIP + _footPos(0)*_footPos(0) - LINKKNEE*LINKKNEE)/(2*LINKHIP*abs(_footPos(0))) );
    }
    ////// Shift hipAngle from -180 < x < +180 ti 0 < x < 360
    if( _hipAngle < 0 )
        _hipAngle = 2*pi + _hipAngle;
    ////// Calculate multi-turnCount of _hipAngle
    double _angleBefore = _theta(1) - _turnCount(_LEG)*2*pi;
    if( (_angleBefore - _hipAngle) > (2*pi - _angleBefore + _hipAngle) )
		_turnCount(_LEG) = _turnCount(_LEG) + 1;
	else if( (_hipAngle - _angleBefore) > (2*pi - _hipAngle + _angleBefore) )
		_turnCount(_LEG) = _turnCount(_LEG) - 1;
	_hipAngle = _hipAngle + 2*pi*_turnCount(_LEG);


    ////// Change knee angle with AIDIN8 coordinates
    double _kneeAngle = _hipAngle - _kneeAngleGap;
    // Calculate multi-turnCount of _kneeAngle
	_kneeAngle = _kneeAngle + 2*pi*_turnCount(_LEG);
    // Shifting the range of _kneeAngle from -180 < x < +180 to  0 < x < 360
	if( (_hipAngle - 2*pi > _kneeAngle) || (_kneeAngle > _hipAngle - minangle) )
        _kneeAngle = 2*pi + _kneeAngle;

    _desJointAngles << _scapAngle, _hipAngle, _kneeAngle;
    
    if( _error == -1 ) 
        _errorCount(_LEG) = _error;
    else
    {
        _currentPos = ForwardKinematic::getFootPosToScapOfLocal(_LEG, _desJointAngles);
        _errorCount(_LEG) = sqrt( dotProduct( _footPos - _currentPos, _footPos - _currentPos) );
    }

    return _desJointAngles;
}
#elif SELECTKINEMATICS == 0
/// @brief Calculate inverse kinematics of each leg
/// @param _LEG The order of LEG
/// @param _des_p_hf Desired foot position with XYZ
/// @param _theta Previous joint angle of LEG
/// @return _turnCount
///     Multi-turn count of each leg
/// @return _errorCount
///     Difference with XYZ target and calculated joint angle
/// @return _desJointAngles
///     Calculated joint angle with inverse kinematics
Matrix<double, 3, 1> InverseKinematic::LegIK(int _LEG, const Ref<Matrix<double, 3, 1>> _des_p_hf, Ref<Matrix<int, 4, 1>> _turnCount, Ref<Matrix<double, 4, 1>> _errorCount, const Ref<Vector3d> _theta)
{
    //// p0 : Origin of scapular link with local frame
    //// p1 : End of scapular link from p0 with local frame
    //// p2 : End of hip link from p0 with local frame
    //// p3 : End of knee link from p0 with local frame

    Matrix<double, 3, 1> _desJointAngles;    _desJointAngles.setZero();
    int _error = 0;
    
    ////// leg is LF or LB //////
    int _sign = -1;
    ////// leg is RF or RB //////
    if( _LEG == 1 || _LEG == 3 )
        _sign = 1;

    //// _footPos == p3 ////
    Vector3d _footPos = _des_p_hf;
    double _footPosMag = sqrt( dotProduct(_footPos, _footPos) );
    Vector3d _footPosUnit = _footPos/_footPosMag;

    Vector3d _YZvec;        _YZvec.setZero();
    Vector3d _YZvecUnit;    _YZvecUnit.setZero();
    Vector3d _currentPos;   _currentPos.setZero();

    ////// Out of the workspace
    ////// If foot position is out of the workspace, then change foot position to inside workspace
    ////// Outside the workspace
    if( Rmax < _footPosMag )
    {
		// std::cout << "No solution, out of workspace. It could be outside" << std::endl;
		_footPos = _footPosUnit*Rmax;
		_error = -1;
    }
    ////// Inside the workspace
	if( Rmin > _footPosMag )
	{
		// std::cout << "No solution, out of workspace. It could be inside" << std::endl;
        ////// If footPos is origin, then change footPos to previous point
        if( _footPosMag == 0 )
            _currentPos = ForwardKinematic::getFootPosToScapOfLocal(_LEG, _theta);
        _footPosMag = sqrt( dotProduct(_currentPos, _currentPos) );
        _footPosUnit = _currentPos/_footPosMag;
		_footPos = _footPosUnit*Rmin;
		_error = -1;
	}
    ////// Inside the frontal cylinder 
    if( (_footPos(1)*_footPos(1) + _footPos(2)*_footPos(2)) < LINKSCAP*LINKSCAP )
    {
        // std::cout << "No solution, out of scapular workspace. It could be inside" << std::endl;
        ////// If footPos is on X axis, then change footPos of YZ plane to previous point
        if( _footPos(1) == 0 && _footPos(2) == 0 )
        {
            _currentPos = ForwardKinematic::getFootPosToScapOfLocal(_LEG, _theta);
            _footPos(1) = _currentPos(1);
            _footPos(2) = _currentPos(2);
        }
        _YZvec << 0, _footPos(1), _footPos(2);
        _YZvecUnit = _YZvec/sqrt( dotProduct(_YZvec, _YZvec) );
        _footPos(1) = LINKSCAP*_YZvecUnit(1);
        _footPos(2) = LINKSCAP*_YZvecUnit(2);

        ////// If footPos is on outside of the workspace after changing footPos, then change footPos to point of inner workspace
        _footPosMag = sqrt( dotProduct(_footPos, _footPos) );
        if( Rmax < _footPosMag )
        {
            if( _footPos(0) >= 0 )
                _footPos(0) = sqrt( Rmax*Rmax - LINKSCAP*LINKSCAP );
            else
                _footPos(0) = -sqrt( Rmax*Rmax - LINKSCAP*LINKSCAP );
        }
        ////// If footPos is on inside of the workspace after changing footPos, then change footPos to point of inner workspace
        else if( Rmin > _footPosMag )
        {
            if( _footPos(0) >= 0 )
                _footPos(0) = sqrt( Rmin*Rmin - LINKSCAP*LINKSCAP );
            else
                _footPos(0) = -sqrt( Rmin*Rmin - LINKSCAP*LINKSCAP );            
        }
        _error = -1;
    }
    ////// Out of the scapular range
    if( (_sign*_footPos(1) > 0) && (fabs(_footPos(2)) < Zbound) )
    {
        // std::cout << "No solution, out of range from -90 to +90" << std::endl;
        ////// Change Z footPos to inner workspace area
        if( _footPos(2) < 0 )
            _YZvec << 0, _footPos(1), -Zbound;
        else if( _footPos(2) > 0 )
            _YZvec << 0, _footPos(1), Zbound;
        else
        {
            if( _theta(0) > 0 )
                _YZvec << 0, _footPos(1), -_sign*Zbound;
            else
                _YZvec << 0, _footPos(1), _sign*Zbound;
        }
        ////// If footPos of YZ plane is on outside of the workspace after changing footPos, the change Y footPos to inner workspace area
        if( sqrt( dotProduct(_YZvec, _YZvec) ) > Rmax )
            _YZvec(1) = _sign*sqrt( Rmax*Rmax - Zbound*Zbound );
        _footPos(1) = _YZvec(1);
        _footPos(2) = _YZvec(2);
        ////// If footPos is on outside of the workspace after changing footPos, the change X, Y footPos to inner workspace area with XY vector direction
        if( Rmax < sqrt( dotProduct(_footPos, _footPos) ) )
        {
            Vector3d _XYvec(_footPos(0), _footPos(1), 0);
            Vector3d _XYvecUnit = _XYvec/sqrt( dotProduct(_XYvec, _XYvec) );
            if( _footPos(0) >= 0 )
            {
                _footPos(0) = sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(0);
                _footPos(1) = sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(1);
            }
            else
            {
                _footPos(0) = -sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(0);
                _footPos(1) = -sqrt( Rmax*Rmax - Zbound*Zbound )*_XYvecUnit(1);
            }
        }
        _error = -1;
    }

    ////// Calculate scapular angle
    ////// Distance from p1 to p3 with yz plane
    double _sqrtComponent = LINKSCAP/sqrt( _footPos(1)*_footPos(1) + _footPos(2)*_footPos(2) );
    if( _sqrtComponent >= 1 )
        _sqrtComponent = 1;
    else if( _sqrtComponent <= -1 )
        _sqrtComponent = -1;
    double _scapAngle = 0;
    if( _footPos(1) == 0 )
    {
        // Downside scapular
        if( _footPos(2) <= 0 )
            _scapAngle = _sign*( pi/2.0 - acos(_sqrtComponent) );
        // Upside scapular
        else
            _scapAngle = _sign*( acos(_sqrtComponent) - pi/2.0 );
    }
    else
    {
        ////// LF, LB //////
        if( _LEG == 0 || _LEG == 2 )
        {
            ////// If footPos is in the 3rd quadrant of YZ plane, p3 is lower than p01
            if( _footPos(1) < 0 && _footPos(2) < 0 )
                _scapAngle = atan2(_footPos(2), _footPos(1)) + acos(_sqrtComponent);
            ////// If footPos is in the 2nd quadrant of YZ plane, p3 is higher than p01
            else if( _footPos(1) < 0 && _footPos(2) > 0 )
                _scapAngle = atan2(_footPos(2), _footPos(1)) - acos(_sqrtComponent);
            else
            {
                double _angleBelow = atan2(_footPos(2), _footPos(1)) + acos(_sqrtComponent);
                double _angleAbove = atan2(_footPos(2), _footPos(1)) - acos(_sqrtComponent);
                ////// If previous angle of scapular is downside, then angleBelow is more considered
                if( _theta(0) <= 0 )
                {
                    if( fabs(_theta(0) - (_angleBelow - 0.0001)) < fabs(_theta(0) - _angleAbove) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
                ////// If previous angle of scapular is upside, then angleAbove is more considered
                else if( _theta(0) > 0 )
                {
                    if( fabs(_theta(0) - _angleBelow) < fabs(_theta(0) - (_angleAbove + 0.0001)) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
                else
                {
                    if( fabs( _theta(0) - _angleBelow) < fabs(_theta(0) - _angleAbove) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
            }
        }
        else
        {
            ////// If footPos is in the 4th quadrant of YZ plane, p3 is lower than p01
            if( _footPos(1) > 0 && _footPos(2) < 0 )
                _scapAngle = pi + ( atan2(_footPos(2), _footPos(1)) - acos(_sqrtComponent) );
            ////// If footPos is in the 1st quadrant of YZ plane, p3 is higher than p01
            else if( _footPos(1) > 0 && _footPos(2) > 0 )
                _scapAngle = -pi + ( atan2(_footPos(2), _footPos(1)) + acos(_sqrtComponent) );
            else
            {
                double _angleBelow = pi + ( atan2(_footPos(2), _footPos(1)) - acos(_sqrtComponent) );
                double _angleAbove = -pi + ( atan2(_footPos(2), _footPos(1)) + acos(_sqrtComponent) );
                ////// If two angle are out of range, then change two angle to inner range
                if( _angleBelow > pi )
                    _angleBelow = _angleBelow - 2*pi;
                if( _angleAbove < -pi )
                    _angleAbove = _angleAbove + 2*pi;

                ////// If previous angle of scapular is upside, then angleAbove is more considered
                if( _theta(0) >= 0 )
                {
                    if( fabs(_theta(0) - (_angleBelow + 0.0001)) < fabs(_theta(0) - _angleAbove) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
                ////// If previous angle of scapular is downside, then angleBelow is more considered
                else if( _theta(0) < 0 )
                {
                    if( fabs(_theta(0) - _angleBelow) < fabs(_theta(0) - (_angleAbove - 0.0001)) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
                else
                {
                    if( fabs( _theta(0) - _angleBelow) < fabs(_theta(0) - _angleAbove) )
                        _scapAngle = _angleBelow;
                    else
                        _scapAngle = _angleAbove;
                }
            }
        }
    }

    ////// Calculate Z axis direction of plane with p1 & p3
    Vector3d _p1(0, -_sign*LINKSCAP*cos(_scapAngle), -_sign*LINKSCAP*sin(_scapAngle));
    Vector3d _p13 = _footPos - _p1;
    double _p13Mag = sqrt( dotProduct(_p13, _p13) );
    _p13 = _p13/_p13Mag;
    Vector3d _Zvec = _sign*( -crossProduct(_p1, Xvec) );
    _Zvec = _Zvec/sqrt( dotProduct(_Zvec, _Zvec) );

    double _cosvalue = dotProduct(_Zvec, _p13);
    double _sinvalue = dotProduct(Xvec, _p13);

    ////// Calculate hip angle
    double _hipAngle = atan2(_sinvalue, _cosvalue);
    ////// Calculate hipAngle with the 2nd cosine law
    _hipAngle = _hipAngle - acos( (_p13Mag*_p13Mag + LINKHIP*LINKHIP - LINKKNEE*LINKKNEE)/(2*_p13Mag*LINKHIP) );
    ////// Shift the range of hipAngle from -180 < x < +180 to 0 < x < 360
    if( _hipAngle < 0 )
        _hipAngle = 2*pi + _hipAngle;


    ////// Calculate multi-turnCount of _hipAngle
    double _angleBefore = _theta(1) - _turnCount(_LEG)*2*pi;
    if ( (_angleBefore - _hipAngle) > (2*pi - _angleBefore + _hipAngle) )
	{
		_turnCount(_LEG) = _turnCount(_LEG) + 1;
	}
	else if ( (_hipAngle - _angleBefore) > (2*pi - _hipAngle + _angleBefore) )
	{
		_turnCount(_LEG) = _turnCount(_LEG) - 1;
	}
	_hipAngle = _hipAngle + 2*pi*_turnCount(_LEG);

    ////// Calculate vector from p3 to p2
    Vector3d _p2(-LINKHIP*sin(_hipAngle), _sign*(-LINKSCAP*cos(_scapAngle))-LINKHIP*cos(_hipAngle)*sin(_scapAngle), _sign*(-LINKSCAP*sin(_scapAngle))+LINKHIP*cos(_hipAngle)*cos(_scapAngle) );
    Vector3d _p32 = _p2 - _footPos;
    _p32 = _p32/sqrt( dotProduct(_p32, _p32) );

    ////// Calculate knee angle between Hip & Knee links
    _cosvalue = dotProduct(_Zvec, _p32);
    _sinvalue = dotProduct(Xvec, _p32);
    double _kneeAngle = atan2(_sinvalue, _cosvalue);
    ////// Calculate multi-turnCount of _kneeAngle
	_kneeAngle = _kneeAngle + 2*pi*_turnCount(_LEG);
    ////// Shifting the range of _kneeAngle from -180 < x < +180 to  0 < x < 360
	if ( (_hipAngle - 2*pi > _kneeAngle) || (_kneeAngle > _hipAngle - minangle) )
    {
        _kneeAngle = 2*pi + _kneeAngle;
    }

    _desJointAngles << _scapAngle, _hipAngle, _kneeAngle;

    if( _error == -1 )
        _errorCount(_LEG) = _error;
    else
    {
        _currentPos = ForwardKinematic::getFootPosToScapOfLocal(_LEG, _desJointAngles);
        _errorCount(_LEG) = sqrt( dotProduct( _footPos - _currentPos, _footPos - _currentPos) );
    }
    
    return _desJointAngles;
}
#endif