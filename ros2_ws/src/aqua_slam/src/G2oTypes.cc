/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Converter.h"
#include "Pinhole.h"
#include "sophus/geometry.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/algorithm/string.hpp>
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
namespace ORB_SLAM3
{

ImuCamPose::ImuCamPose(KeyFrame *pKF):its(0)
{
    // Load IMU pose
    twb = Converter::toVector3d(pKF->GetImuPosition());
    Rwb = Converter::toMatrix3d(pKF->GetImuRotation());

    // Load camera poses
    int num_cams;
    if(pKF->mpCamera2)
        num_cams=2;
    else
        num_cams=1;

    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    tcb.resize(num_cams);
    Rcb.resize(num_cams);
    Rbc.resize(num_cams);
    tbc.resize(num_cams);
    pCamera.resize(num_cams);

    // Left camera
    tcw[0] = Converter::toVector3d(pKF->GetTranslation());
    Rcw[0] = Converter::toMatrix3d(pKF->GetRotation());
    tcb[0] = Converter::toVector3d(pKF->mImuCalib.Tcb.rowRange(0,3).col(3));
    Rcb[0] = Converter::toMatrix3d(pKF->mImuCalib.Tcb.rowRange(0,3).colRange(0,3));
    Rbc[0] = Rcb[0].transpose();
    tbc[0] = Converter::toVector3d(pKF->mImuCalib.Tbc.rowRange(0,3).col(3));
    pCamera[0] = pKF->mpCamera;
    bf = pKF->mbf;

    if(num_cams>1)
    {
        Eigen::Matrix4d Trl = Converter::toMatrix4d(pKF->mTrl);
        Rcw[1] = Trl.block<3,3>(0,0)*Rcw[0];
        tcw[1] = Trl.block<3,3>(0,0)*tcw[0]+Trl.block<3,1>(0,3);
        tcb[1] = Trl.block<3,3>(0,0)*tcb[0]+Trl.block<3,1>(0,3);
        Rcb[1] = Trl.block<3,3>(0,0)*Rcb[0];
        Rbc[1] = Rcb[1].transpose();
        tbc[1] = -Rbc[1]*tcb[1];
        pCamera[1] = pKF->mpCamera2;
    }

    // For posegraph 4DoF
    Rwb0 = Rwb;
    DR.setIdentity();
}

ImuCamPose::ImuCamPose(Frame *pF):its(0)
{
    // Load IMU pose
    twb = Converter::toVector3d(pF->GetImuPosition());
    Rwb = Converter::toMatrix3d(pF->GetImuRotation());

    // Load camera poses
    int num_cams;
    if(pF->mpCamera2)
        num_cams=2;
    else
        num_cams=1;

    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    tcb.resize(num_cams);
    Rcb.resize(num_cams);
    Rbc.resize(num_cams);
    tbc.resize(num_cams);
    pCamera.resize(num_cams);

    // Left camera
    tcw[0] = Converter::toVector3d(pF->mTcw.rowRange(0,3).col(3));
    Rcw[0] = Converter::toMatrix3d(pF->mTcw.rowRange(0,3).colRange(0,3));
    tcb[0] = Converter::toVector3d(pF->GetExtrinsicParamters().Tcb.rowRange(0,3).col(3));
    Rcb[0] = Converter::toMatrix3d(pF->GetExtrinsicParamters().Tcb.rowRange(0,3).colRange(0,3));
    Rbc[0] = Rcb[0].transpose();
    tbc[0] = Converter::toVector3d(pF->GetExtrinsicParamters().Tbc.rowRange(0,3).col(3));
    pCamera[0] = pF->mpCamera;
    bf = pF->mbf;

    if(num_cams>1)
    {
        Eigen::Matrix4d Trl = Converter::toMatrix4d(pF->mTrl);
        Rcw[1] = Trl.block<3,3>(0,0)*Rcw[0];
        tcw[1] = Trl.block<3,3>(0,0)*tcw[0]+Trl.block<3,1>(0,3);
        tcb[1] = Trl.block<3,3>(0,0)*tcb[0]+Trl.block<3,1>(0,3);
        Rcb[1] = Trl.block<3,3>(0,0)*Rcb[0];
        Rbc[1] = Rcb[1].transpose();
        tbc[1] = -Rbc[1]*tcb[1];
        pCamera[1] = pF->mpCamera2;
    }

    // For posegraph 4DoF
    Rwb0 = Rwb;
    DR.setIdentity();
}

ImuCamPose::ImuCamPose(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame* pKF): its(0)
{
    // This is only for posegrpah, we do not care about multicamera
    tcw.resize(1);
    Rcw.resize(1);
    tcb.resize(1);
    Rcb.resize(1);
    Rbc.resize(1);
    tbc.resize(1);
    pCamera.resize(1);

    tcb[0] = Converter::toVector3d(pKF->mImuCalib.Tcb.rowRange(0,3).col(3));
    Rcb[0] = Converter::toMatrix3d(pKF->mImuCalib.Tcb.rowRange(0,3).colRange(0,3));
    Rbc[0] = Rcb[0].transpose();
    tbc[0] = Converter::toVector3d(pKF->mImuCalib.Tbc.rowRange(0,3).col(3));
    twb = _Rwc*tcb[0]+_twc;
    Rwb = _Rwc*Rcb[0];
    Rcw[0] = _Rwc.transpose();
    tcw[0] = -Rcw[0]*_twc;
    pCamera[0] = pKF->mpCamera;
    bf = pKF->mbf;

    // For posegraph 4DoF
    Rwb0 = Rwb;
    DR.setIdentity();
}

void ImuCamPose::SetParam(const std::vector<Eigen::Matrix3d> &_Rcw, const std::vector<Eigen::Vector3d> &_tcw, const std::vector<Eigen::Matrix3d> &_Rbc,
              const std::vector<Eigen::Vector3d> &_tbc, const double &_bf)
{
    Rbc = _Rbc;
    tbc = _tbc;
    Rcw = _Rcw;
    tcw = _tcw;
    const int num_cams = Rbc.size();
    Rcb.resize(num_cams);
    tcb.resize(num_cams);

    for(int i=0; i<tcb.size(); i++)
    {
        Rcb[i] = Rbc[i].transpose();
        tcb[i] = -Rcb[i]*tbc[i];
    }
    Rwb = Rcw[0].transpose()*Rcb[0];
    twb = Rcw[0].transpose()*(tcb[0]-tcw[0]);

    bf = _bf;
}

Eigen::Vector2d ImuCamPose::Project(const Eigen::Vector3d &Xw, int cam_idx) const
{
    Eigen::Vector3d Xc = Rcw[cam_idx]*Xw+tcw[cam_idx];

    return pCamera[cam_idx]->project(Xc);
}

Eigen::Vector3d ImuCamPose::ProjectStereo(const Eigen::Vector3d &Xw, int cam_idx) const
{
    Eigen::Vector3d Pc = Rcw[cam_idx]*Xw+tcw[cam_idx];
    Eigen::Vector3d pc;
    double invZ = 1/Pc(2);
    pc.head(2) = pCamera[cam_idx]->project(Pc);
    pc(2) = pc(0) - bf*invZ;
    return pc;
}

bool ImuCamPose::isDepthPositive(const Eigen::Vector3d &Xw, int cam_idx) const
{
    return (Rcw[cam_idx].row(2)*Xw+tcw[cam_idx](2))>0.0;
}

void ImuCamPose::Update(const double *pu)
{
    Eigen::Vector3d ur, ut;
    ur << pu[0], pu[1], pu[2];
    ut << pu[3], pu[4], pu[5];

    // Update body pose
    twb += Rwb*ut;
    Rwb = Rwb*ExpSO3(ur);

    // Normalize rotation after 5 updates
    its++;
    if(its>=3)
    {
        NormalizeRotation(Rwb);
        its=0;
    }

    // Update camera poses
    const Eigen::Matrix3d Rbw = Rwb.transpose();
    const Eigen::Vector3d tbw = -Rbw*twb;

    for(int i=0; i<pCamera.size(); i++)
    {
        Rcw[i] = Rcb[i]*Rbw;
        tcw[i] = Rcb[i]*tbw+tcb[i];
    }

}

void ImuCamPose::UpdateW(const double *pu)
{
    Eigen::Vector3d ur, ut;
    ur << pu[0], pu[1], pu[2];
    ut << pu[3], pu[4], pu[5];


    const Eigen::Matrix3d dR = ExpSO3(ur);
    DR = dR*DR;
    Rwb = DR*Rwb0;
    // Update body pose
    twb += ut;

    // Normalize rotation after 5 updates
    its++;
    if(its>=5)
    {
        DR(0,2)=0.0;
        DR(1,2)=0.0;
        DR(2,0)=0.0;
        DR(2,1)=0.0;
        NormalizeRotation(DR);
        its=0;
    }

    // Update camera pose
    const Eigen::Matrix3d Rbw = Rwb.transpose();
    const Eigen::Vector3d tbw = -Rbw*twb;

    for(int i=0; i<pCamera.size(); i++)
    {
        Rcw[i] = Rcb[i]*Rbw;
        tcw[i] = Rcb[i]*tbw+tcb[i];
    }
}

InvDepthPoint::InvDepthPoint(double _rho, double _u, double _v, KeyFrame* pHostKF): u(_u), v(_v), rho(_rho),
    fx(pHostKF->fx), fy(pHostKF->fy), cx(pHostKF->cx), cy(pHostKF->cy), bf(pHostKF->mbf)
{
}

void InvDepthPoint::Update(const double *pu)
{
    rho += *pu;
}


bool VertexPose::read(std::istream& is)
{
    std::vector<Eigen::Matrix<double,3,3> > Rcw;
    std::vector<Eigen::Matrix<double,3,1> > tcw;
    std::vector<Eigen::Matrix<double,3,3> > Rbc;
    std::vector<Eigen::Matrix<double,3,1> > tbc;

    const int num_cams = _estimate.Rbc.size();
    for(int idx = 0; idx<num_cams; idx++)
    {
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                is >> Rcw[idx](i,j);
        }
        for (int i=0; i<3; i++){
            is >> tcw[idx](i);
        }

        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                is >> Rbc[idx](i,j);
        }
        for (int i=0; i<3; i++){
            is >> tbc[idx](i);
        }

        float nextParam;
        for(size_t i = 0; i < _estimate.pCamera[idx]->size(); i++){
            is >> nextParam;
            _estimate.pCamera[idx]->setParameter(nextParam,i);
        }
    }

    double bf;
    is >> bf;
    _estimate.SetParam(Rcw,tcw,Rbc,tbc,bf);
    updateCache();
    
    return true;
}

bool VertexPose::write(std::ostream& os) const
{
    std::vector<Eigen::Matrix<double,3,3> > Rcw = _estimate.Rcw;
    std::vector<Eigen::Matrix<double,3,1> > tcw = _estimate.tcw;

    std::vector<Eigen::Matrix<double,3,3> > Rbc = _estimate.Rbc;
    std::vector<Eigen::Matrix<double,3,1> > tbc = _estimate.tbc;

    const int num_cams = tcw.size();

    for(int idx = 0; idx<num_cams; idx++)
    {
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                os << Rcw[idx](i,j) << " ";
        }
        for (int i=0; i<3; i++){
            os << tcw[idx](i) << " ";
        }

        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++)
                os << Rbc[idx](i,j) << " ";
        }
        for (int i=0; i<3; i++){
            os << tbc[idx](i) << " ";
        }

        for(size_t i = 0; i < _estimate.pCamera[idx]->size(); i++){
            os << _estimate.pCamera[idx]->getParameter(i) << " ";
        }
    }

    os << _estimate.bf << " ";

    return os.good();
}


void EdgeMono::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];

    const Eigen::Matrix<double,2,3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv; // TODO optimize this product
}

void EdgeMonoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];

    Eigen::Matrix<double,2,3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv; // symbol different becasue of update mode
}

void EdgeStereo::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0/(Xc(2)*Xc(2));

    Eigen::Matrix<double,3,3> proj_jac;
    proj_jac.block<2,3>(0,0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    proj_jac.block<1,3>(2,0) = proj_jac.block<1,3>(0,0);
    proj_jac(2,2) += bf*inv_z2;

    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv;


    /*const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw;
    const Eigen::Vector3d &tcw = VPose->estimate().tcw;
    const Eigen::Vector3d Xc = Rcw*VPoint->estimate() + tcw;
    const double &xc = Xc[0];
    const double &yc = Xc[1];
    const double invzc = 1.0/Xc[2];
    const double invzc2 = invzc*invzc;
    const double &fx = VPose->estimate().fx;
    const double &fy = VPose->estimate().fy;
    const double &bf = VPose->estimate().bf;
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb;

    // Jacobian wrt Point
    _jacobianOplusXi(0,0) = -fx*invzc*Rcw(0,0)+fx*xc*invzc2*Rcw(2,0);
    _jacobianOplusXi(0,1) = -fx*invzc*Rcw(0,1)+fx*xc*invzc2*Rcw(2,1);
    _jacobianOplusXi(0,2) = -fx*invzc*Rcw(0,2)+fx*xc*invzc2*Rcw(2,2);

    _jacobianOplusXi(1,0) = -fy*invzc*Rcw(1,0)+fy*yc*invzc2*Rcw(2,0);
    _jacobianOplusXi(1,1) = -fy*invzc*Rcw(1,1)+fy*yc*invzc2*Rcw(2,1);
    _jacobianOplusXi(1,2) = -fy*invzc*Rcw(1,2)+fy*yc*invzc2*Rcw(2,2);

    _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*invzc2*Rcw(2,0);
    _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*invzc2*Rcw(2,1);
    _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*invzc2*Rcw(2,2);

    const Eigen::Vector3d Xb = VPose->estimate().Rbc*Xc + VPose->estimate().tbc;
    const Eigen::Matrix3d RS = VPose->estimate().Rcb*Skew(Xb);

    // Jacobian wrt Imu Pose
    _jacobianOplusXj(0,0) = -fx*invzc*RS(0,0)+fx*xc*invzc2*RS(2,0);
    _jacobianOplusXj(0,1) = -fx*invzc*RS(0,1)+fx*xc*invzc2*RS(2,1);
    _jacobianOplusXj(0,2) = -fx*invzc*RS(0,2)+fx*xc*invzc2*RS(2,2);
    _jacobianOplusXj(0,3) = fx*invzc*Rcb(0,0)-fx*xc*invzc2*Rcb(2,0);
    _jacobianOplusXj(0,4) = fx*invzc*Rcb(0,1)-fx*xc*invzc2*Rcb(2,1);
    _jacobianOplusXj(0,5) = fx*invzc*Rcb(0,2)-fx*xc*invzc2*Rcb(2,2);

    _jacobianOplusXj(1,0) = -fy*invzc*RS(1,0)+fy*yc*invzc2*RS(2,0);
    _jacobianOplusXj(1,1) = -fy*invzc*RS(1,1)+fy*yc*invzc2*RS(2,1);
    _jacobianOplusXj(1,2) = -fy*invzc*RS(1,2)+fy*yc*invzc2*RS(2,2);
    _jacobianOplusXj(1,3) = fy*invzc*Rcb(1,0)-fy*yc*invzc2*Rcb(2,0);
    _jacobianOplusXj(1,4) = fy*invzc*Rcb(1,1)-fy*yc*invzc2*Rcb(2,1);
    _jacobianOplusXj(1,5) = fy*invzc*Rcb(1,2)-fy*yc*invzc2*Rcb(2,2);

    _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0) - bf*invzc2*RS(2,0);
    _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1) - bf*invzc2*RS(2,1);
    _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2) - bf*invzc2*RS(2,2);
    _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3) + bf*invzc2*Rcb(2,0);
    _jacobianOplusXj(2,4) = _jacobianOplusXj(0,4) + bf*invzc2*Rcb(2,1);
    _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5) + bf*invzc2*Rcb(2,2);
    */
}

void EdgeStereoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx]*Xc+VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0/(Xc(2)*Xc(2));

    Eigen::Matrix<double,3,3> proj_jac;
    proj_jac.block<2,3>(0,0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    proj_jac.block<1,3>(2,0) = proj_jac.block<1,3>(0,0);
    proj_jac(2,2) += bf*inv_z2;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv;

    /*const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw;
    const Eigen::Vector3d &tcw = VPose->estimate().tcw;
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;
    const double &xc = Xc[0];
    const double &yc = Xc[1];
    const double invzc = 1.0/Xc[2];
    const double invzc2 = invzc*invzc;
    const double &fx = VPose->estimate().fx;
    const double &fy = VPose->estimate().fy;
    const double &bf = VPose->estimate().bf;
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb;

    const Eigen::Vector3d Xb = VPose->estimate().Rbc*Xc + VPose->estimate().tbc;
    const Eigen::Matrix3d RS = VPose->estimate().Rcb*Skew(Xb);

    // Jacobian wrt Imu Pose
    _jacobianOplusXi(0,0) = -fx*invzc*RS(0,0)+fx*xc*invzc2*RS(2,0);
    _jacobianOplusXi(0,1) = -fx*invzc*RS(0,1)+fx*xc*invzc2*RS(2,1);
    _jacobianOplusXi(0,2) = -fx*invzc*RS(0,2)+fx*xc*invzc2*RS(2,2);
    _jacobianOplusXi(0,3) = fx*invzc*Rcb(0,0)-fx*xc*invzc2*Rcb(2,0);
    _jacobianOplusXi(0,4) = fx*invzc*Rcb(0,1)-fx*xc*invzc2*Rcb(2,1);
    _jacobianOplusXi(0,5) = fx*invzc*Rcb(0,2)-fx*xc*invzc2*Rcb(2,2);

    _jacobianOplusXi(1,0) = -fy*invzc*RS(1,0)+fy*yc*invzc2*RS(2,0);
    _jacobianOplusXi(1,1) = -fy*invzc*RS(1,1)+fy*yc*invzc2*RS(2,1);
    _jacobianOplusXi(1,2) = -fy*invzc*RS(1,2)+fy*yc*invzc2*RS(2,2);
    _jacobianOplusXi(1,3) = fy*invzc*Rcb(1,0)-fy*yc*invzc2*Rcb(2,0);
    _jacobianOplusXi(1,4) = fy*invzc*Rcb(1,1)-fy*yc*invzc2*Rcb(2,1);
    _jacobianOplusXi(1,5) = fy*invzc*Rcb(1,2)-fy*yc*invzc2*Rcb(2,2);

    _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0) - bf*invzc2*RS(2,0);
    _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1) - bf*invzc2*RS(2,1);
    _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2) - bf*invzc2*RS(2,2);
    _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3) + bf*invzc2*Rcb(2,0);
    _jacobianOplusXi(2,4) = _jacobianOplusXi(0,4) + bf*invzc2*Rcb(2,1);
    _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5) + bf*invzc2*Rcb(2,2);
    */
}

/*Eigen::Vector2d EdgeMonoInvdepth::cam_project(const Eigen::Vector3d & trans_xyz) const{
  double invZ = 1./trans_xyz[2];
  Eigen::Vector2d res;
  res[0] = invZ*trans_xyz[0]*fx + cx;
  res[1] = invZ*trans_xyz[1]*fy + cy;
  return res;
}

Eigen::Vector3d EdgeMonoInvdepth::cam_unproject(const double u, const double v, const double invDepth) const{
  Eigen::Vector3d res;
  res[2] = 1./invDepth;
  double z_x=res[2]/fx;
  double z_y=res[2]/fy;
  res[0] = (u-cx)*z_x;
  res[1] = (v-cy)*z_y;

  return res;
}

void EdgeMonoInvdepth::linearizeOplus()
{
    VertexInvDepth *vPt = static_cast<VertexInvDepth*>(_vertices[0]);
    g2o::VertexSE3Expmap *vHost = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
    g2o::VertexSE3Expmap *vObs = static_cast<g2o::VertexSE3Expmap*>(_vertices[2]);

    //
    g2o::SE3Quat Tiw(vObs->estimate());
    g2o::SE3Quat T0w(vHost->estimate());
    g2o::SE3Quat Ti0 = Tiw*T0w.inverse();
    double o_rho_j = vPt->estimate().rho;
    Eigen::Vector3d o_X_j = cam_unproject(vPt->estimate().u,vPt->estimate().v,o_rho_j);
    Eigen::Vector3d i_X_j = Ti0.map(o_X_j);
    double i_rho_j = 1./i_X_j[2];
    Eigen::Vector2d i_proj_j = cam_project(i_X_j);

    // i_rho_j*C_ij matrix
    Eigen::Matrix<double,2,3> rhoC_ij;
    rhoC_ij(0,0) = i_rho_j*fx;
    rhoC_ij(0,1) = 0.0;
    rhoC_ij(0,2) = i_rho_j*(cx-i_proj_j[0]);
    rhoC_ij(1,0) = 0.0;
    rhoC_ij(1,1) = i_rho_j*fy;
    rhoC_ij(1,2) = i_rho_j*(cy-i_proj_j[1]);

    // o_rho_j^{-2}*K^{-1}*0_proj_j vector
    Eigen::Matrix3d Ri0 = Ti0.rotation().toRotationMatrix();
    Eigen::Matrix<double, 2, 3> tmp;
    tmp = rhoC_ij*Ri0;

    // Skew matrices
    Eigen::Matrix3d skew_0_X_j;
    Eigen::Matrix3d skew_i_X_j;

    skew_0_X_j=Eigen::MatrixXd::Zero(3,3);
    skew_i_X_j=Eigen::MatrixXd::Zero(3,3);

    skew_0_X_j(0,1) = -o_X_j[2];
    skew_0_X_j(1,0) = -skew_0_X_j(0,1);
    skew_0_X_j(0,2) = o_X_j[1];
    skew_0_X_j(2,0) = -skew_0_X_j(0,2);
    skew_0_X_j(1,2) = -o_X_j[0];
    skew_0_X_j(2,1) = -skew_0_X_j(1,2);

    skew_i_X_j(0,1) = -i_X_j[2];
    skew_i_X_j(1,0) = -skew_i_X_j(0,1);
    skew_i_X_j(0,2) = i_X_j[1];
    skew_i_X_j(2,0) = -skew_i_X_j(0,2);
    skew_i_X_j(1,2) = -i_X_j[0];
    skew_i_X_j(2,1) = -skew_i_X_j(1,2);

    // Jacobians w.r.t inverse depth in the host frame
    _jacobianOplus[0].setZero();
    _jacobianOplus[0] = (1./o_rho_j)*rhoC_ij*Ri0*o_X_j;

    // Jacobians w.r.t pose host frame
    _jacobianOplus[1].setZero();
    // Rotation
    _jacobianOplus[1].block<2,3>(0,0) = -tmp*skew_0_X_j;
    // translation
    _jacobianOplus[1].block<2,3>(0,3) = tmp;

    // Jacobians w.r.t pose observer frame
    _jacobianOplus[2].setZero();
    // Rotation
    _jacobianOplus[2].block<2,3>(0,0) = rhoC_ij*skew_i_X_j;
    // translation
    _jacobianOplus[2].block<2,3>(0,3) = -rhoC_ij;
}


Eigen::Vector2d EdgeMonoInvdepthBody::cam_project(const Eigen::Vector3d & trans_xyz) const{
  double invZ = 1./trans_xyz[2];
  Eigen::Vector2d res;
  res[0] = invZ*trans_xyz[0]*fx + cx;
  res[1] = invZ*trans_xyz[1]*fy + cy;
  return res;
}

Eigen::Vector3d EdgeMonoInvdepthBody::cam_unproject(const double u, const double v, const double invDepth) const{
  Eigen::Vector3d res;
  res[2] = 1./invDepth;
  double z_x=res[2]/fx;
  double z_y=res[2]/fy;
  res[0] = (u-cx)*z_x;
  res[1] = (v-cy)*z_y;

  return res;
}*/

VertexVelocity::VertexVelocity(KeyFrame* pKF)
{
    Eigen::Vector3d v_d;
    pKF->GetDvlVelocity(v_d);
    setEstimate(v_d);
}

VertexVelocity::VertexVelocity(Frame* pF)
{
    setEstimate(Converter::toVector3d(pF->mVw));
}

bool VertexVelocity::read(istream &is)
{
    Eigen::Vector3d v;
    is >> v[0] >> v[1] >> v[2];
    setEstimate(v);
    return true;
}

bool VertexVelocity::write(ostream &os) const
{
    Eigen::Vector3d v = estimate();
    os << v[0] << " " << v[1] << " " << v[2];
    return true;
}

    VertexGyroBias::VertexGyroBias(KeyFrame *pKF)
{
    setEstimate(Converter::toVector3d(pKF->GetGyroBias()));
}

VertexGyroBias::VertexGyroBias(Frame *pF)
{
    Eigen::Vector3d bg;
    bg << pF->mImuBias.bwx, pF->mImuBias.bwy,pF->mImuBias.bwz;
    setEstimate(bg);
}

bool VertexGyroBias::read(istream &is)
{
    Eigen::Vector3d b;
    is >> b[0] >> b[1] >> b[2];
    setEstimate(b);
    return true;
}

bool VertexGyroBias::write(ostream &os) const
{
    Eigen::Vector3d b = estimate();
    os << b[0] << " " << b[1] << " " << b[2];
    return true;
}

VertexAccBias::VertexAccBias(KeyFrame *pKF)
{
    setEstimate(Converter::toVector3d(pKF->GetAccBias()));
}

VertexAccBias::VertexAccBias(Frame *pF)
{
    Eigen::Vector3d ba;
    ba << pF->mImuBias.bax, pF->mImuBias.bay,pF->mImuBias.baz;
    setEstimate(ba);
}

bool VertexAccBias::read(istream &is)
{
    Eigen::Vector3d b;
    is >> b[0] >> b[1] >> b[2];
    setEstimate(b);
    return true;
}

bool VertexAccBias::write(ostream &os) const
{
    Eigen::Vector3d b = estimate();
    os << b[0] << " " << b[1] << " " << b[2];
    return true;
}


    EdgeInertial::EdgeInertial(IMU::Preintegrated *pInt):JRg(Converter::toMatrix3d(pInt->JRg)),
    JVg(Converter::toMatrix3d(pInt->JVg)), JPg(Converter::toMatrix3d(pInt->JPg)), JVa(Converter::toMatrix3d(pInt->JVa)),
    JPa(Converter::toMatrix3d(pInt->JPa)), mpInt(pInt), dt(pInt->dT)
{
    // This edge links 6 vertices
    resize(6);
    g << 0, 0, -IMU::GRAVITY_VALUE;
    cv::Mat cvInfo = pInt->C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
    Matrix9d Info;
    for(int r=0;r<9;r++)
        for(int c=0;c<9;c++)
            Info(r,c)=cvInfo.at<float>(r,c);
    Info = (Info+Info.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9> > es(Info);
     Eigen::Matrix<double,9,1> eigs = es.eigenvalues();
     for(int i=0;i<9;i++)
         if(eigs[i]<1e-12)
             eigs[i]=0;
    Info = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    setInformation(Info);
}




void EdgeInertial::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);
    const Eigen::Matrix3d dR = Converter::toMatrix3d(mpInt->GetDeltaRotation(b1));
    const Eigen::Vector3d dV = Converter::toVector3d(mpInt->GetDeltaVelocity(b1));
    const Eigen::Vector3d dP = Converter::toVector3d(mpInt->GetDeltaPosition(b1));

    const Eigen::Vector3d er = LogSO3(dR.transpose()*VP1->estimate().Rwb.transpose()*VP2->estimate().Rwb);
    const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose()*(VV2->estimate() - VV1->estimate() - g*dt) - dV;
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose()*(VP2->estimate().twb - VP1->estimate().twb
                                                               - VV1->estimate()*dt - g*dt*dt/2) - dP;

    _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus()
{
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2= static_cast<const VertexVelocity*>(_vertices[5]);
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);
    const IMU::Bias db = mpInt->GetDeltaBias(b1);
    Eigen::Vector3d dbg;
    dbg << db.bwx, db.bwy, db.bwz;

    const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;

    const Eigen::Matrix3d dR = Converter::toMatrix3d(mpInt->GetDeltaRotation(b1));
    const Eigen::Matrix3d eR = dR.transpose()*Rbw1*Rwb2;
    const Eigen::Vector3d er = LogSO3(eR);
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
     // rotation
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1; // OK
    _jacobianOplus[0].block<3,3>(3,0) = Skew(Rbw1*(VV2->estimate() - VV1->estimate() - g*dt)); // OK
    _jacobianOplus[0].block<3,3>(6,0) = Skew(Rbw1*(VP2->estimate().twb - VP1->estimate().twb
                                                   - VV1->estimate()*dt - 0.5*g*dt*dt)); // OK
    // translation
    _jacobianOplus[0].block<3,3>(6,3) = -Eigen::Matrix3d::Identity(); // OK

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(3,0) = -Rbw1; // OK
    _jacobianOplus[1].block<3,3>(6,0) = -Rbw1*dt; // OK

    // Jacobians wrt Gyro 1
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(0,0) = -invJr*eR.transpose()*RightJacobianSO3(JRg*dbg)*JRg; // OK
    _jacobianOplus[2].block<3,3>(3,0) = -JVg; // OK
    _jacobianOplus[2].block<3,3>(6,0) = -JPg; // OK

    // Jacobians wrt Accelerometer 1
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(3,0) = -JVa; // OK
    _jacobianOplus[3].block<3,3>(6,0) = -JPa; // OK

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3,3>(0,0) = invJr; // OK
    // translation
    _jacobianOplus[4].block<3,3>(6,3) = Rbw1*Rwb2; // OK

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3,3>(3,0) = Rbw1; // OK
}

EdgeInertialGS::EdgeInertialGS(IMU::Preintegrated *pInt):JRg(Converter::toMatrix3d(pInt->JRg)),
    JVg(Converter::toMatrix3d(pInt->JVg)), JPg(Converter::toMatrix3d(pInt->JPg)), JVa(Converter::toMatrix3d(pInt->JVa)),
    JPa(Converter::toMatrix3d(pInt->JPa)), mpInt(pInt), dt(pInt->dT)
{
    // This edge links 8 vertices
    resize(8);
    gI << 0, 0, -IMU::GRAVITY_VALUE;
    cv::Mat cvInfo = pInt->C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
    Matrix9d Info;
    for(int r=0;r<9;r++)
        for(int c=0;c<9;c++)
            Info(r,c)=cvInfo.at<float>(r,c);
    Info = (Info+Info.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9> > es(Info);
     Eigen::Matrix<double,9,1> eigs = es.eigenvalues();
     for(int i=0;i<9;i++)
         if(eigs[i]<1e-12)
             eigs[i]=0;
    Info = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    setInformation(Info);
}



void EdgeInertialGS::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
    const IMU::Bias b(VA->estimate()[0],VA->estimate()[1],VA->estimate()[2],VG->estimate()[0],VG->estimate()[1],VG->estimate()[2]);
    g = VGDir->estimate().Rwg*gI;
    const double s = VS->estimate();
    const Eigen::Matrix3d dR = Converter::toMatrix3d(mpInt->GetDeltaRotation(b));
    const Eigen::Vector3d dV = Converter::toVector3d(mpInt->GetDeltaVelocity(b));
    const Eigen::Vector3d dP = Converter::toVector3d(mpInt->GetDeltaPosition(b));

    const Eigen::Vector3d er = LogSO3(dR.transpose()*VP1->estimate().Rwb.transpose()*VP2->estimate().Rwb);
    const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose()*(s*(VV2->estimate() - VV1->estimate()) - g*dt) - dV;
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose()*(s*(VP2->estimate().twb - VP1->estimate().twb - VV1->estimate()*dt) - g*dt*dt/2) - dP;

    _error << er, ev, ep;
}

void EdgeInertialGS::linearizeOplus()
{
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGDir* VGDir = static_cast<const VertexGDir*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
    const IMU::Bias b(VA->estimate()[0],VA->estimate()[1],VA->estimate()[2],VG->estimate()[0],VG->estimate()[1],VG->estimate()[2]);
    const IMU::Bias db = mpInt->GetDeltaBias(b);

    Eigen::Vector3d dbg;
    dbg << db.bwx, db.bwy, db.bwz;

    const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;
    const Eigen::Matrix3d Rwg = VGDir->estimate().Rwg;
    Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3,2);
    Gm(0,1) = -IMU::GRAVITY_VALUE;
    Gm(1,0) = IMU::GRAVITY_VALUE;
    const double s = VS->estimate();
    const Eigen::MatrixXd dGdTheta = Rwg*Gm;
    const Eigen::Matrix3d dR = Converter::toMatrix3d(mpInt->GetDeltaRotation(b));
    const Eigen::Matrix3d eR = dR.transpose()*Rbw1*Rwb2;
    const Eigen::Vector3d er = LogSO3(eR);
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
     // rotation
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1;
    _jacobianOplus[0].block<3,3>(3,0) = Skew(Rbw1*(s*(VV2->estimate() - VV1->estimate()) - g*dt));
    _jacobianOplus[0].block<3,3>(6,0) = Skew(Rbw1*(s*(VP2->estimate().twb - VP1->estimate().twb
                                                   - VV1->estimate()*dt) - 0.5*g*dt*dt));
    // translation
    _jacobianOplus[0].block<3,3>(6,3) = -s*Eigen::Matrix3d::Identity();

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(3,0) = -s*Rbw1;
    _jacobianOplus[1].block<3,3>(6,0) = -s*Rbw1*dt;

    // Jacobians wrt Gyro bias
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(0,0) = -invJr*eR.transpose()*RightJacobianSO3(JRg*dbg)*JRg;
    _jacobianOplus[2].block<3,3>(3,0) = -JVg;
    _jacobianOplus[2].block<3,3>(6,0) = -JPg;

    // Jacobians wrt Accelerometer bias
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(3,0) = -JVa;
    _jacobianOplus[3].block<3,3>(6,0) = -JPa;

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3,3>(0,0) = invJr;
    // translation
    _jacobianOplus[4].block<3,3>(6,3) = s*Rbw1*Rwb2;

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3,3>(3,0) = s*Rbw1;

    // Jacobians wrt Gravity direction
    _jacobianOplus[6].setZero();
    _jacobianOplus[6].block<3,2>(3,0) = -Rbw1*dGdTheta*dt;
    _jacobianOplus[6].block<3,2>(6,0) = -0.5*Rbw1*dGdTheta*dt*dt;

    // Jacobians wrt scale factor
    _jacobianOplus[7].setZero();
    _jacobianOplus[7].block<3,1>(3,0) = Rbw1*(VV2->estimate()-VV1->estimate());
    _jacobianOplus[7].block<3,1>(6,0) = Rbw1*(VP2->estimate().twb-VP1->estimate().twb-VV1->estimate()*dt);
}

EdgePriorPoseImu::EdgePriorPoseImu(ConstraintPoseImu *c)
{
    resize(4);
    Rwb = c->Rwb;
    twb = c->twb;
    vwb = c->vwb;
    bg = c->bg;
    ba = c->ba;
    setInformation(c->H);
}

void EdgePriorPoseImu::computeError()
{
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV = static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);

    const Eigen::Vector3d er = LogSO3(Rwb.transpose()*VP->estimate().Rwb);
    const Eigen::Vector3d et = Rwb.transpose()*(VP->estimate().twb-twb);
    const Eigen::Vector3d ev = VV->estimate() - vwb;
    const Eigen::Vector3d ebg = VG->estimate() - bg;
    const Eigen::Vector3d eba = VA->estimate() - ba;

    _error << er, et, ev, ebg, eba;
}

void EdgePriorPoseImu::linearizeOplus()
{
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d er = LogSO3(Rwb.transpose()*VP->estimate().Rwb);
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3,3>(0,0) = InverseRightJacobianSO3(er);
    _jacobianOplus[0].block<3,3>(3,3) = Rwb.transpose()*VP->estimate().Rwb;
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(6,0) = Eigen::Matrix3d::Identity();
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(9,0) = Eigen::Matrix3d::Identity();
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(12,0) = Eigen::Matrix3d::Identity();
}

void EdgePriorAcc::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

}

bool EdgePriorAcc::read(istream &is)
{
    Eigen::Vector3d prior;
    is >> prior[0] >> prior[1] >> prior[2];
    bprior = prior;
    Eigen::Matrix3d info;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
        {
            is >> info(i,j);
        }
    setInformation(info);
    return true;
}

bool EdgePriorAcc::write(ostream &os) const
{
    Eigen::Vector3d prior = bprior;
    os << prior[0] << " " << prior[1] << " " << prior[2] << " ";
    Eigen::Matrix3d info = information();
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
        {
            os << info(i,j) << " ";
        }
    return true;
}

void EdgePriorGyro::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

}

bool EdgePriorGyro::read(istream &is)
{
    Eigen::Vector3d prior;
    is >> prior[0] >> prior[1] >> prior[2];
    bprior = prior;
    Eigen::Matrix3d info;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
        {
            is >> info(i,j);
        }
    setInformation(info);
    return true;
}

bool EdgePriorGyro::write(ostream &os) const
{
    Eigen::Vector3d prior = bprior;
    os << prior[0] << " " << prior[1] << " " << prior[2] << " ";
    Eigen::Matrix3d info = information();
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
        {
            os << info(i,j) << " ";
        }
    return true;
}

// SO3 FUNCTIONS
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w)
{
    return ExpSO3(w[0],w[1],w[2]);
}

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W +0.5*W*W;
        return Converter::toMatrix3d(IMU::NormalizeRotation(Converter::toCvMat(res)));
    }
    else
    {
        Eigen::Matrix3d res =Eigen::Matrix3d::Identity() + W*sin(d)/d + W*W*(1.0-cos(d))/d2;
        return Converter::toMatrix3d(IMU::NormalizeRotation(Converter::toCvMat(res)));
    }
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
{
    const double tr = R(0,0)+R(1,1)+R(2,2);
    Eigen::Vector3d w;
    w << (R(2,1)-R(1,2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
    const double costheta = (tr-1.0)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const double theta = acos(costheta);
    const double s = sin(theta);
    if(fabs(s)<1e-5)
        return w;
    else
        return theta*w/s;
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v)
{
    return InverseRightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
        return Eigen::Matrix3d::Identity();
    else
        return Eigen::Matrix3d::Identity() + W/2 + W*W*(1.0/d2 - (1.0+cos(d))/(2.0*d*sin(d)));
}

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v)
{
    return RightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        return Eigen::Matrix3d::Identity() - W*(1.0-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Eigen::Matrix3d Skew(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d W;
    W << 0.0, -w[2], w[1],w[2], 0.0, -w[0],-w[1],  w[0], 0.0;
    return W;
}

Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d &R)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R,Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU()*svd.matrixV();
}

DvlImuCamPose::DvlImuCamPose(Frame *pF)
{
    mTimestamp = pF->mTimeStamp;
    mPoorVision = pF->mPoorVision;
	// Load Gyro rotation and Dvl translation
//	t_c0_gyro = Converter::toVector3d(pF->GetDvlPosition());
//	R_c0_gyro = Converter::toMatrix3d(pF->GetGyroRotation());
	Eigen::Isometry3d T_w_c = Eigen::Isometry3d::Identity();
	cv::cv2eigen(pF->mTcw,T_w_c.matrix());
	T_w_c = T_w_c.inverse();
	Rwc = T_w_c.rotation();
	twc = T_w_c.translation();
//	Rwc = Converter::toMatrix3d(pF->mRwc.t());
//	twc = - Rwc * Converter::toVector3d(pF->mTcw.rowRange(0,3).col(3));


	// Load camera poses
	int num_cams;
	if(pF->mpCamera2)
		num_cams=2;
	else
		num_cams=1;

	tcw.resize(num_cams);
	Rcw.resize(num_cams);
	t_c_gyro.resize(num_cams);
	R_c_gyro.resize(num_cams);
	t_gyro_c.resize(num_cams);
	R_gyro_c.resize(num_cams);
	t_dvl_c.resize(num_cams);
	R_dvl_c.resize(num_cams);
	t_c_dvl.resize(num_cams);
	R_c_dvl.resize(num_cams);
	pCamera.resize(num_cams);

	// Left camera
	tcw[0] = Converter::toVector3d(pF->mTcw.rowRange(0,3).col(3));
	Rcw[0] = Converter::toMatrix3d(pF->mTcw.rowRange(0,3).colRange(0,3));
	t_c_gyro[0] = Converter::toVector3d(pF->GetExtrinsicParamters().mT_c_gyro.rowRange(0,3).col(3));
	R_c_gyro[0] = Converter::toMatrix3d(pF->GetExtrinsicParamters().mT_c_gyro.rowRange(0, 3).colRange(0, 3));
	t_gyro_c[0] = Converter::toVector3d(pF->GetExtrinsicParamters().mT_gyro_c.rowRange(0,3).col(3));
	R_gyro_c[0] = Converter::toMatrix3d(pF->GetExtrinsicParamters().mT_gyro_c.rowRange(0, 3).colRange(0, 3));
	t_c_dvl[0] = Converter::toVector3d(pF->GetExtrinsicParamters().mT_c_dvl.rowRange(0,3).col(3));
	R_c_dvl[0] = Converter::toMatrix3d(pF->GetExtrinsicParamters().mT_c_dvl.rowRange(0, 3).colRange(0, 3));
	t_dvl_c[0] = Converter::toVector3d(pF->GetExtrinsicParamters().mT_dvl_c.rowRange(0,3).col(3));
	R_dvl_c[0] = Converter::toMatrix3d(pF->GetExtrinsicParamters().mT_dvl_c.rowRange(0, 3).colRange(0, 3));
	pCamera[0] = pF->mpCamera;
	bf = pF->mbf;

	if(num_cams>1)
	{
		Eigen::Matrix4d Trl = Converter::toMatrix4d(pF->mTrl);
		T_r_l.matrix() = Trl;
		Rcw[1] = Trl.block<3,3>(0,0)*Rcw[0];
		tcw[1] = Trl.block<3,3>(0,0)*tcw[0]+Trl.block<3,1>(0,3);

		t_c_gyro[1] = Trl.block<3,3>(0,0)*t_c_gyro[0]+Trl.block<3,1>(0,3);
		R_c_gyro[1] = Trl.block<3,3>(0,0) * R_c_gyro[0];
		R_gyro_c[1] = R_c_gyro[1].transpose();
		t_gyro_c[1] = -R_gyro_c[1]*t_c_gyro[1];

		t_c_dvl[1] = Trl.block<3,3>(0,0)*t_c_dvl[1]+Trl.block<3,1>(0,3);
		R_c_dvl[1] = Trl.block<3,3>(0,0) * R_c_dvl[0];
		R_dvl_c[1] = R_c_dvl[1].transpose();
		t_dvl_c[1] = -R_dvl_c[1]*t_c_dvl[1];

		pCamera[1] = pF->mpCamera2;
	}

	// For posegraph 4DoF
//	Rwb0 = R_c0_gyro;
//	DR.setIdentity();
}

DvlImuCamPose::DvlImuCamPose(KeyFrame *pKF)
{
    mTimestamp = pKF->mTimeStamp;
    mPoorVision = pKF->mPoorVision;
	// Load camera poses
	int num_cams;
	if(pKF->mpCamera2)
		num_cams=2;
	else
		num_cams=1;
    mCamNum = num_cams;

	tcw.resize(num_cams);
	Rcw.resize(num_cams);
	t_c_gyro.resize(num_cams);
	R_c_gyro.resize(num_cams);
	t_gyro_c.resize(num_cams);
	R_gyro_c.resize(num_cams);
	t_dvl_c.resize(num_cams);
	R_dvl_c.resize(num_cams);
	t_c_dvl.resize(num_cams);
	R_c_dvl.resize(num_cams);
	pCamera.resize(num_cams);

	// Left camera
	tcw[0] = Converter::toVector3d(pKF->GetTranslation());
	Rcw[0] = Converter::toMatrix3d(pKF->GetRotation());

	Rwc = Rcw[0].transpose();
	twc = - Rwc * tcw[0];

	t_c_gyro[0] = Converter::toVector3d(pKF->mImuCalib.mT_c_gyro.rowRange(0,3).col(3));
	R_c_gyro[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_c_gyro.rowRange(0, 3).colRange(0, 3));
	t_gyro_c[0] = Converter::toVector3d(pKF->mImuCalib.mT_gyro_c.rowRange(0,3).col(3));
	R_gyro_c[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_gyro_c.rowRange(0, 3).colRange(0, 3));

	t_c_dvl[0] = Converter::toVector3d(pKF->mImuCalib.mT_c_dvl.rowRange(0,3).col(3));
	R_c_dvl[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_c_dvl.rowRange(0, 3).colRange(0, 3));
	t_dvl_c[0] = Converter::toVector3d(pKF->mImuCalib.mT_dvl_c.rowRange(0,3).col(3));
	R_dvl_c[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_dvl_c.rowRange(0, 3).colRange(0, 3));

	pCamera[0] = pKF->mpCamera;
	bf = pKF->mbf;

	if(num_cams>1)
	{
		Eigen::Matrix4d Trl = Converter::toMatrix4d(pKF->mTrl);
		T_r_l.matrix() = Trl;
		Rcw[1] = Trl.block<3,3>(0,0)*Rcw[0];
		tcw[1] = Trl.block<3,3>(0,0)*tcw[0]+Trl.block<3,1>(0,3);

		t_c_gyro[1] = Trl.block<3,3>(0,0)*t_c_gyro[0]+Trl.block<3,1>(0,3);
		R_c_gyro[1] = Trl.block<3,3>(0,0) * R_c_gyro[0];
		R_gyro_c[1] = R_c_gyro[1].transpose();
		t_gyro_c[1] = -R_gyro_c[1]*t_c_gyro[1];

		t_c_dvl[1] = Trl.block<3,3>(0,0)*t_c_dvl[1]+Trl.block<3,1>(0,3);
		R_c_dvl[1] = Trl.block<3,3>(0,0) * R_c_dvl[0];
		R_dvl_c[1] = R_c_dvl[1].transpose();
		t_dvl_c[1] = -R_dvl_c[1]*t_c_dvl[1];
		pCamera[1] = pKF->mpCamera2;
	}
}

void DvlImuCamPose::Update(const double *pu)
{
//	Eigen::Vector3d ur, ut;
//	ur << pu[0], pu[1], pu[2];
//	ut << pu[3], pu[4], pu[5];
//
//	// Update body pose
//	twc += Rwc*ut;
//	Rwc = Rwc*ExpSO3(ur);

	Eigen::Matrix<double,6,1> update_se3; // tranlation first, then rotation
	update_se3<<pu[3],pu[4],pu[5],pu[0],pu[1],pu[2];
	Sophus::SE3d T_w_c_SE3(Eigen::Quaterniond(Rwc), twc);
	T_w_c_SE3 = T_w_c_SE3 * Sophus::SE3d::exp(update_se3);

	twc = T_w_c_SE3.translation();
	Rwc = T_w_c_SE3.rotationMatrix();

	// Normalize rotation after 5 updates
	its++;
	if(its>=3)
	{
		NormalizeRotation(Rwc);
		its=0;
	}

	// Update left camera poses
	Rcw[0] = Rwc.transpose();
	tcw[0] = -Rcw[0] * twc;

	if (pCamera.size()>1)
	{
		Rcw[1] =  T_r_l.rotation() * Rwc.transpose();
		tcw[1] = -Rcw[1] * tcw[0] + T_r_l.translation();
	}

}

Eigen::Vector2d DvlImuCamPose::Project(const Eigen::Vector3d &Xw, int cam_idx) const
{
	Eigen::Vector3d Xc = Rcw[cam_idx]*Xw+tcw[cam_idx];

	return pCamera[cam_idx]->project(Xc);
}

Eigen::Vector3d DvlImuCamPose::ProjectStereo(const Eigen::Vector3d &Xw, int cam_idx) const
{
	Eigen::Vector3d Pc = Rcw[cam_idx]*Xw+tcw[cam_idx];
	Eigen::Vector3d pc;
	double invZ = 1/Pc(2);
	pc.head(2) = pCamera[cam_idx]->project(Pc);
	pc(2) = pc(0) - bf*invZ;
	return pc;
}

void DvlImuCamPose::write(ofstream &fout)
{
    boost::archive::text_oarchive oa(fout);
    oa << pCamera[0] <<"\n";
    // oa << pCamera[1] <<"\n";

    fout << tcw[0] <<"\n";
    fout << Rcw[0] <<"\n";

    fout << Rwc <<"\n";
    fout << twc <<"\n";

    // t_c_gyro[0] = Converter::toVector3d(pKF->mImuCalib.mT_c_gyro.rowRange(0,3).col(3));
    fout << t_c_gyro[0] <<"\n";
    // R_c_gyro[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_c_gyro.rowRange(0, 3).colRange(0, 3));
    fout << R_c_gyro[0] <<"\n";
    // t_gyro_c[0] = Converter::toVector3d(pKF->mImuCalib.mT_gyro_c.rowRange(0,3).col(3));
    fout << t_gyro_c[0] <<"\n";
    // R_gyro_c[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_gyro_c.rowRange(0, 3).colRange(0, 3));
    fout << R_gyro_c[0] <<"\n";

    // t_c_dvl[0] = Converter::toVector3d(pKF->mImuCalib.mT_c_dvl.rowRange(0,3).col(3));
    fout << t_c_dvl[0] <<"\n";
    // R_c_dvl[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_c_dvl.rowRange(0, 3).colRange(0, 3));
    fout << R_c_dvl[0] <<"\n";
    // t_dvl_c[0] = Converter::toVector3d(pKF->mImuCalib.mT_dvl_c.rowRange(0,3).col(3));
    fout << t_dvl_c[0] <<"\n";
    // R_dvl_c[0] = Converter::toMatrix3d(pKF->mImuCalib.mT_dvl_c.rowRange(0, 3).colRange(0, 3));
    fout << R_dvl_c[0] <<"\n";


    // bf = pKF->mbf;
    fout << bf <<"\n";

    // Right camera
    {
        // Eigen::Matrix4d Trl = Converter::toMatrix4d(pKF->mTrl);
        // T_r_l.matrix() = Trl;
        fout << T_r_l.matrix() <<"\n";
        // Rcw[1] = Trl.block<3,3>(0,0)*Rcw[0];
        fout << Rcw[1] <<"\n";
        // tcw[1] = Trl.block<3,3>(0,0)*tcw[0]+Trl.block<3,1>(0,3);
        fout << tcw[1] <<"\n";

        // t_c_gyro[1] = Trl.block<3,3>(0,0)*t_c_gyro[0]+Trl.block<3,1>(0,3);
        fout << t_c_gyro[1] <<"\n";
        // R_c_gyro[1] = Trl.block<3,3>(0,0) * R_c_gyro[0];
        fout << R_c_gyro[1] <<"\n";
        // R_gyro_c[1] = R_c_gyro[1].transpose();
        fout << R_gyro_c[1] <<"\n";
        // t_gyro_c[1] = -R_gyro_c[1]*t_c_gyro[1];
        fout << t_gyro_c[1] <<"\n";

        // t_c_dvl[1] = Trl.block<3,3>(0,0)*t_c_dvl[1]+Trl.block<3,1>(0,3);
        fout << t_c_dvl[1] <<"\n";
        // R_c_dvl[1] = Trl.block<3,3>(0,0) * R_c_dvl[0];
        fout << R_c_dvl[1] <<"\n";
        // R_dvl_c[1] = R_c_dvl[1].transpose();
        fout << R_dvl_c[1] <<"\n";
        // t_dvl_c[1] = -R_dvl_c[1]*t_c_dvl[1];
        fout << t_dvl_c[1] <<"\n";
        // pCamera[1] = pKF->mpCamera2;

    }

}
void ReadToEigen3D(ifstream &fin, Eigen::Matrix3d &mat)
{
    fin >> mat(0,0) >> mat(1,0) >> mat(2,0)
        >> mat(0,1) >> mat(1,1) >> mat(2,1)
        >> mat(0,2) >> mat(1,2) >> mat(2,2);
}
void ReadToEigen4D(ifstream &fin, Eigen::Matrix4d &mat)
{
    fin >> mat(0,0) >> mat(1,0) >> mat(2,0) >> mat(3,0)
        >> mat(0,1) >> mat(1,1) >> mat(2,1) >> mat(3,1)
        >> mat(0,2) >> mat(1,2) >> mat(2,2) >> mat(3,2)
        >> mat(0,3) >> mat(1,3) >> mat(2,3) >> mat(3,3);
}

void ReadtoVector3D(ifstream &fin, Eigen::Vector3d &vec)
{
    for(int i=0; i<3; i++)
    {
        fin >> vec(i);
    }
}

void DvlImuCamPose::read(ifstream &fin)
{
    // Load camera poses
    int num_cams = 2;

    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    t_c_gyro.resize(num_cams);
    R_c_gyro.resize(num_cams);
    t_gyro_c.resize(num_cams);
    R_gyro_c.resize(num_cams);
    t_dvl_c.resize(num_cams);
    R_dvl_c.resize(num_cams);
    t_c_dvl.resize(num_cams);
    R_c_dvl.resize(num_cams);
    pCamera.resize(num_cams);

    pCamera[0] = new Pinhole();
    boost::archive::text_iarchive inputArchive{fin};
    inputArchive >> pCamera[0];
    pCamera[1] = new Pinhole(static_cast<Pinhole*>(pCamera[0]));

    // fin >> tcw[0];
    ReadtoVector3D(fin, tcw[0]);
    // fin >> Rcw[0];
    ReadToEigen3D(fin, Rcw[0]);

    // fin >> Rwc;
    ReadToEigen3D(fin, Rwc);
    // fin >> twc;
    ReadtoVector3D(fin, twc);

    ReadtoVector3D(fin, t_c_gyro[0]);
    ReadToEigen3D(fin, R_c_gyro[0]);
    ReadtoVector3D(fin, t_gyro_c[0]);
    ReadToEigen3D(fin, R_gyro_c[0]);


    ReadtoVector3D(fin, t_c_dvl[0]);
    ReadToEigen3D(fin, R_c_dvl[0]);
    ReadtoVector3D(fin, t_dvl_c[0]);
    ReadToEigen3D(fin, R_dvl_c[0]);



    fin >> bf;

    // Right camera
    {
        // Eigen::Matrix4d Trl = Converter::toMatrix4d(pKF->mTrl);
        // T_r_l.matrix() = Trl;
        ReadToEigen4D(fin, T_r_l.matrix());

        ReadToEigen3D(fin, Rcw[1]);

        ReadtoVector3D(fin, tcw[1]);

        ReadtoVector3D(fin, t_c_gyro[1]);
        ReadToEigen3D(fin, R_c_gyro[1]);
        ReadToEigen3D(fin, R_gyro_c[1]);
        ReadtoVector3D(fin, t_gyro_c[1]);
        ReadtoVector3D(fin, t_c_dvl[1]);
        ReadToEigen3D(fin, R_c_dvl[1]);
        ReadToEigen3D(fin, R_dvl_c[1]);
        ReadtoVector3D(fin, t_dvl_c[1]);
    }
}

void DvlImuCamPose::Reset()
{
    // Load camera poses
    int num_cams = 2;

    tcw.resize(num_cams);
    Rcw.resize(num_cams);


    tcw[0] = Eigen::Vector3d::Zero();
    Rcw[0] = Eigen::Matrix3d::Identity();
    tcw[1] = Eigen::Vector3d::Zero();
    Rcw[1] = Eigen::Matrix3d::Identity();
    Rwc = Eigen::Matrix3d::Identity();
    twc = Eigen::Vector3d::Zero();

}

DvlImuCamPose::DvlImuCamPose(const DvlImuCamPose &dic)
{
    mTimestamp = dic.mTimestamp;
    mPoorVision = dic.mPoorVision;
    // Load Gyro rotation and Dvl translation
    //	t_c0_gyro = Converter::toVector3d(pF->GetDvlPosition());
    //	R_c0_gyro = Converter::toMatrix3d(pF->GetGyroRotation());

    Rwc = dic.Rwc;
    twc = dic.twc;
    //	Rwc = Converter::toMatrix3d(pF->mRwc.t());
    //	twc = - Rwc * Converter::toVector3d(pF->mTcw.rowRange(0,3).col(3));


    // Load camera poses
    int num_cams = dic.tcw.size();

    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    t_c_gyro.resize(num_cams);
    R_c_gyro.resize(num_cams);
    t_gyro_c.resize(num_cams);
    R_gyro_c.resize(num_cams);
    t_dvl_c.resize(num_cams);
    R_dvl_c.resize(num_cams);
    t_c_dvl.resize(num_cams);
    R_c_dvl.resize(num_cams);
    pCamera.resize(num_cams);

    // Left camera
    tcw[0] = dic.tcw[0];
    Rcw[0] = dic.Rcw[0];
    t_c_gyro[0] = dic.t_c_gyro[0];
    R_c_gyro[0] = dic.R_c_gyro[0];
    t_gyro_c[0] = dic.t_gyro_c[0];
    R_gyro_c[0] = dic.R_gyro_c[0];
    t_c_dvl[0] = dic.t_c_dvl[0];
    R_c_dvl[0] = dic.R_c_dvl[0];
    t_dvl_c[0] = dic.t_dvl_c[0];
    R_dvl_c[0] = dic.R_dvl_c[0];
    pCamera[0] = dic.pCamera[0];
    bf = dic.bf;

    if(num_cams>1)
    {
        T_r_l.matrix() = dic.T_r_l.matrix();
        Rcw[1] = dic.Rcw[1];
        tcw[1] = dic.tcw[1];

        t_c_gyro[1] = dic.t_c_gyro[1];
        R_c_gyro[1] = dic.R_c_gyro[1];
        R_gyro_c[1] = dic.R_gyro_c[1];
        t_gyro_c[1] = dic.t_gyro_c[1];

        t_c_dvl[1] = dic.t_c_dvl[1];
        R_c_dvl[1] = dic.R_c_dvl[1];
        R_dvl_c[1] = dic.R_dvl_c[1];
        t_dvl_c[1] = dic.t_dvl_c[1];

        pCamera[1] = dic.pCamera[1];
    }
}

    template<typename MatrixType, typename Archive>
void serializeEigenMatrix(Archive &ar, MatrixType &m, const unsigned int version)
{
    int cols = m.cols();
    int rows = m.rows();
    ar & cols;
    ar & rows;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            ar & m(i, j);
        }
    }
}

template<class Archive>
void DvlImuCamPose::serialize(Archive &ar, const unsigned int version)
{
    ar & mCamNum;
    ar & mTimestamp;
    ar & mPoorVision;

    tcw.resize(mCamNum);
    Rcw.resize(mCamNum);
    t_c_gyro.resize(mCamNum);
    R_c_gyro.resize(mCamNum);
    t_gyro_c.resize(mCamNum);
    R_gyro_c.resize(mCamNum);
    t_dvl_c.resize(mCamNum);
    R_dvl_c.resize(mCamNum);
    t_c_dvl.resize(mCamNum);
    R_c_dvl.resize(mCamNum);
    pCamera.resize(mCamNum);

    ar & bf;

    for(int i=0; i<mCamNum; i++)
    {
        ar & pCamera[i];
        serializeEigenMatrix(ar,tcw[i], version);
        serializeEigenMatrix(ar,Rcw[i], version);
        serializeEigenMatrix(ar,t_c_gyro[i], version);
        serializeEigenMatrix(ar,R_c_gyro[i], version);
        serializeEigenMatrix(ar,t_gyro_c[i], version);
        serializeEigenMatrix(ar,R_gyro_c[i], version);

        serializeEigenMatrix(ar,t_c_dvl[i], version);
        serializeEigenMatrix(ar,R_c_dvl[i], version);
        serializeEigenMatrix(ar,t_dvl_c[i], version);
        serializeEigenMatrix(ar,R_dvl_c[i], version);
    }
    // Left camera

    Rwc = Rcw[0].transpose();
    twc = - Rwc * tcw[0];
    if (mCamNum == 2)
    {
        serializeEigenMatrix(ar,T_r_l.matrix(), version);
    }

}

EdgeDvlGyroInit::EdgeDvlGyroInit(DVLGroPreIntegration *pInt):mpInt(pInt), dt(pInt->dT)
{
	resize(5);
}
void EdgeDvlGyroInit::computeError()
{
	const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
	const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
	const auto * V_bw = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
	const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[3]);
	const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[4]);

	const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
	const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
	const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
	const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
	const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

	const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
	const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
	cv::Mat R_g_d;
	cv::eigen2cv(R_gyros_dvl,R_g_d);
	R_g_d.convertTo(R_g_d,CV_32FC1);
	const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();
//	const Eigen::Vector3d t_gyros_dvl=T_gyros_dvl.translation();
//	const Eigen::Vector3d t_dvl_gyros=T_gyros_dvl.inverse().translation();

	const IMU::Bias b(0,0,0,V_bw->estimate()[0],V_bw->estimate()[1],V_bw->estimate()[2]);
	// mpInt->ReintegrateWithBiasAndRotation(b,R_g_d);
//	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->GetDeltaRotation(b,R_g_d));
//	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->GetDeltaPosition(b,R_g_d));
	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->dR);
	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->dP_dvl);

//	Eigen::Isometry3d T_gi_gj_mea=Eigen::Isometry3d::Identity();
//	Eigen::Isometry3d T_gi_gj_est=Eigen::Isometry3d::Identity();
//	T_gi_gj_mea.rotate(dR);
//	T_gi_gj_mea.pretranslate(dP);



//	const Eigen::Matrix3d R_est=R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
//	const Eigen::Vector3d t_est=R_gyros_dvl * (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
//		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));
	const Eigen::Matrix3d R_est= R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
	const Eigen::Vector3d t_est= (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));
//	const Eigen::Vector3d t_est=R_dvl_c * VP1->estimate().Rcw[0]*(t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
//		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));

//	T_gi_gj_est.rotate(R_est);
//	T_gi_gj_est.pretranslate(t_est);

//	Eigen::Vector3d p_0(1,1,1);


//	Eigen::Vector3d err= T_gi_gj_est.inverse() * p_0 - T_gi_gj_mea.inverse() * p_0;

	const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

	const Eigen::Vector3d e_p =  t_est - dP;

//	_error<<err;
	_error<<e_R,e_p;
//	cout<<"\nid:  "<<VP2->id()<<"error_R:\n"<<e_R.transpose()
//	<<"\ndR: \n"<<dR
//	<<"\nR_est: \n"<<R_est<<endl;
//
//	cout<<"error_p:\n"<<e_p.transpose()
//	<<"\ndP: \n"<<dP
//	<<"\nP_est: \n"<<t_est<<endl;

}
EdgeDvlGyroInit2::EdgeDvlGyroInit2(DVLGroPreIntegration *pInt):mpInt(pInt), dt(pInt->dT)
{
	resize(6);
}
void EdgeDvlGyroInit2::computeError()
{
	const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
	const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
	const auto * V_bw = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
	const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[3]);
	const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[4]);
	const auto * V_alpha_beta = dynamic_cast<const VertexDVLBeamOritenstion*>(_vertices[5]);

	const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
	const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
	const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
	const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
	const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

	const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
	const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
	cv::Mat R_g_d;
	cv::eigen2cv(R_gyros_dvl,R_g_d);
	R_g_d.convertTo(R_g_d,CV_32FC1);
	const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

	Eigen::Vector4d alpha,beta,alpha_gt,beta_gt,alpha_1,beta_1;
	Eigen::Matrix<double,8,1> alpha_beta = V_alpha_beta->estimate();
	alpha << alpha_beta(0,0), alpha_beta(1,0), alpha_beta(2,0), alpha_beta(3,0);
	beta << alpha_beta(4,0), alpha_beta(5,0), alpha_beta(6,0), alpha_beta(7,0);
	alpha_gt<< 67.5 / 180.0 * M_PI,
		67.5 / 180.0 * M_PI,
		67.5 / 180.0 * M_PI,
		67.5 / 180.0 * M_PI;
	beta_gt<<45 / 180.0 * M_PI,
		45 / 180.0 * M_PI,
		45 / 180.0 * M_PI,
		45 / 180.0 * M_PI;
	alpha_1<< 1 / 180.0 * M_PI,
		1 / 180.0 * M_PI,
		1 / 180.0 * M_PI,
		1 / 180.0 * M_PI;
	beta_1<<1 / 180.0 * M_PI,
		1 / 180.0 * M_PI,
		1 / 180.0 * M_PI,
		1 / 180.0 * M_PI;


	const IMU::Bias b(0,0,0,V_bw->estimate()[0],V_bw->estimate()[1],V_bw->estimate()[2]);
	mpInt->ReintegrateWithBiasRotationBeamOri(b,R_g_d,alpha,beta);

	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->dR);
	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->dP_dvl);


	const Eigen::Matrix3d R_est= R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
	const Eigen::Vector3d t_est= (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));


//	const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);
	const Eigen::Vector3d e_R(0,0,0);

	const Eigen::Vector3d e_p =  t_est - dP;

	_error<<e_R,e_p;

}

EdgeDvlGyroInit3::EdgeDvlGyroInit3(DVLGroPreIntegration *pInt):mpInt(pInt), dt(pInt->dT)
{
	resize(6);
}
void EdgeDvlGyroInit3::computeError()
{
	const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
	const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
	const auto * V_bw = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
	const auto * V_v = dynamic_cast<const VertexVelocity*>(_vertices[3]);
	const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[4]);
	const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[5]);


	const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
	const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
	const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
	const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
	const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

	const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
	const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
	cv::Mat R_g_d;
	cv::eigen2cv(R_gyros_dvl,R_g_d);
	R_g_d.convertTo(R_g_d,CV_32FC1);
	const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

	Eigen::Vector3d v_d = V_v->estimate();
	Eigen::Vector3d v_test(0.1,0.1,0.1);
	Eigen::Vector3d v_gt(mpInt->v_dk_dvl.x, mpInt->v_dk_dvl.y, mpInt->v_dk_dvl.z);

	mpInt->ReintegrateWithVelocity(v_d);



	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->dR);
	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->dP_dvl);


	const Eigen::Matrix3d R_est= R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
	const Eigen::Vector3d t_est= (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));



//	mpInt->ReintegrateWithVelocity(v_gt );

//	const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

	const Eigen::Vector3d e_R(0,0,0);

	const Eigen::Vector3d e_p =  t_est - dP;

	_error<<e_R,e_p;

}

EdgeDvlIMUInitWithoutBias::EdgeDvlIMUInitWithoutBias(DVLGroPreIntegration *pInt): mpInt(pInt), dt(pInt->dT)
{
	resize(9);
}
void EdgeDvlIMUInitWithoutBias::computeError()
{
	const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
	const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
    const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
    const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
    const auto * VG = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
    const auto * VA = dynamic_cast<const VertexAccBias*>(_vertices[5]);
	const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
	const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
    const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);


	const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
	const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
	const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
	const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
	const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

	const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
	const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
	cv::Mat R_g_d;
	cv::eigen2cv(R_gyros_dvl,R_g_d);
	R_g_d.convertTo(R_g_d,CV_32FC1);
	const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

	Eigen::Vector3d v1 = VV1->estimate();
    Eigen::Vector3d v2 = VV2->estimate();

    Eigen::Matrix3d R_b0_w = VR_G->estimate().Rwg;
    Eigen::Vector3d g_w = Eigen::Vector3d(0,0,-9.81);

    IMU::Bias b(VA->estimate().x(),VA->estimate().y(),VA->estimate().z(), VG->estimate().x(),VG->estimate().y(),VG->estimate().z());
    // mpInt->ReintegrateWithBias(b);

    // mpInt->ReintegrateWithVelocity();
    const Eigen::Vector3d dDelta_V = Converter::toVector3d(mpInt->GetDeltaVelocity(b));


    // R_b_d * R_d_c * R_ci_c0 *  ( * R_c0_cj * R_c_d*V_dj -  R_c0_cj
    // * R_c_d*V_di - R_c_d * R_d_b * R_b0_w * g_w * t_ij)
    const Eigen::Vector3d VDelta_est = R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0]* (VP2->estimate().Rwc*R_c_dvl*v2
            - VP1->estimate().Rwc*R_c_dvl*v1 - R_c_dvl *R_dvl_gyros * R_b0_w * g_w*mpInt->dT);

//	mpInt->ReintegrateWithVelocity(v_gt );

//	const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

	const Eigen::Vector3d e_V = VDelta_est - dDelta_V;


	_error<<e_V;
}

EdgeDvlIMUInit::EdgeDvlIMUInit(DVLGroPreIntegration* pInt): mpInt(pInt), dt(mpInt->dT)
{
    resize(9);
}

void EdgeDvlIMUInit::computeError()
{
    const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
    const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
    const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
    const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
    const auto * VG = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
    const auto * VA = dynamic_cast<const VertexAccBias*>(_vertices[5]);
    const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
    const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
    const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);


    const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
    const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
    const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
    const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
    const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

    const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
    const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
    cv::Mat R_g_d;
    cv::eigen2cv(R_gyros_dvl,R_g_d);
    R_g_d.convertTo(R_g_d,CV_32FC1);
    const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

    Eigen::Vector3d v1 = VV1->estimate();
    Eigen::Vector3d v2 = VV2->estimate();

    Eigen::Matrix3d R_b0_w = VR_G->estimate().Rwg;
    Eigen::Vector3d g_w = Eigen::Vector3d(0,0,-9.81);

    IMU::Bias b(VA->estimate().x(),VA->estimate().y(),VA->estimate().z(), VG->estimate().x(),VG->estimate().y(),VG->estimate().z());
    // mpInt->ReintegrateWithBias(b);

    // mpInt->ReintegrateWithVelocity();
    const Eigen::Vector3d dDelta_V = Converter::toVector3d(mpInt->GetDeltaVelocity(b));


    // R_b_d * R_d_c * R_ci_c0 *  ( * R_c0_cj * R_c_d*V_dj -  R_c0_cj
    // * R_c_d*V_di - R_c_d * R_d_b * R_b0_w * g_w * t_ij)
    const Eigen::Vector3d VDelta_est = R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0]* (VP2->estimate().Rwc*R_c_dvl*v2
                                                                                        - VP1->estimate().Rwc*R_c_dvl*v1 - R_c_dvl *R_dvl_gyros * R_b0_w * g_w*mpInt->dT);

    //	mpInt->ReintegrateWithVelocity(v_gt );

    //	const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

    const Eigen::Vector3d e_V = VDelta_est - dDelta_V;


    _error<<e_V;
}

EdgeDvlIMUGravityRefine::EdgeDvlIMUGravityRefine(DVLGroPreIntegration* pInt):mpInt(pInt), dt(pInt->dT)
{
    resize(9);
}

void EdgeDvlIMUGravityRefine::computeError()
{
    const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
    const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
    const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
    const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
    const auto * VG = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
    const auto * VA = dynamic_cast<const VertexAccBias*>(_vertices[5]);
    const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
    const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
    const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);

    const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
    const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
    const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
    const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
    const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

    const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
    const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
    cv::Mat R_g_d;
    cv::eigen2cv(R_gyros_dvl,R_g_d);
    R_g_d.convertTo(R_g_d,CV_32FC1);
    const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

    Eigen::Vector3d v1 = VV1->estimate();
    Eigen::Vector3d v2 = VV2->estimate();

    Eigen::Matrix3d R_b0_w = VR_G->estimate().Rwg;
    Eigen::Vector3d g_w = Eigen::Vector3d(0,0,-9.81);

    IMU::Bias b(VA->estimate().x(),VA->estimate().y(),VA->estimate().z(), VG->estimate().x(),VG->estimate().y(),VG->estimate().z());
    // mpInt->ReintegrateWithBias(b);

    // mpInt->ReintegrateWithVelocity();
    const Eigen::Vector3d dDelta_V = Converter::toVector3d(mpInt->GetDeltaVelocity(b));


    // R_b_d * R_d_c * R_ci_c0 *  ( * R_c0_cj * R_c_d*V_dj -  R_c0_cj
    // * R_c_d*V_di - R_c_d * R_d_b * R_b0_w * g_w * t_ij)
    const Eigen::Vector3d VDelta_est = R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0]* (VP2->estimate().Rwc*R_c_dvl*v2
                                                                                        - VP1->estimate().Rwc*R_c_dvl*v1 - R_c_dvl *R_dvl_gyros * R_b0_w * g_w*mpInt->dT);

    //	mpInt->ReintegrateWithVelocity(v_gt );

    //	const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

    const Eigen::Vector3d e_V = VDelta_est - dDelta_V;


    _error<<e_V;
}

EdgeDvlIMUGravityRefineWithBias::EdgeDvlIMUGravityRefineWithBias(DVLGroPreIntegration* pInt):mpInt(pInt), dt(pInt->dT)
{
    resize(9);
}

void EdgeDvlIMUGravityRefineWithBias::computeError()
{
    const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
    const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
    const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
    const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
    const auto * VG = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
    const auto * VA = dynamic_cast<const VertexAccBias*>(_vertices[5]);
    const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
    const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
    const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);

    const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
    const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
    const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
    const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
    const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

    const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
    const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
    // cv::Mat R_g_d;
    // cv::eigen2cv(R_gyros_dvl,R_g_d);
    // R_g_d.convertTo(R_g_d,CV_32FC1);
    const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

    Eigen::Isometry3d T_b_c = T_gyros_dvl *T_dvl_c;
    const Eigen::Matrix3d R_b_c=T_b_c.rotation();
    const Eigen::Vector3d t_b_c=T_b_c.translation();
    const Eigen::Matrix3d R_c_b=T_b_c.inverse().rotation();
    const Eigen::Vector3d t_c_b=T_b_c.inverse().translation();

    Eigen::Vector3d v1 = VV1->estimate();
    Eigen::Vector3d v2 = VV2->estimate();

    Eigen::Matrix3d R_b0_w = VR_G->estimate().Rwg;
    Eigen::Vector3d g_w = Eigen::Vector3d(0,0,-9.81);

    IMU::Bias b(VA->estimate().x(),VA->estimate().y(),VA->estimate().z(), VG->estimate().x(),VG->estimate().y(),VG->estimate().z());
    // mpInt->ReintegrateWithBias(b);

    // mpInt->ReintegrateWithVelocity();
    const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->GetDeltaRotation(b));
    const Eigen::Vector3d dDelta_V = Converter::toVector3d(mpInt->GetDeltaVelocity(b));
    const Eigen::Vector3d dP_acc =Converter::toVector3d(mpInt->GetDeltaPosition(b));



    const Eigen::Matrix3d R_est= R_b_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_b;

    // R_b_d * R_d_c * R_ci_c0 *  ( * R_c0_cj * R_c_d*V_dj -  R_c0_cj
    // * R_c_d*V_di - R_c_d * R_d_b * R_b0_w * g_w * t_ij)
    const Eigen::Vector3d VDelta_est = R_b_c * VP1->estimate().Rcw[0]* (VP2->estimate().Rwc*R_c_dvl*v2
                                                                        - VP1->estimate().Rwc*R_c_dvl*v1 - R_c_b * R_b0_w * g_w*mpInt->dT);

    // R_b_c * R_ci_c0 * [R_c0_cj * P_c_c_b + P_c0_c0_cj - (R_c0_ci * P_c_c_b + p_c0_c0_ci)
    // - R_c0_ci * R_c_d * V_di * t_ij - 0.5 * R_c_b * R_b0_w * g_w * t_ij^2]

    const Eigen::Vector3d P_acc_est = R_b_c * VP1->estimate().Rcw[0] * (VP2->estimate().Rwc * t_c_b + VP2->estimate().twc - (VP1->estimate().Rwc * t_c_b  + VP1->estimate().twc)
                                                                        - VP1->estimate().Rwc * R_c_dvl * v1 * dt - 0.5 * R_c_b * R_b0_w * g_w*dt*dt);

    //	mpInt->ReintegrateWithVelocity(v_gt );

    const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

    const Eigen::Vector3d e_V = VDelta_est - dDelta_V;
    const Eigen::Vector3d e_P = P_acc_est - dP_acc;

    _error<<e_R, e_V, e_P;
}

EdgeDvlIMUWithBias::EdgeDvlIMUWithBias(DVLGroPreIntegration* pInt):mpInt(pInt), dt(pInt->dT)
{
    resize(9);
}

void EdgeDvlIMUWithBias::computeError()
{
    const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
    const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
    const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
    const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
    const auto * VG = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
    const auto * VA = dynamic_cast<const VertexAccBias*>(_vertices[5]);
    const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
    const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
    const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);

    const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
    const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
    const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
    const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
    const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

    const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
    const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
    // cv::Mat R_g_d;
    // cv::eigen2cv(R_gyros_dvl,R_g_d);
    // R_g_d.convertTo(R_g_d,CV_32FC1);
    const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

    Eigen::Isometry3d T_b_c = T_gyros_dvl *T_dvl_c;
    const Eigen::Matrix3d R_b_c=T_b_c.rotation();
    const Eigen::Vector3d t_b_c=T_b_c.translation();
    const Eigen::Matrix3d R_c_b=T_b_c.inverse().rotation();
    const Eigen::Vector3d t_c_b=T_b_c.inverse().translation();

    Eigen::Vector3d v1 = VV1->estimate();
    Eigen::Vector3d v2 = VV2->estimate();

    Eigen::Matrix3d R_b0_w = VR_G->estimate().Rwg;
    Eigen::Vector3d g_w = Eigen::Vector3d(0,0,-9.81);

    IMU::Bias b(VA->estimate().x(),VA->estimate().y(),VA->estimate().z(), VG->estimate().x(),VG->estimate().y(),VG->estimate().z());
    // mpInt->ReintegrateWithBias(b);

    // mpInt->ReintegrateWithVelocity();
    const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->GetDeltaRotation(b));
    const Eigen::Vector3d dDelta_V = Converter::toVector3d(mpInt->GetDeltaVelocity(b));
    const Eigen::Vector3d dP_acc =Converter::toVector3d(mpInt->GetDeltaPosition(b));



    const Eigen::Matrix3d R_est= R_b_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_b;

    // R_b_d * R_d_c * R_ci_c0 *  ( * R_c0_cj * R_c_d*V_dj -  R_c0_cj
    // * R_c_d*V_di - R_c_d * R_d_b * R_b0_w * g_w * t_ij)
    const Eigen::Vector3d VDelta_est = R_b_c * VP1->estimate().Rcw[0]* (VP2->estimate().Rwc*R_c_dvl*v2
                                                                        - VP1->estimate().Rwc*R_c_dvl*v1 - R_c_b * R_b0_w * g_w*mpInt->dT);

    // R_b_c * R_ci_c0 * [R_c0_cj * P_c_c_b + P_c0_c0_cj - (R_c0_ci * P_c_c_b + p_c0_c0_ci)
    // - R_c0_ci * R_c_d * V_di * t_ij - 0.5 * R_c_b * R_b0_w * g_w * t_ij^2]

    const Eigen::Vector3d P_acc_est = R_b_c * VP1->estimate().Rcw[0] * (VP2->estimate().Rwc * t_c_b + VP2->estimate().twc - (VP1->estimate().Rwc * t_c_b  + VP1->estimate().twc)
                                                                        - VP1->estimate().Rwc * R_c_dvl * v1 * dt - 0.5 * R_c_b * R_b0_w * g_w*dt*dt);

    //	mpInt->ReintegrateWithVelocity(v_gt );

    const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

    const Eigen::Vector3d e_V = VDelta_est - dDelta_V;
    const Eigen::Vector3d e_P = P_acc_est - dP_acc;

    _error<<e_R, e_V, e_P;

}

EdgeDvlIMU::EdgeDvlIMU(DVLGroPreIntegration* pInt):JRg(Converter::toMatrix3d(pInt->JRg)),
                                                   JVg(Converter::toMatrix3d(pInt->JVg)), JPg(Converter::toMatrix3d(pInt->JPg)), JVa(Converter::toMatrix3d(pInt->JVa)),
                                                   JPa(Converter::toMatrix3d(pInt->JPa)),mpInt(pInt), dt(pInt->dT)
{
    resize(9);
}

void EdgeDvlIMU::computeError()
{
    const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
    const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
    const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
    const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
    const auto * VG = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
    const auto * VA = dynamic_cast<const VertexAccBias*>(_vertices[5]);
    const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
    const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
    const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);

    const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
    const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
    const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
    const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
    const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

    const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
    const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
    // cv::Mat R_g_d;
    // cv::eigen2cv(R_gyros_dvl,R_g_d);
    // R_g_d.convertTo(R_g_d,CV_32FC1);
    const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

    Eigen::Isometry3d T_b_c = T_gyros_dvl *T_dvl_c;
    const Eigen::Matrix3d R_b_c=T_b_c.rotation();
    const Eigen::Vector3d t_b_c=T_b_c.translation();
    const Eigen::Matrix3d R_c_b=T_b_c.inverse().rotation();
    const Eigen::Vector3d t_c_b=T_b_c.inverse().translation();

    Eigen::Vector3d v1 = VV1->estimate();
    Eigen::Vector3d v2 = VV2->estimate();
    Eigen::Vector3d avg_v = (v1+v2)/2;

    Eigen::Matrix3d R_b0_w = VR_G->estimate().Rwg;
    Eigen::Vector3d g_w = Eigen::Vector3d(0,0,-9.81);

    IMU::Bias b(VA->estimate().x(),VA->estimate().y(),VA->estimate().z(), VG->estimate().x(),VG->estimate().y(),VG->estimate().z());
    // mpInt->ReintegrateWithBias(b);

    // mpInt->ReintegrateWithVelocity();
    const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->GetDeltaRotation(b));
    const Eigen::Vector3d dDelta_V = Converter::toVector3d(mpInt->GetDeltaVelocity(b));
    const Eigen::Vector3d dP_acc =Converter::toVector3d(mpInt->GetDeltaPosition(b));
    const Eigen::Vector3d dP_dvl =Converter::toVector3d(mpInt->GetDVLPosition(b,avg_v));



    const Eigen::Matrix3d R_est= R_b_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_b;

    // R_b_d * R_d_c * R_ci_c0 *  ( * R_c0_cj * R_c_d*V_dj -  R_c0_cj
    // * R_c_d*V_di - R_c_d * R_d_b * R_b0_w * g_w * t_ij)
    const Eigen::Vector3d VDelta_est = R_b_c * VP1->estimate().Rcw[0]* (VP2->estimate().Rwc*R_c_dvl*v2 - VP1->estimate().Rwc*R_c_dvl*v1 - R_c_b * R_b0_w * g_w*mpInt->dT);

    // R_b_c * R_ci_c0 * [R_c0_cj * P_c_c_b + P_c0_c0_cj - (R_c0_ci * P_c_c_b + p_c0_c0_ci) - R_c0_ci * R_c_d * V_di * t_ij - 0.5 * R_c_b * R_b0_w * g_w * t_ij^2]

    const Eigen::Vector3d P_acc_est = R_b_c * VP1->estimate().Rcw[0] * (VP2->estimate().Rwc * t_c_b + VP2->estimate().twc - (VP1->estimate().Rwc * t_c_b  + VP1->estimate().twc) - VP1->estimate().Rwc * R_c_dvl * v1 * dt - 0.5 * R_c_b * R_b0_w * g_w*dt*dt);
    const Eigen::Vector3d P_dvl_est= (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
                                  + R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));

    //	mpInt->ReintegrateWithVelocity(v_gt );

    // Eigen::Vector3d e_R = R_b0_w.transpose() * LogSO3(dR.transpose() * R_est);
    Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);
    // e_R[0]=e_R[0]*50;//10_24 GX5 X axis is yaw
    // e_R[1]=e_R[1]*50;//bofore 10_24 GX5 Y axis is yaw
    // e_R[2]=e_R[2]*10;

    const Eigen::Vector3d e_V =  (VDelta_est - dDelta_V);
    const Eigen::Vector3d e_P =  (P_acc_est - dP_acc);
    // const Eigen::Vector3d e_P =  (P_dvl_est - dP_dvl);

    _error<<e_R, e_V, e_P;
}

bool EdgeDvlIMU::read(istream &is)
{
    //get information matrix
    Eigen::Matrix<double, 9, 9> info;
    for(int i=0; i<9; i++)
        for(int j=0; j<9; j++)
            is>>info(i,j);
    setInformation(info);
    //get preintegration
    is.get();
    std::string str;
    getline(is, str);
    boost::replace_all(str, "*newline*", "\n");
    stringstream ss(str);
    boost::archive::text_iarchive ia(ss);
    mpInt = new DVLGroPreIntegration();
    ia >> mpInt;
    dt = mpInt->dT;
    return true;
}

bool EdgeDvlIMU::write(ostream &os) const
{
    //save information matrix
    for(int i=0; i<9; i++)
        for(int j=0; j<9; j++)
            os<<_information(i,j)<<" ";
    //save preintegration
    stringstream ss;
    boost::archive::text_oarchive oa2(ss);
    oa2 << mpInt;
    std::string str = ss.str();
    boost::replace_all(str, "\n", "*newline*");
    os << str<<" ";
    return true;
}

EdgeDvlIMU::EdgeDvlIMU():JRg(Eigen::Matrix3d::Zero()),JVg(Eigen::Matrix3d::Zero()),
JVa(Eigen::Matrix3d::Zero()),JPg(Eigen::Matrix3d::Zero()),JPa(Eigen::Matrix3d::Zero()),
dt(0.0),mpInt(NULL)
{
    resize(9);
}

// void EdgeDvlIMU::linearizeOplus()
// {
//     const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
//     const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
//     const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
//     const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
//     const auto * VG1 = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
//     const auto * VA1 = dynamic_cast<const VertexAccBias*>(_vertices[5]);
//     const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
//     const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
//     const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);
//
//     const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);
//     const IMU::Bias db = mpInt->GetDeltaBias(b1);
//     Eigen::Vector3d dbg;
//     dbg << db.bwx, db.bwy, db.bwz;
//     Eigen::Vector3d g(0,0,-9.81);
//
//     const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
//     const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
//     const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
//     const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
//     const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();
//
//     const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
//     const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
//     // cv::Mat R_g_d;
//     // cv::eigen2cv(R_gyros_dvl,R_g_d);
//     // R_g_d.convertTo(R_g_d,CV_32FC1);
//     const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();
//
//     Eigen::Isometry3d T_b_c = T_gyros_dvl *T_dvl_c;
//     const Eigen::Matrix3d R_b_c=T_b_c.rotation();
//     const Eigen::Vector3d t_b_c=T_b_c.translation();
//     const Eigen::Matrix3d R_c_b=T_b_c.inverse().rotation();
//     const Eigen::Vector3d t_c_b=T_b_c.inverse().translation();
//
//     Eigen::Matrix3d R_b0_w = VR_G->estimate().Rwg;
//     Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3,2);
//     Gm(0,1) = -IMU::GRAVITY_VALUE;
//     Gm(1,0) = IMU::GRAVITY_VALUE;
//     const Eigen::MatrixXd dGdTheta = R_b0_w*Gm;
//
//     Eigen::Isometry3d T_c0_ci =Eigen::Isometry3d::Identity();
//     T_c0_ci.rotate(VP1->estimate().Rwc);
//     T_c0_ci.pretranslate(VP1->estimate().twc);
//     Eigen::Isometry3d T_b0_bi = T_b_c * T_c0_ci * T_b_c.inverse();
//     const Eigen::Matrix3d Rwb1 = T_b0_bi.rotation();
//     const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
//
//     Eigen::Isometry3d T_c0_cj = Eigen::Isometry3d::Identity();
//     T_c0_cj.rotate(VP2->estimate().Rwc);
//     T_c0_cj.pretranslate(VP2->estimate().twc);
//     Eigen::Isometry3d T_b0_bj = T_b_c * T_c0_cj * T_b_c.inverse();
//     Eigen::Vector3d twb1 = T_b0_bi.translation();
//
//
//     const Eigen::Matrix3d Rwb2 = T_b0_bj.rotation();
//     Eigen::Vector3d twb2 = T_b0_bj.translation();
//
//
//     const Eigen::Matrix3d dR = Converter::toMatrix3d(mpInt->GetDeltaRotation(b1));
//     const Eigen::Matrix3d eR = dR.transpose()*Rbw1*Rwb2;
//     const Eigen::Vector3d er = LogSO3(eR);
//     const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);
//
//     // Jacobians wrt Pose 1
//     _jacobianOplus[0].setZero();
//     // rotation
//     _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1; // OK
//     _jacobianOplus[0].block<3,3>(3,0) = Skew(Rbw1*(VV2->estimate() - VV1->estimate() - g*dt)); // OK
//     _jacobianOplus[0].block<3,3>(6,0) = Skew(Rbw1*( twb2- twb1
//                                                    - VV1->estimate()*dt - 0.5*g*dt*dt)); // OK
//     // translation
//     _jacobianOplus[0].block<3,3>(6,3) = -Eigen::Matrix3d::Identity(); // OK
//
//     // Jacobians wrt Pose 2
//     _jacobianOplus[1].setZero();
//     // rotation
//     _jacobianOplus[1].block<3,3>(0,0) = invJr;
//     // translation
//     _jacobianOplus[1].block<3,3>(6,3) = Rbw1*Rwb2;
//
//     // Jacobians wrt Velocity 1
//     _jacobianOplus[2].setZero();
//     _jacobianOplus[2].block<3,3>(3,0) = Rbw1;
//     _jacobianOplus[2].block<3,3>(6,0) = Rbw1*dt;
//     // Jacobians wrt Velocity 2
//     _jacobianOplus[3].setZero();
//     _jacobianOplus[3].block<3,3>(3,0) = Rbw1;
//
//     // Jacobians wrt Gyro bias
//     _jacobianOplus[4].setZero();
//     _jacobianOplus[4].block<3,3>(0,0) = -invJr*eR.transpose()*RightJacobianSO3(JRg*dbg)*JRg;
//     _jacobianOplus[4].block<3,3>(3,0) = -JVg;
//     _jacobianOplus[4].block<3,3>(6,0) = -JPg;
//
//     // Jacobians wrt Accelerometer bias
//     _jacobianOplus[5].setZero();
//     _jacobianOplus[5].block<3,3>(3,0) = -JVa;
//     _jacobianOplus[5].block<3,3>(6,0) = -JPa;
//
//
//     // Jacobians wrt extrinsic VT_d_c
//     _jacobianOplus[6].setZero();
//
//     // Jacobians wrt extrinsic VT_g_d
//     _jacobianOplus[7].setZero();
//
//     // Jacobians wrt Gravity direction
//     _jacobianOplus[8].setZero();
//     _jacobianOplus[8].block<3,2>(3,0) = -Rbw1*dGdTheta*dt;
//     _jacobianOplus[8].block<3,2>(6,0) = -0.5*Rbw1*dGdTheta*dt*dt;
//
//
// }

    EdgeDvlGyroBA::EdgeDvlGyroBA(DVLGroPreIntegration *pInt):mpInt(pInt), dt(pInt->dT)
{
	resize(5);
}

void EdgeDvlGyroBA::computeError()
{
	const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
	const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
	const auto * V_bw = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
	const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[3]);
	const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[4]);

	const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
	const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
	const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
	const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
	const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

	const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
	const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
	cv::Mat R_g_d;
	cv::eigen2cv(R_gyros_dvl,R_g_d);
	R_g_d.convertTo(R_g_d,CV_32FC1);
	const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();
	//	const Eigen::Vector3d t_gyros_dvl=T_gyros_dvl.translation();
	//	const Eigen::Vector3d t_dvl_gyros=T_gyros_dvl.inverse().translation();

	const IMU::Bias b(0,0,0,V_bw->estimate()[0],V_bw->estimate()[1],V_bw->estimate()[2]);
	//	mpInt->ReintegrateWithBiasAndRotation(b,R_g_d);
	//	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->GetDeltaRotation(b,R_g_d));
	//	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->GetDeltaPosition(b,R_g_d));
	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->GetDeltaRotation(b));
	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->GetDVLPosition(b));

	//	Eigen::Isometry3d T_gi_gj_mea=Eigen::Isometry3d::Identity();
	//	Eigen::Isometry3d T_gi_gj_est=Eigen::Isometry3d::Identity();
	//	T_gi_gj_mea.rotate(dR);
	//	T_gi_gj_mea.pretranslate(dP);



	//	const Eigen::Matrix3d R_est=R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
	//	const Eigen::Vector3d t_est=R_gyros_dvl * (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
	//		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));
	const Eigen::Matrix3d R_est= R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
	const Eigen::Vector3d t_est= (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));
	//	const Eigen::Vector3d t_est=R_dvl_c * VP1->estimate().Rcw[0]*(t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
	//		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));

	//	T_gi_gj_est.rotate(R_est);
	//	T_gi_gj_est.pretranslate(t_est);

	//	Eigen::Vector3d p_0(1,1,1);


	//	Eigen::Vector3d err= T_gi_gj_est.inverse() * p_0 - T_gi_gj_mea.inverse() * p_0;

	Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

	Eigen::Vector3d e_p =  t_est - dP;
    // e_R(0) = 0;
    // e_R(1) = 0;
    // e_p(2) = 0;

	//	_error<<err;
	_error<<e_R,e_p;
}

void EdgeDvlGyroTrack::computeError()
{
	const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
	const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
	const auto * V_bw = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
	const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[3]);
	const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[4]);

	const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
	const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
	const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
	const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
	const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

	const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
	const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
	cv::Mat R_g_d;
	cv::eigen2cv(R_gyros_dvl,R_g_d);
	R_g_d.convertTo(R_g_d,CV_32FC1);
	const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();
//	const Eigen::Vector3d t_gyros_dvl=T_gyros_dvl.translation();
//	const Eigen::Vector3d t_dvl_gyros=T_gyros_dvl.inverse().translation();

	const IMU::Bias b(0,0,0,V_bw->estimate()[0],V_bw->estimate()[1],V_bw->estimate()[2]);
//	mpInt->ReintegrateWithBiasAndRotation(b,R_g_d);
//	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->GetDeltaRotation(b,R_g_d));
//	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->GetDeltaPosition(b,R_g_d));
	const Eigen::Matrix3d dR=Converter::toMatrix3d(mpInt->dR);
	const Eigen::Vector3d dP=Converter::toVector3d(mpInt->dP_dvl);

//	Eigen::Isometry3d T_gi_gj_mea=Eigen::Isometry3d::Identity();
//	Eigen::Isometry3d T_gi_gj_est=Eigen::Isometry3d::Identity();
//	T_gi_gj_mea.rotate(dR);
//	T_gi_gj_mea.pretranslate(dP);



//	const Eigen::Matrix3d R_est=R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
//	const Eigen::Vector3d t_est=R_gyros_dvl * (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
//		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));
	const Eigen::Matrix3d R_est= R_gyros_dvl * R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * R_dvl_gyros;
	const Eigen::Vector3d t_est= (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));
//	const Eigen::Vector3d t_est=R_dvl_c * VP1->estimate().Rcw[0]*(t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
//		+ R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));

//	T_gi_gj_est.rotate(R_est);
//	T_gi_gj_est.pretranslate(t_est);

//	Eigen::Vector3d p_0(1,1,1);


//	Eigen::Vector3d err= T_gi_gj_est.inverse() * p_0 - T_gi_gj_mea.inverse() * p_0;

	const Eigen::Vector3d e_R = LogSO3(dR.transpose() * R_est);

	const Eigen::Vector3d e_p =  t_est - dP;

//	_error<<err;
	_error<<e_R,e_p;
//	cout<<"\nid:  "<<VP2->id()<<"error_R:\n"<<e_R.transpose()
//	<<"\ndR: \n"<<dR
//	<<"\nR_est: \n"<<R_est<<endl;
//
//	cout<<"error_p:\n"<<e_p.transpose()
//	<<"\ndP: \n"<<dP
//	<<"\nP_est: \n"<<t_est<<endl;

}

void EdgeSE3DVLIMU::computeError()
{
    const VertexPoseDvlIMU *VPi = static_cast<const VertexPoseDvlIMU *>(_vertices[0]);
    const VertexPoseDvlIMU *VPj = static_cast<const VertexPoseDvlIMU *>(_vertices[1]);
    Eigen::Matrix3d R_c0_ci = VPi->estimate().Rwc;
    Eigen::Matrix3d R_c0_cj = VPj->estimate().Rwc;
    Eigen::Vector3d t_c0_ci = VPi->estimate().twc;
    Eigen::Vector3d t_c0_cj = VPj->estimate().twc;
    Eigen::Isometry3d T_c0_ci = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_c0_cj = Eigen::Isometry3d::Identity();
    T_c0_ci.linear() = R_c0_ci;
    T_c0_ci.translation() = t_c0_ci;
    T_c0_cj.linear() = R_c0_cj;
    T_c0_cj.translation() = t_c0_cj;
    Eigen::Isometry3d T_ci_cj = T_c0_ci.inverse() * T_c0_cj;
    Eigen::Matrix3d R_ci_cj = T_ci_cj.linear();
    Eigen::Vector3d t_ci_cj = T_ci_cj.translation();
    Eigen::Vector3d e_R = LogSO3(R_ci_cj.transpose() * mT_ci_cj.rotation());
    Eigen::Vector3d e_p = t_ci_cj - mT_ci_cj.translation();
    _error << e_R, e_p;
}

bool EdgeSE3DVLIMU::read(istream &in)
{
    //get information matrix
    Eigen::Matrix<double, 6, 6> info;
    for(int i=0; i<6; i++)
        for(int j=0; j<6; j++)
            in>>info(i,j);
    setInformation(info);
    //get measurement
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            in>>mT_ci_cj.matrix()(i,j);
    return true;
}

bool EdgeSE3DVLIMU::write(ostream &out) const
{
    //write information matrix
    Eigen::Matrix<double, 6, 6> info=information();
    for(int i=0; i<6; i++)
        for(int j=0; j<6; j++)
            out<<info(i,j)<<" ";
    //write measurement
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            out<<mT_ci_cj.matrix()(i,j)<<" ";
    return true;
}

bool VertexPoseDvlIMU::read(istream &is)
{
    is.get();
    std::string str;
    getline(is, str);
    boost::replace_all(str, "*newline*", "\n");
    stringstream ss(str);
    boost::archive::text_iarchive ia(ss);
    ia >> _estimate;
    return true;
}

bool VertexPoseDvlIMU::write(ostream &os) const
{
    // boost::archive::binary_oarchive oa(os);
    // oa << _estimate;
    stringstream ss;
    boost::archive::text_oarchive oa2(ss);
    oa2 << _estimate;
    std::string str = ss.str();
    boost::replace_all(str, "\n", "*newline*");
    os << str;
    return true;
}

bool VertexGDir::read(istream &is)
{
    Eigen::Matrix3d Rb0w;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            is >> Rb0w(i, j);
        }
    }
    _estimate.Rwg = Rb0w;

    return true;
}

bool VertexGDir::write(ostream &os) const
{
    Eigen::Matrix3d Rb0w = _estimate.Rwg;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            os << Rb0w(i, j) << " ";
        }
    }
    return true;
}

bool EdgeMonoBA_DvlGyros::read(istream &is)
{
    Eigen::Vector2d obs;
    is>>obs[0]>>obs[1];
    setMeasurement(obs);
    for(int i=0; i<2; i++)
    {
        for(int j=0; j<2; j++)
        {
            double info;
            is>>info;
            _information(i,j)=info;
        }
    }
    return true;
}

bool EdgeMonoBA_DvlGyros::write(ostream &os) const
{
    Eigen::Vector2d obs = _measurement;
    os<<obs[0]<<" "<<obs[1]<<" ";
    for(int i=0; i<2; i++)
    {
        for(int j=0; j<2; j++)
        {
            double info = _information(i,j);
            os<<info<<" ";
        }
    }
    return true;
}

bool EdgeStereoBA_DvlGyros::read(istream &is)
{
    Eigen::Vector3d obs;
    is>>obs[0]>>obs[1]>>obs[2];
    setMeasurement(obs);
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            double info;
            is>>info;
            _information(i,j)=info;
        }
    }
    return true;
}

bool EdgeStereoBA_DvlGyros::write(ostream &os) const
{
    Eigen::Vector3d obs = _measurement;
    os<<obs[0]<<" "<<obs[1]<<" "<<obs[2]<<" ";
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            double info = _information(i,j);
            os<<info<<" ";
        }
    }
    return true;
}

bool EdgeDvlVelocity::read(istream &is)
{
    Eigen::Vector3d obs;
    is>>obs[0]>>obs[1]>>obs[2];
    mV = obs;
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            double info;
            is>>info;
            _information(i,j)=info;
        }
    }
    return true;
}

bool EdgeDvlVelocity::write(ostream &os) const
{
    Eigen::Vector3d obs = mV;
    os<<obs[0]<<" "<<obs[1]<<" "<<obs[2]<<" ";
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            double info = _information(i,j);
            os<<info<<" ";
        }
    }
    return true;
}

EdgeDvlIMU2::EdgeDvlIMU2(DVLGroPreIntegration* pInt_i,DVLGroPreIntegration* pInt_j):mpInt_i(pInt_i), mpInt_j(pInt_j), dt(mpInt_j->dT)
{
    resize(9);
}

bool EdgeDvlIMU2::read(istream &is)
{
    //get information matrix
    Eigen::Matrix<double, 9, 9> info;
    for(int i=0; i<9; i++)
        for(int j=0; j<9; j++)
            is>>info(i,j);
    setInformation(info);
    //get preintegration
    is.get();
    std::string str;
    getline(is, str);
    boost::replace_all(str, "*newline*", "\n");
    stringstream ss(str);
    boost::archive::text_iarchive ia(ss);
    mpInt_j = new DVLGroPreIntegration();
    ia >> mpInt_j;
    mpInt_i = new DVLGroPreIntegration();
    ia >> mpInt_j;
    dt = mpInt_j->dT;
    return true;
}

bool EdgeDvlIMU2::write(ostream &os) const
{
    //save information matrix
    for(int i=0; i<9; i++)
        for(int j=0; j<9; j++)
            os<<_information(i,j)<<" ";
    //save preintegration
    stringstream ss;
    boost::archive::text_oarchive oa2(ss);
    oa2 << mpInt_j;
    oa2 << mpInt_i;
    std::string str = ss.str();
    boost::replace_all(str, "\n", "*newline*");
    os << str<<" ";
    return true;
}

void EdgeDvlIMU2::computeError()
{
    const auto * VP1 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[0]);
    const auto * VP2 = dynamic_cast<const VertexPoseDvlIMU*>(_vertices[1]);
    const auto * VV1 = dynamic_cast<const VertexVelocity*>(_vertices[2]);
    const auto * VV2 = dynamic_cast<const VertexVelocity*>(_vertices[3]);
    const auto * VG = dynamic_cast<const VertexGyroBias*>(_vertices[4]);
    const auto * VA = dynamic_cast<const VertexAccBias*>(_vertices[5]);
    const auto * VT_d_c = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[6]);
    const auto * VT_g_d = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[7]);
    const auto * VR_G = dynamic_cast<const VertexGDir*>(_vertices[8]);

    const Eigen::Isometry3d T_dvl_c=VT_d_c->estimate();
    const Eigen::Matrix3d R_dvl_c=T_dvl_c.rotation();
    const Eigen::Matrix3d R_c_dvl=T_dvl_c.inverse().rotation();
    const Eigen::Vector3d t_dvl_c=T_dvl_c.translation();
    const Eigen::Vector3d t_c_dvl=T_dvl_c.inverse().translation();

    const Eigen::Isometry3d T_gyros_dvl=VT_g_d->estimate();
    const Eigen::Matrix3d R_gyros_dvl=T_gyros_dvl.rotation();
    // cv::Mat R_g_d;
    // cv::eigen2cv(R_gyros_dvl,R_g_d);
    // R_g_d.convertTo(R_g_d,CV_32FC1);
    const Eigen::Matrix3d R_dvl_gyros=R_gyros_dvl.transpose();

    Eigen::Isometry3d T_b_c = T_gyros_dvl *T_dvl_c;
    const Eigen::Matrix3d R_b_c=T_b_c.rotation();
    const Eigen::Vector3d t_b_c=T_b_c.translation();
    const Eigen::Matrix3d R_c_b=T_b_c.inverse().rotation();
    const Eigen::Vector3d t_c_b=T_b_c.inverse().translation();

    Eigen::Vector3d vdi = VV1->estimate();
    Eigen::Vector3d vdj = VV2->estimate();

    const Eigen::Vector3d dV_di  = Converter::toVector3d(mpInt_i->mVelocity);
    const Eigen::Vector3d dV_dj  = Converter::toVector3d(mpInt_j->mVelocity);
    Eigen::Vector3d avg_v = (dV_di + dV_dj) / 2;

    IMU::Bias b(VA->estimate().x(),VA->estimate().y(),VA->estimate().z(), VG->estimate().x(),VG->estimate().y(),VG->estimate().z());

    const Eigen::Vector3d dP_dvl = Converter::toVector3d(mpInt_j->GetDVLPosition(b,avg_v));
    const Eigen::Vector3d P_dvl_est= (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c
                                      + R_dvl_c *(VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc));
    const Eigen::Matrix3d R_est = R_b_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_b;

    Eigen::Vector3d err_vi = vdi - dV_di;
    Eigen::Vector3d err_vj = vdj - dV_dj;
    Eigen::Vector3d err_P = P_dvl_est - dP_dvl;
    _error<<err_vi, err_vj, err_P;
}

EdgeTrajAlign::EdgeTrajAlign()
{
    resize(3);
}

void EdgeTrajAlign::computeError()
{
    //estimation pose
    const auto * VP1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    //gt pose
    const auto * VP2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    //align matrix
    const auto * VP_align = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[2]);

    Eigen::Isometry3d T_e0_ci = VP1->estimate();
    Eigen::Isometry3d T_g0_ci = VP2->estimate();
    Eigen::Isometry3d T_g0_e0 = VP_align->estimate();

    Eigen::Isometry3d T_g0_ci_estimated =  T_g0_e0 * T_e0_ci;
    Eigen::Isometry3d diff = T_g0_ci.inverse() * T_g0_ci_estimated;
    Eigen::Matrix3d R_diff = diff.rotation();
    Eigen::Vector3d R_diff_so3= LogSO3(R_diff);
    Eigen::Vector3d t_diff = diff.translation();

    _error<<R_diff_so3,t_diff;


}
}

BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::DvlImuCamPose)
template void ORB_SLAM3::DvlImuCamPose::serialize(boost::archive::text_oarchive & ar, const unsigned int version);
template void ORB_SLAM3::DvlImuCamPose::serialize(boost::archive::text_iarchive & ar, const unsigned int version);
template void ORB_SLAM3::DvlImuCamPose::serialize(boost::archive::binary_oarchive & ar, const unsigned int version);
template void ORB_SLAM3::DvlImuCamPose::serialize(boost::archive::binary_iarchive & ar, const unsigned int version);
G2O_REGISTER_TYPE(VertexPoseDvlIMU, VertexPoseDvlIMU)
G2O_REGISTER_TYPE(VertexGyroBias, VertexGyroBias)
G2O_REGISTER_TYPE(VertexAccBias, VertexAccBias)
G2O_REGISTER_TYPE(VertexVelocity, VertexVelocity)
G2O_REGISTER_TYPE(VertexGDir,VertexGDir)
G2O_REGISTER_TYPE(EdgeDvlIMU, EdgeDvlIMU)
G2O_REGISTER_TYPE(EdgeDvlIMU2, EdgeDvlIMU2)
G2O_REGISTER_TYPE(EdgeMonoBA_DvlGyros,EdgeMonoBA_DvlGyros)
G2O_REGISTER_TYPE(EdgeStereoBA_DvlGyros,EdgeStereoBA_DvlGyros)
G2O_REGISTER_TYPE(EdgePriorAcc,EdgePriorAcc)
G2O_REGISTER_TYPE(EdgePriorGyro,EdgePriorGyro)
G2O_REGISTER_TYPE(EdgeDvlVelocity,EdgeDvlVelocity)
G2O_REGISTER_TYPE(EdgeSE3DVLIMU, EdgeSE3DVLIMU)
