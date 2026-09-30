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

#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/factory.h"

#include <opencv2/core/core.hpp>

#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <Frame.h>
#include <KeyFrame.h>
#include <DVLGroPreIntegration.h>

#include "Converter.h"
#include <math.h>

namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class GeometricCamera;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef Eigen::Matrix<double, 9, 1> Vector9d;

typedef Eigen::Matrix<double, 12, 1> Vector12d;

typedef Eigen::Matrix<double, 15, 1> Vector15d;

typedef Eigen::Matrix<double, 12, 12> Matrix12d;

typedef Eigen::Matrix<double, 15, 15> Matrix15d;

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w);

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);

Eigen::Matrix3d Skew(const Eigen::Vector3d &w);
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);

Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d &R);

class ImuCamPose
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ImuCamPose()
	{}
	ImuCamPose(KeyFrame *pKF);
	ImuCamPose(Frame *pF);
	ImuCamPose(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame *pKF);

	void SetParam(const std::vector<Eigen::Matrix3d> &_Rcw,
	              const std::vector<Eigen::Vector3d> &_tcw,
	              const std::vector<Eigen::Matrix3d> &_Rbc,
	              const std::vector<Eigen::Vector3d> &_tbc,
	              const double &_bf);

	void Update(const double *pu);                                                   // update in the imu reference
	void UpdateW(const double *pu);                                                  // update in the world reference
	Eigen::Vector2d Project(const Eigen::Vector3d &Xw, int cam_idx = 0) const;       // Mono
	Eigen::Vector3d ProjectStereo(const Eigen::Vector3d &Xw, int cam_idx = 0) const; // Stereo
	bool isDepthPositive(const Eigen::Vector3d &Xw, int cam_idx = 0) const;

public:
	// For IMU
	Eigen::Matrix3d Rwb;
	Eigen::Vector3d twb;

	// For set of cameras
	std::vector<Eigen::Matrix3d> Rcw;
	std::vector<Eigen::Vector3d> tcw;
	std::vector<Eigen::Matrix3d> Rcb, Rbc;
	std::vector<Eigen::Vector3d> tcb, tbc;
	double bf;
	std::vector<GeometricCamera *> pCamera;

	// For posegraph 4DoF
	Eigen::Matrix3d Rwb0;
	Eigen::Matrix3d DR;

	int its;
};

class DvlImuCamPose
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	DvlImuCamPose()
	{}
	DvlImuCamPose(KeyFrame *pKF);
	DvlImuCamPose(Frame *pF);
    DvlImuCamPose(const DvlImuCamPose& dic);
//		GyroDvlCamPose(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame *pKF);

//		void SetParam(const std::vector<Eigen::Matrix3d> &_Rcw, const std::vector<Eigen::Vector3d> &_tcw, const std::vector<Eigen::Matrix3d> &_Rbc,
//					  const std::vector<Eigen::Vector3d> &_tbc, const double &_bf);

	void Update(const double *pu);                                                   // update in the imu reference
    void Reset();
//		void UpdateW(const double *pu);                                                  // update in the world reference
	Eigen::Vector2d Project(const Eigen::Vector3d &Xw, int cam_idx = 0) const;       // Mono
	Eigen::Vector3d ProjectStereo(const Eigen::Vector3d &Xw, int cam_idx = 0) const; // Stereo
//		bool isDepthPositive(const Eigen::Vector3d &Xw, int cam_idx = 0) const;
    void write(std::ofstream &fout);
    void read(std::ifstream &fin);

public:
	// for pose update
	//R_c0_cj
	Eigen::Matrix3d Rwc;
	//c0_t_c0_cj
	Eigen::Vector3d twc;

	// For set of cameras
	//R_cj_c0
	std::vector<Eigen::Matrix3d> Rcw;
	//cj_t_cj_c0
	std::vector<Eigen::Vector3d> tcw;
	std::vector<Eigen::Matrix3d> R_c_gyro, R_gyro_c, R_c_dvl, R_dvl_c;
	std::vector<Eigen::Vector3d> t_c_gyro, t_gyro_c, t_c_dvl, t_dvl_c;
	Eigen::Isometry3d T_r_l;
	double bf;
	std::vector<GeometricCamera *> pCamera;

	// For posegraph 4DoF
//		Eigen::Matrix3d Rwb0;
//		Eigen::Matrix3d DR;

	int its;

    int mCamNum;
    double mTimestamp;
    bool mPoorVision;
};

class InvDepthPoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	InvDepthPoint()
	{}
	InvDepthPoint(double _rho, double _u, double _v, KeyFrame *pHostKF);

	void Update(const double *pu);

	double rho;
	double u, v; // they are not variables, observation in the host frame

	double fx, fy, cx, cy, bf; // from host frame

	int its;
};

// Optimizable parameters are IMU pose
class VertexPose: public g2o::BaseVertex<6, ImuCamPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexPose()
	{}
	VertexPose(KeyFrame *pKF)
	{
		setEstimate(ImuCamPose(pKF));
	}
	VertexPose(Frame *pF)
	{
		setEstimate(ImuCamPose(pF));
	}

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate.Update(update_);
		updateCache();
	}
};

class VertexPoseDvlIMU: public g2o::BaseVertex<6, DvlImuCamPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexPoseDvlIMU()
	{}
	VertexPoseDvlIMU(KeyFrame *pKF)
	{
		setEstimate(DvlImuCamPose(pKF));
	}
	VertexPoseDvlIMU(Frame *pF)
	{
		setEstimate(DvlImuCamPose(pF));
	}

    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

	virtual void setToOriginImpl()
	{
        _estimate.Reset();
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate.Update(update_);
		updateCache();
	}
};

class VertexPose4DoF: public g2o::BaseVertex<4, ImuCamPose>
{
	// Translation and yaw are the only optimizable variables
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexPose4DoF()
	{}
	VertexPose4DoF(KeyFrame *pKF)
	{
		setEstimate(ImuCamPose(pKF));
	}
	VertexPose4DoF(Frame *pF)
	{
		setEstimate(ImuCamPose(pF));
	}
	VertexPose4DoF(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame *pKF)
	{

		setEstimate(ImuCamPose(_Rwc, _twc, pKF));
	}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		double update6DoF[6];
		update6DoF[0] = 0;
		update6DoF[1] = 0;
		update6DoF[2] = update_[0];
		update6DoF[3] = update_[1];
		update6DoF[4] = update_[2];
		update6DoF[5] = update_[3];
		_estimate.UpdateW(update6DoF);
		updateCache();
	}
};

class VertexVelocity: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexVelocity()
	{}
	VertexVelocity(KeyFrame *pKF);
	VertexVelocity(Frame *pF);

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		Eigen::Vector3d uv;
		uv << update_[0], update_[1], update_[2];
		setEstimate(estimate() + uv);
		updateCache();
	}
};

class VertexGyroBias: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexGyroBias()
	{}
    VertexGyroBias(const Eigen::Vector3d &bg)
    {
        setEstimate(bg);
    }
	VertexGyroBias(KeyFrame *pKF);
	VertexGyroBias(Frame *pF);

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		Eigen::Vector3d ubg;
		_estimate(0) += update_[0];
		_estimate(1) += update_[1];
		_estimate(2) += update_[2];
		updateCache();
	}
};

class VertexAccBias: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexAccBias()
	{ setEstimate(Eigen::Vector3d::Zero());}
    VertexAccBias(const Eigen::Vector3d &ba)
    {
        setEstimate(ba);
    }
	VertexAccBias(KeyFrame *pKF);
	VertexAccBias(Frame *pF);

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		Eigen::Vector3d uba;
		uba << update_[0], update_[1], update_[2];
		setEstimate(estimate() + uba);
	}
};

class VertexDVLBeamOritenstion: public g2o::BaseVertex<8, Eigen::Matrix<double, 8, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexDVLBeamOritenstion()
	{}
	VertexDVLBeamOritenstion(const Eigen::Matrix<double, 8, 1> &r)
	{
		setEstimate(Eigen::Matrix<double, 8, 1>(r));
	}

	virtual bool read(std::istream &is)
	{}
	virtual bool write(std::ostream &os) const
	{}

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate(0) += update_[0];
		_estimate(1) += update_[1];
		_estimate(2) += update_[2];
		_estimate(3) += update_[3];
		_estimate(4) += update_[4];
		_estimate(5) += update_[5];
		_estimate(6) += update_[6];
		_estimate(7) += update_[7];
		updateCache();
	}
};

// Gravity direction vertex
class GDirection
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	GDirection()
	{}
	GDirection(Eigen::Matrix3d pRwg)
		: Rwg(pRwg)
	{}
    GDirection(const GDirection &pGDir)
        : Rwg(pGDir.Rwg){}


	void Update(const double *pu)
	{
		Rwg = Rwg * ExpSO3(pu[0], pu[1], 0.0);
	}

	Eigen::Matrix3d Rwg;
};

class VertexGDir: public g2o::BaseVertex<2, GDirection>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexGDir()
	{setEstimate(GDirection(Eigen::Matrix3d::Identity()));}
	VertexGDir(Eigen::Matrix3d pRwg)
	{
		setEstimate(GDirection(pRwg));
	}

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate.Update(update_);
		updateCache();
	}
};

// scale vertex
class VertexScale: public g2o::BaseVertex<1, double>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexScale()
	{
		setEstimate(1.0);
	}
	VertexScale(double ps)
	{
		setEstimate(ps);
	}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	virtual void setToOriginImpl()
	{
		setEstimate(1.0);
	}

	virtual void oplusImpl(const double *update_)
	{
		setEstimate(estimate() * exp(*update_));
	}
};

// Inverse depth point (just one parameter, inverse depth at the host frame)
class VertexInvDepth: public g2o::BaseVertex<1, InvDepthPoint>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexInvDepth()
	{}
	VertexInvDepth(double invDepth, double u, double v, KeyFrame *pHostKF)
	{
		setEstimate(InvDepthPoint(invDepth, u, v, pHostKF));
	}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	virtual void setToOriginImpl()
	{
	}

	virtual void oplusImpl(const double *update_)
	{
		_estimate.Update(update_);
		updateCache();
	}
};

class EdgeMono: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeMono(int cam_idx_ = 0)
		: cam_idx(cam_idx_)
	{
	}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const g2o::VertexSBAPointXYZ *VPoint = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
		const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
		const Eigen::Vector2d obs(_measurement);
		_error = obs - VPose->estimate().Project(VPoint->estimate(), cam_idx);
	}

	virtual void linearizeOplus();

	bool isDepthPositive()
	{
		const g2o::VertexSBAPointXYZ *VPoint = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
		const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
		return VPose->estimate().isDepthPositive(VPoint->estimate(), cam_idx);
	}

	Eigen::Matrix<double, 2, 9> GetJacobian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 2, 9> J;
		J.block<2, 3>(0, 0) = _jacobianOplusXi;
		J.block<2, 6>(0, 3) = _jacobianOplusXj;
		return J;
	}

	Eigen::Matrix<double, 9, 9> GetHessian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 2, 9> J;
		J.block<2, 3>(0, 0) = _jacobianOplusXi;
		J.block<2, 6>(0, 3) = _jacobianOplusXj;
		return J.transpose() * information() * J;
	}

public:
	const int cam_idx;
};

class EdgeMonoOnlyPose: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeMonoOnlyPose(const cv::Mat &Xw_, int cam_idx_ = 0)
		: Xw(Converter::toVector3d(Xw_)),
		  cam_idx(cam_idx_)
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
		const Eigen::Vector2d obs(_measurement);
		_error = obs - VPose->estimate().Project(Xw, cam_idx);
	}

	virtual void linearizeOplus();

	bool isDepthPositive()
	{
		const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
		return VPose->estimate().isDepthPositive(Xw, cam_idx);
	}

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

public:
	const Eigen::Vector3d Xw;
	const int cam_idx;
};
class EdgeMonoOnlyPose_DvlGyros: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPoseDvlIMU>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeMonoOnlyPose_DvlGyros(const cv::Mat &Xw_, int cam_idx_ = 0)
		: Xw(Converter::toVector3d(Xw_)),
		  cam_idx(cam_idx_)
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexPoseDvlIMU *VPose = static_cast<const VertexPoseDvlIMU *>(_vertices[0]);
		Eigen::Matrix<double, 2, 1> obs(_measurement);
		debug_pose = static_cast<const DvlImuCamPose *>(&(VPose->estimate()));
		_error = obs - VPose->estimate().Project(Xw, cam_idx);
	}

//	virtual void linearizeOplus();

//	bool isDepthPositive()
//	{
//		const VertexPoseDvlGro *VPose = static_cast<const VertexPoseDvlGro *>(_vertices[0]);
//		return VPose->estimate().isDepthPositive(Xw, cam_idx);
//	}

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

public:
	const Eigen::Vector3d Xw;
	const int cam_idx;
	const DvlImuCamPose *debug_pose;
};

class EdgeMonoBA_DvlGyros: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPoseDvlIMU, g2o::VertexSBAPointXYZ>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeMonoBA_DvlGyros(int cam_idx_ = 0)
		: cam_idx(cam_idx_)
	{}

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	void computeError()
	{
		const VertexPoseDvlIMU *VPose = static_cast<const VertexPoseDvlIMU *>(_vertices[0]);
		const g2o::VertexSBAPointXYZ *Vmp = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[1]);
		Eigen::Matrix<double, 2, 1> obs(_measurement);
		debug_pose = static_cast<const DvlImuCamPose *>(&(VPose->estimate()));
		_error = obs - VPose->estimate().Project(Vmp->estimate(), cam_idx);
	}

//	virtual void linearizeOplus();

//	bool isDepthPositive()
//	{
//		const VertexPoseDvlGro *VPose = static_cast<const VertexPoseDvlGro *>(_vertices[0]);
//		return VPose->estimate().isDepthPositive(Xw, cam_idx);
//	}

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

public:
	const int cam_idx;
	const DvlImuCamPose *debug_pose;
};

class EdgeStereoBA_DvlGyros: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPoseDvlIMU, g2o::VertexSBAPointXYZ>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeStereoBA_DvlGyros(int cam_idx_ = 0)
		: cam_idx(cam_idx_)
	{}

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	void computeError()
	{
		const VertexPoseDvlIMU *VPose = static_cast<const VertexPoseDvlIMU *>(_vertices[0]);
		const g2o::VertexSBAPointXYZ *Vmp = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[1]);
		Eigen::Matrix<double, 3, 1> obs(_measurement);
		// debug_pose = static_cast<const DvlImuCamPose *>(&(VPose->estimate()));
		_error = obs - VPose->estimate().ProjectStereo(Vmp->estimate(), cam_idx);
	}
//	virtual void linearizeOplus();

//	bool isDepthPositive()
//	{
//		const VertexPoseDvlGro *VPose = static_cast<const VertexPoseDvlGro *>(_vertices[0]);
//		return VPose->estimate().isDepthPositive(Xw, cam_idx);
//	}

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

public:
	const int cam_idx;
	const DvlImuCamPose *debug_pose;
};

class EdgeStereo: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeStereo(int cam_idx_ = 0)
		: cam_idx(cam_idx_)
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const g2o::VertexSBAPointXYZ *VPoint = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
		const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[1]);
		const Eigen::Vector3d obs(_measurement);
		_error = obs - VPose->estimate().ProjectStereo(VPoint->estimate(), cam_idx);
	}

	virtual void linearizeOplus();

	Eigen::Matrix<double, 3, 9> GetJacobian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 3, 9> J;
		J.block<3, 3>(0, 0) = _jacobianOplusXi;
		J.block<3, 6>(0, 3) = _jacobianOplusXj;
		return J;
	}

	Eigen::Matrix<double, 9, 9> GetHessian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 3, 9> J;
		J.block<3, 3>(0, 0) = _jacobianOplusXi;
		J.block<3, 6>(0, 3) = _jacobianOplusXj;
		return J.transpose() * information() * J;
	}

public:
	const int cam_idx;
};

class EdgeStereoOnlyPose: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeStereoOnlyPose(const cv::Mat &Xw_, int cam_idx_ = 0)
		: Xw(Converter::toVector3d(Xw_)), cam_idx(cam_idx_)
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);
		const Eigen::Vector3d obs(_measurement);
		_error = obs - VPose->estimate().ProjectStereo(Xw, cam_idx);
	}

	virtual void linearizeOplus();

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

public:
	const Eigen::Vector3d Xw; // 3D point coordinates
	const int cam_idx;
};

class EdgeStereoOnlyPose_DvlGyros: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPoseDvlIMU>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeStereoOnlyPose_DvlGyros(const cv::Mat &Xw_, int cam_idx_ = 0)
		: Xw(Converter::toVector3d(Xw_)), cam_idx(cam_idx_)
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexPoseDvlIMU *VPose = static_cast<const VertexPoseDvlIMU *>(_vertices[0]);
		const Eigen::Vector3d obs(_measurement);
		_error = obs - VPose->estimate().ProjectStereo(Xw, cam_idx);
	}

//	virtual void linearizeOplus();

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

public:
	const Eigen::Vector3d Xw; // 3D point coordinates
	const int cam_idx;
};

class EdgeInertial: public g2o::BaseMultiEdge<9, Vector9d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeInertial(IMU::Preintegrated *pInt);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
	virtual void linearizeOplus();

	Eigen::Matrix<double, 24, 24> GetHessian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 24> J;
		J.block<9, 6>(0, 0) = _jacobianOplus[0];
		J.block<9, 3>(0, 6) = _jacobianOplus[1];
		J.block<9, 3>(0, 9) = _jacobianOplus[2];
		J.block<9, 3>(0, 12) = _jacobianOplus[3];
		J.block<9, 6>(0, 15) = _jacobianOplus[4];
		J.block<9, 3>(0, 21) = _jacobianOplus[5];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 18, 18> GetHessianNoPose1()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 18> J;
		J.block<9, 3>(0, 0) = _jacobianOplus[1];
		J.block<9, 3>(0, 3) = _jacobianOplus[2];
		J.block<9, 3>(0, 6) = _jacobianOplus[3];
		J.block<9, 6>(0, 9) = _jacobianOplus[4];
		J.block<9, 3>(0, 15) = _jacobianOplus[5];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 9, 9> GetHessian2()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 9> J;
		J.block<9, 6>(0, 0) = _jacobianOplus[4];
		J.block<9, 3>(0, 6) = _jacobianOplus[5];
		return J.transpose() * information() * J;
	}

	const Eigen::Matrix3d JRg, JVg, JPg;
	const Eigen::Matrix3d JVa, JPa;
	IMU::Preintegrated *mpInt;
	const double dt;
	Eigen::Vector3d g;
};

// Edge inertial whre gravity is included as optimizable variable and it is not supposed to be pointing in -z axis, as well as scale
class EdgeInertialGS: public g2o::BaseMultiEdge<9, Vector9d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// EdgeInertialGS(IMU::Preintegrated* pInt);
	EdgeInertialGS(IMU::Preintegrated *pInt);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
	virtual void linearizeOplus();

	const Eigen::Matrix3d JRg, JVg, JPg;
	const Eigen::Matrix3d JVa, JPa;
	IMU::Preintegrated *mpInt;
	const double dt;
	Eigen::Vector3d g, gI;

	Eigen::Matrix<double, 27, 27> GetHessian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 27> J;
		J.block<9, 6>(0, 0) = _jacobianOplus[0];
		J.block<9, 3>(0, 6) = _jacobianOplus[1];
		J.block<9, 3>(0, 9) = _jacobianOplus[2];
		J.block<9, 3>(0, 12) = _jacobianOplus[3];
		J.block<9, 6>(0, 15) = _jacobianOplus[4];
		J.block<9, 3>(0, 21) = _jacobianOplus[5];
		J.block<9, 2>(0, 24) = _jacobianOplus[6];
		J.block<9, 1>(0, 26) = _jacobianOplus[7];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 27, 27> GetHessian2()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 27> J;
		J.block<9, 3>(0, 0) = _jacobianOplus[2];
		J.block<9, 3>(0, 3) = _jacobianOplus[3];
		J.block<9, 2>(0, 6) = _jacobianOplus[6];
		J.block<9, 1>(0, 8) = _jacobianOplus[7];
		J.block<9, 3>(0, 9) = _jacobianOplus[1];
		J.block<9, 3>(0, 12) = _jacobianOplus[5];
		J.block<9, 6>(0, 15) = _jacobianOplus[0];
		J.block<9, 6>(0, 21) = _jacobianOplus[4];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 9, 9> GetHessian3()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 9> J;
		J.block<9, 3>(0, 0) = _jacobianOplus[2];
		J.block<9, 3>(0, 3) = _jacobianOplus[3];
		J.block<9, 2>(0, 6) = _jacobianOplus[6];
		J.block<9, 1>(0, 8) = _jacobianOplus[7];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 1, 1> GetHessianScale()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 1> J = _jacobianOplus[7];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 3, 3> GetHessianBiasGyro()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 3> J = _jacobianOplus[2];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 3, 3> GetHessianBiasAcc()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 3> J = _jacobianOplus[3];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 2, 2> GetHessianGDir()
	{
		linearizeOplus();
		Eigen::Matrix<double, 9, 2> J = _jacobianOplus[6];
		return J.transpose() * information() * J;
	}
};

class EdgeGyroRW: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeGyroRW()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexGyroBias *VG1 = static_cast<const VertexGyroBias *>(_vertices[0]);
		const VertexGyroBias *VG2 = static_cast<const VertexGyroBias *>(_vertices[1]);
		_error = VG2->estimate() - VG1->estimate();
	}

	virtual void linearizeOplus()
	{
		_jacobianOplusXi = -Eigen::Matrix3d::Identity();
		_jacobianOplusXj.setIdentity();
	}

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 3, 6> J;
		J.block<3, 3>(0, 0) = _jacobianOplusXi;
		J.block<3, 3>(0, 3) = _jacobianOplusXj;
		return J.transpose() * information() * J;
	}

	Eigen::Matrix3d GetHessian2()
	{
		linearizeOplus();
		return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
	}
};

class EdgeAccRW: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeAccRW()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexAccBias *VA1 = static_cast<const VertexAccBias *>(_vertices[0]);
		const VertexAccBias *VA2 = static_cast<const VertexAccBias *>(_vertices[1]);
		_error = VA2->estimate() - VA1->estimate();
	}

	virtual void linearizeOplus()
	{
		_jacobianOplusXi = -Eigen::Matrix3d::Identity();
		_jacobianOplusXj.setIdentity();
	}

	Eigen::Matrix<double, 6, 6> GetHessian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 3, 6> J;
		J.block<3, 3>(0, 0) = _jacobianOplusXi;
		J.block<3, 3>(0, 3) = _jacobianOplusXj;
		return J.transpose() * information() * J;
	}

	Eigen::Matrix3d GetHessian2()
	{
		linearizeOplus();
		return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
	}
};

class ConstraintPoseImu
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ConstraintPoseImu(const Eigen::Matrix3d &Rwb_, const Eigen::Vector3d &twb_, const Eigen::Vector3d &vwb_,
	                  const Eigen::Vector3d &bg_, const Eigen::Vector3d &ba_, const Matrix15d &H_)
		: Rwb(Rwb_), twb(twb_), vwb(vwb_), bg(bg_), ba(ba_), H(H_)
	{
		H = (H + H) / 2;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
		Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
		for (int i = 0; i < 15; i++)
			if (eigs[i] < 1e-12) {
				eigs[i] = 0;
			}
		H = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
	}
	ConstraintPoseImu(const cv::Mat &Rwb_, const cv::Mat &twb_, const cv::Mat &vwb_,
	                  const IMU::Bias &b, const cv::Mat &H_)
	{
		Rwb = Converter::toMatrix3d(Rwb_);
		twb = Converter::toVector3d(twb_);
		vwb = Converter::toVector3d(vwb_);
		bg << b.bwx, b.bwy, b.bwz;
		ba << b.bax, b.bay, b.baz;
		for (int i = 0; i < 15; i++)
			for (int j = 0; j < 15; j++)
				H(i, j) = H_.at<float>(i, j);
		H = (H + H) / 2;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
		Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
		for (int i = 0; i < 15; i++)
			if (eigs[i] < 1e-12) {
				eigs[i] = 0;
			}
		H = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
	}

	Eigen::Matrix3d Rwb;
	Eigen::Vector3d twb;
	Eigen::Vector3d vwb;
	Eigen::Vector3d bg;
	Eigen::Vector3d ba;
	Matrix15d H;
};

class EdgePriorPoseImu: public g2o::BaseMultiEdge<15, Vector15d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgePriorPoseImu(ConstraintPoseImu *c);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
	virtual void linearizeOplus();

	Eigen::Matrix<double, 15, 15> GetHessian()
	{
		linearizeOplus();
		Eigen::Matrix<double, 15, 15> J;
		J.block<15, 6>(0, 0) = _jacobianOplus[0];
		J.block<15, 3>(0, 6) = _jacobianOplus[1];
		J.block<15, 3>(0, 9) = _jacobianOplus[2];
		J.block<15, 3>(0, 12) = _jacobianOplus[3];
		return J.transpose() * information() * J;
	}

	Eigen::Matrix<double, 9, 9> GetHessianNoPose()
	{
		linearizeOplus();
		Eigen::Matrix<double, 15, 9> J;
		J.block<15, 3>(0, 0) = _jacobianOplus[1];
		J.block<15, 3>(0, 3) = _jacobianOplus[2];
		J.block<15, 3>(0, 6) = _jacobianOplus[3];
		return J.transpose() * information() * J;
	}
	Eigen::Matrix3d Rwb;
	Eigen::Vector3d twb, vwb;
	Eigen::Vector3d bg, ba;
};

// Priors for biases
class EdgePriorAcc: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexAccBias>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorAcc():bprior(Eigen::Vector3d::Zero()){}
	EdgePriorAcc(const cv::Mat &bprior_)
		: bprior(Converter::toVector3d(bprior_))
	{}

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	void computeError()
	{
		const VertexAccBias *VA = static_cast<const VertexAccBias *>(_vertices[0]);
		_error = bprior - VA->estimate();
	}
	virtual void linearizeOplus();

	Eigen::Matrix<double, 3, 3> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

	Eigen::Vector3d bprior;
};

class EdgePriorGyro: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexGyroBias>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorGyro():bprior(Eigen::Vector3d::Zero()){}
	EdgePriorGyro(const cv::Mat &bprior_)
		: bprior(Converter::toVector3d(bprior_))
	{}

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	void computeError()
	{
		const VertexGyroBias *VG = static_cast<const VertexGyroBias *>(_vertices[0]);
		_error = bprior - VG->estimate();
	}
	virtual void linearizeOplus();

	Eigen::Matrix<double, 3, 3> GetHessian()
	{
		linearizeOplus();
		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
	}

    Eigen::Vector3d bprior;
};

class Edge4DoF: public g2o::BaseBinaryEdge<6, Vector6d, VertexPose4DoF, VertexPose4DoF>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Edge4DoF(const Eigen::Matrix4d &deltaT)
	{
		dTij = deltaT;
		dRij = deltaT.block<3, 3>(0, 0);
		dtij = deltaT.block<3, 1>(0, 3);
	}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexPose4DoF *VPi = static_cast<const VertexPose4DoF *>(_vertices[0]);
		const VertexPose4DoF *VPj = static_cast<const VertexPose4DoF *>(_vertices[1]);
		_error << LogSO3(VPi->estimate().Rcw[0] * VPj->estimate().Rcw[0].transpose() * dRij.transpose()),
			VPi->estimate().Rcw[0] * (-VPj->estimate().Rcw[0].transpose() * VPj->estimate().tcw[0])
				+ VPi->estimate().tcw[0] - dtij;
	}

	// virtual void linearizeOplus(); // numerical implementation

	Eigen::Matrix4d dTij;
	Eigen::Matrix3d dRij;
	Eigen::Vector3d dtij;
};

class EdgeSE3DVLPoseOnly: public g2o::BaseUnaryEdge<3, double, g2o::VertexSE3Expmap>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3DVLPoseOnly()
	{
		mT_ei_e0 = Eigen::Isometry3d::Identity();
		mT_ej_e0 = Eigen::Isometry3d::Identity();
	}
	EdgeSE3DVLPoseOnly(Eigen::Isometry3d T_ej_e0,
	                   Eigen::Isometry3d T_ei_e0,
	                   Eigen::Isometry3d T_ci_c0,
	                   Eigen::Isometry3d T_e_c,
	                   Eigen::Vector3d p_c0)
		: mT_ej_e0(T_ej_e0), mT_ei_e0(T_ei_e0), mT_ci_c0(T_ci_c0), mT_e_c(T_e_c), mP_c0(p_c0)
	{}

	virtual void computeError()
	{
		// T_ej_e0* mT_e_c * P_c0
		// T_ej_e0 * P_e0 = P_ej
		// T_ej_e0 * T_e0_ei * T_ei_c0
		Eigen::Isometry3d T_ej_c0_ekf = mT_ej_e0 * mT_ei_e0.inverse() * mT_e_c * mT_ci_c0;
		Eigen::Vector3d p_cur_EKF = T_ej_c0_ekf * mP_c0;
		const g2o::VertexSE3Expmap *v = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);


		// T_cj_c0
		// need to convert from camera to EKF
		Eigen::Isometry3d T_cj_c0 = Eigen::Isometry3d(v->estimate());
		Eigen::Isometry3d T_ej_c0_BA = mT_e_c * T_cj_c0;
		// T_e_c * T_cj_c0 * P_c0
		// =T_ej_c0 * P_c0=P_ej
		Eigen::Vector3d p_cur_BA = T_ej_c0_BA * mP_c0;

//		Eigen::Matrix3d R_EKF=T_ej_c0_ekf.rotation();
//		Eigen::Matrix3d R_BA=T_ej_c0_BA.rotation();
//		Eigen::Vector3d error_r=LogSO3(R_EKF*R_EKF.inverse());
//		Eigen::Vector3d error_t=T_ej_c0_ekf.translation()-T_ej_c0_BA.translation();

		_error = p_cur_EKF - p_cur_BA;
//		_error<<error_r,error_t;
	}

	virtual bool read(std::istream &in)
	{}
	virtual bool write(std::ostream &out) const
	{}

protected:
	// mT_prev: T_i_w
	Eigen::Isometry3d mT_ei_e0;
	// mT_cur: T_j_w
	Eigen::Isometry3d mT_ej_e0;
	Eigen::Isometry3d mT_ci_c0;

	Eigen::Isometry3d mT_e_c;
	// 3d point position in the world(under EKF coordinate)
	// in orb slam the origin of the world is the position of the first camera frame
	Eigen::Vector3d mP_c0;
};

class EdgeSE3DVLIMU: public g2o::BaseBinaryEdge<6, double, VertexPoseDvlIMU, VertexPoseDvlIMU>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3DVLIMU()
    {
        mT_ci_cj = Eigen::Isometry3d::Identity();
    }
    EdgeSE3DVLIMU(Eigen::Isometry3d T_ci_cj):mT_ci_cj(T_ci_cj)
    {}


    virtual void computeError();

    virtual bool read(std::istream &in);
    virtual bool write(std::ostream &out) const;

protected:
    Eigen::Isometry3d mT_ci_cj;
};

class EdgeSE3DVLPoseOnly2: public g2o::BaseUnaryEdge<6, double, g2o::VertexSE3Expmap>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3DVLPoseOnly2()
	{
		mT_ei_e0 = Eigen::Isometry3d::Identity();
		mT_ej_e0 = Eigen::Isometry3d::Identity();
	}
	EdgeSE3DVLPoseOnly2(Eigen::Isometry3d T_ej_e0,
	                    Eigen::Isometry3d T_ei_e0,
	                    Eigen::Isometry3d T_ci_c0,
	                    Eigen::Isometry3d T_e_c)
		: mT_ej_e0(T_ej_e0), mT_ei_e0(T_ei_e0), mT_ci_c0(T_ci_c0), mT_e_c(T_e_c)
	{}

	virtual void computeError()
	{
		// T_ej_e0* mT_e_c * P_c0
		// T_ej_e0 * P_e0 = P_ej
		// T_ej_e0 * T_e0_ei * T_ei_c0
		Eigen::Isometry3d T_cj_c0_ekf = mT_e_c.inverse() * mT_ej_e0 * mT_ei_e0.inverse() * mT_e_c * mT_ci_c0;
		const g2o::VertexSE3Expmap *v = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);


		// T_cj_c0
		// need to convert from camera to EKF
		Eigen::Isometry3d T_cj_c0 = Eigen::Isometry3d(v->estimate());
		Eigen::Isometry3d T_cj_c0_BA = T_cj_c0;
		// T_e_c * T_cj_c0 * P_c0
		// =T_ej_c0 * P_c0=P_ej

		Eigen::Matrix3d R_EKF = T_cj_c0_ekf.rotation();
		Eigen::Matrix3d R_BA = T_cj_c0_BA.rotation();
		Eigen::Vector3d error_r = LogSO3(R_EKF * R_BA.inverse());
		Eigen::Vector3d error_t = T_cj_c0_ekf.translation() - T_cj_c0_BA.translation();

//            _error = p_cur_EKF - p_cur_BA;
		_error << error_r, error_t;
	}

	virtual bool read(std::istream &in)
	{}
	virtual bool write(std::ostream &out) const
	{}

protected:
	// mT_prev: T_i_w
	Eigen::Isometry3d mT_ei_e0;
	// mT_cur: T_j_w
	Eigen::Isometry3d mT_ej_e0;
	Eigen::Isometry3d mT_ci_c0;

	Eigen::Isometry3d mT_e_c;
	// 3d point position in the world(under EKF coordinate)
	// in orb slam the origin of the world is the position of the first camera frame
	Eigen::Vector3d mP_c0;
};

// dimension of error: 6 (3 for translation, 3 for rotation)
// datatype of measurement: Eigen::Matrix<double,3,1>
// datatype of two vertex: g2o::VertexSE3Expmap(0: camera pose), g2o::VertexSE3Expmap(1: tf from EKF to camera)

class EdgeSE3DVLBA: public g2o::BaseBinaryEdge<6,
                                               Eigen::Matrix<double, 3, 1>,
                                               g2o::VertexSE3Expmap,
                                               g2o::VertexSE3Expmap>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3DVLBA()
	{
		mT_ei_ej = Eigen::Isometry3d::Identity();

	}
	EdgeSE3DVLBA(Eigen::Isometry3d T_ei_ej, Eigen::Isometry3d T_e_c)
		: mT_ei_ej(T_ei_ej), mT_e_c(T_e_c)
	{}

	virtual void computeError()
	{
		// T_ej_e0* mT_e_c * P_c0
		// T_ej_e0 * P_e0 = P_ej
		// T_ej_e0 * T_e0_ei * T_ei_c0
//		Eigen::Isometry3d T_ej_c0 = mT_ej_e0 * mT_ei_e0.inverse() * mT_e_c * mT_ci_c0;
//		Eigen::Vector3d p_cur_EKF = T_ej_c0 * mP_c0;
		const g2o::VertexSE3Expmap *v_i = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
		const g2o::VertexSE3Expmap *v_j = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);


		// T_cj_c0
		// need to convert from camera to EKF
		Eigen::Isometry3d T_ci_c0 = Eigen::Isometry3d(v_i->estimate());
		Eigen::Isometry3d T_cj_c0 = Eigen::Isometry3d(v_j->estimate());

//		cout<<"in edge: "<<endl;
//		cout<<"t_cj_c0: "<<T_cj_c0.translation()<<endl;
//		cout<<"r_cj_c0: "<<T_cj_c0.rotation()<<endl;


		Eigen::Isometry3d T_ci_cj_orb = T_cj_c0 * T_cj_c0.inverse();
		Eigen::Isometry3d T_ci_cj_ekf = mT_e_c.inverse() * (mT_ei_ej) * mT_e_c;

		Eigen::Matrix3d R_ci_cj_orb = T_ci_cj_orb.rotation();
		Eigen::Matrix3d R_ci_cj_ekf = T_ci_cj_ekf.rotation();
		Eigen::Vector3d e_r = LogSO3(R_ci_cj_orb * R_ci_cj_ekf.inverse());

		Eigen::Vector3d t_ci_cj_orb = T_ci_cj_orb.translation();
		Eigen::Vector3d t_ci_cj_ekf = T_ci_cj_ekf.translation();
		Eigen::Vector3d e_t = t_ci_cj_orb - t_ci_cj_ekf;

		_error << e_r, e_t;

	}

	virtual bool read(std::istream &in)
	{}
	virtual bool write(std::ostream &out) const
	{}

protected:
	// mT_prev: T_i_w
//	Eigen::Isometry3d mT_ei_e0;
	// mT_cur: T_j_w
//	Eigen::Isometry3d mT_ej_e0;

	Eigen::Isometry3d mT_ei_ej;
//	Eigen::Isometry3d mT_ci_c0;

	Eigen::Isometry3d mT_e_c;
	// 3d point position in the world(under EKF coordinate)
	// in orb slam the origin of the world is the position of the first camera frame
	Eigen::Vector3d mP_c0;
};

class EdgeDVLRefine: public g2o::BaseUnaryEdge<6, double, g2o::VertexSE3Expmap>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeDVLRefine()
	{
		mT_ei_ej = Eigen::Isometry3d::Identity();

	}
	EdgeDVLRefine(const Eigen::Isometry3d &T_ei_ej, const Eigen::Isometry3d &T_ci_c0, const Eigen::Isometry3d &T_cj_c0)
		: mT_ei_ej(T_ei_ej), mT_ci_c0(T_ci_c0), mT_cj_c0(T_cj_c0)
	{}

	virtual void computeError()
	{
		// T_ej_e0* mT_e_c * P_c0
		// T_ej_e0 * P_e0 = P_ej
		// T_ej_e0 * T_e0_ei * T_ei_c0
//		Eigen::Isometry3d T_ej_c0 = mT_ej_e0 * mT_ei_e0.inverse() * mT_e_c * mT_ci_c0;
//		Eigen::Vector3d p_cur_EKF = T_ej_c0 * mP_c0;
//		const g2o::VertexSE3Expmap *v_i = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
//		const g2o::VertexSE3Expmap *v_j = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
		const g2o::VertexSE3Expmap *v_e_c = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);


		// T_cj_c0
		// need to convert from camera to EKF
//		Eigen::Isometry3d T_ci_c0 = Eigen::Isometry3d(v_i->estimate());
//		Eigen::Isometry3d T_cj_c0 = Eigen::Isometry3d(v_j->estimate());
//		Eigen::Isometry3d T_e_c = Eigen::Isometry3d (v_e_c->estimate());
		Eigen::Isometry3d T_e_c = v_e_c->estimate();

//		fstream file;
//		file.open("data/Edge_T.txt",ios::out|ios::app);
//		if (!file)
//			cout<<"fail to open file data/Edge_T.txt"<<endl;
//
//
//		file<<"in edge"<<endl;
//		file<<"t: "<<T_e_c.translation()<<endl;
//		file<<"r: "<<T_e_c.rotation().eulerAngles(0,1,2)<<endl;
//		file.close();

		Eigen::Isometry3d T_ci_cj_orb = mT_ci_c0 * mT_cj_c0.inverse();
		Eigen::Isometry3d T_ci_cj_ekf = T_e_c.inverse() * (mT_ei_ej) * T_e_c;

		Eigen::Matrix3d R_ci_cj_orb = T_ci_cj_orb.rotation();
		Eigen::Matrix3d R_ci_cj_ekf = T_ci_cj_ekf.rotation();
		Eigen::Vector3d e_r = LogSO3(R_ci_cj_orb * R_ci_cj_ekf.inverse());

		Eigen::Vector3d t_ci_cj_orb = T_ci_cj_orb.translation();
		Eigen::Vector3d t_ci_cj_ekf = T_ci_cj_ekf.translation();
		Eigen::Vector3d e_t = t_ci_cj_orb - t_ci_cj_ekf;

		_error << e_r, e_t;

	}

	virtual bool read(std::istream &in)
	{}
	virtual bool write(std::ostream &out) const
	{}

protected:


	Eigen::Isometry3d mT_ei_ej;

	Eigen::Isometry3d mT_ci_c0;
	Eigen::Isometry3d mT_cj_c0;

};

/***
 * vertex0: pose_i
 * vertex1: pose_j
 * vertex2: bias_gyro
 * vertex3: T_d_c
 * vertex4: T_g_d
 */
class EdgeDvlGyroTrack: public g2o::BaseMultiEdge<6, Eigen::Matrix<float, 6, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDvlGyroTrack(DVLGroPreIntegration *pInt):mpInt(pInt)
	{
		resize(5);
	}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
//	virtual void linearizeOplus();

//	const Eigen::Matrix3d JRg, JVg, JPg;
//	const Eigen::Matrix3d JVa, JPa;
	DVLGroPreIntegration *mpInt;

};

/***
 * vertex0: pose_i
 * vertex1: pose_j
 * vertex2: bias_gyro
 * vertex3: T_d_c
 * vertex4: T_g_d
 */
class EdgeDvlGyroInit: public g2o::BaseMultiEdge<6, Eigen::Matrix<float, 6, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDvlGyroInit(DVLGroPreIntegration *pInt);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
//	virtual void linearizeOplus();

//	const Eigen::Matrix3d JRg, JVg, JPg;
//	const Eigen::Matrix3d JVa, JPa;
	DVLGroPreIntegration *mpInt;
	const double dt;

};

/***
 * vertex0: pose_i
 * vertex1: pose_j
 * vertex2: bias_gyro
 * vertex3: T_d_c
 * vertex4: T_g_d
 * vertex5: beam_orientation
 */
class EdgeDvlGyroInit2: public g2o::BaseMultiEdge<6, Eigen::Matrix<float, 6, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDvlGyroInit2(DVLGroPreIntegration *pInt);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
//	virtual void linearizeOplus();

//	const Eigen::Matrix3d JRg, JVg, JPg;
//	const Eigen::Matrix3d JVa, JPa;
	DVLGroPreIntegration *mpInt;
	const double dt;

};

/***
 * vertex0: pose_i
 * vertex1: pose_j
 * vertex2: bias_gyro
 * vertex3: velocity
 * vertex4: T_d_c
 * vertex5: T_g_d
 */
class EdgeDvlGyroInit3: public g2o::BaseMultiEdge<6, Eigen::Matrix<float, 6, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDvlGyroInit3(DVLGroPreIntegration *pInt);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
//	virtual void linearizeOplus();

//	const Eigen::Matrix3d JRg, JVg, JPg;
//	const Eigen::Matrix3d JVa, JPa;
	DVLGroPreIntegration *mpInt;
	const double dt;

};

class EdgeDvlIMUWithBias: public g2o::BaseMultiEdge<9, Eigen::Matrix<double, 9, 1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeDvlIMUWithBias(DVLGroPreIntegration *pInt);

    virtual bool read(std::istream &is)
    { return false; }
    virtual bool write(std::ostream &os) const
    { return false; }

    void computeError();
    //	virtual void linearizeOplus();

    //	const Eigen::Matrix3d JRg, JVg, JPg;
    //	const Eigen::Matrix3d JVa, JPa;
    DVLGroPreIntegration *mpInt;
    const double dt;

};

class EdgeDvlVelocity: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexVelocity>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeDvlVelocity():mV(Eigen::Vector3d::Zero()){}
    EdgeDvlVelocity(const Eigen::Vector3d &v)
            : mV(v){}

    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
        const VertexVelocity *VG = static_cast<const VertexVelocity *>(_vertices[0]);
        _error = mV - VG->estimate();
    }
    virtual void linearizeOplus()
    {
        _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix<double, 3, 3> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    Eigen::Vector3d mV;
};

class EdgeDvlIMU: public g2o::BaseMultiEdge<9, Eigen::Matrix<double, 9, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeDvlIMU();
	EdgeDvlIMU(DVLGroPreIntegration *pInt);

	virtual bool read(std::istream &is);
	virtual bool write(std::ostream &os) const;

	void computeError();
    // virtual void linearizeOplus();

    const Eigen::Matrix3d JRg, JVg, JPg;
    const Eigen::Matrix3d JVa, JPa;
	DVLGroPreIntegration *mpInt;
	double dt;

};

class EdgeDvlIMU2: public g2o::BaseMultiEdge<9, Eigen::Matrix<double, 9, 1>>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeDvlIMU2(): dt(0.0),mpInt_j(NULL),mpInt_i(NULL) {resize(9);}
        EdgeDvlIMU2(DVLGroPreIntegration *pInt_i,DVLGroPreIntegration *pInt_j);

        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError();
        //	virtual void linearizeOplus();

        //	const Eigen::Matrix3d JRg, JVg, JPg;
        //	const Eigen::Matrix3d JVa, JPa;
        DVLGroPreIntegration *mpInt_i,*mpInt_j;
        double dt;

    };

class EdgeDvlIMUGravityRefine: public g2o::BaseMultiEdge<3, Eigen::Matrix<double, 3, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeDvlIMUGravityRefine(DVLGroPreIntegration *pInt);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
//	virtual void linearizeOplus();

//	const Eigen::Matrix3d JRg, JVg, JPg;
//	const Eigen::Matrix3d JVa, JPa;
	DVLGroPreIntegration *mpInt;
	const double dt;

};

class EdgeDvlIMUGravityRefineWithBias: public g2o::BaseMultiEdge<9, Eigen::Matrix<double, 9, 1>>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeDvlIMUGravityRefineWithBias(DVLGroPreIntegration *pInt);

        virtual bool read(std::istream &is)
        { return false; }
        virtual bool write(std::ostream &os) const
        { return false; }

        void computeError();
        //	virtual void linearizeOplus();

        //	const Eigen::Matrix3d JRg, JVg, JPg;
        //	const Eigen::Matrix3d JVa, JPa;
        DVLGroPreIntegration *mpInt;
        const double dt;

    };

class EdgeDvlIMUInitWithoutBias: public g2o::BaseMultiEdge<3, Eigen::Matrix<double, 3, 1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeDvlIMUInitWithoutBias(DVLGroPreIntegration *pInt);

    virtual bool read(std::istream &is)
    { return false; }
    virtual bool write(std::ostream &os) const
    { return false; }

    void computeError();
    //	virtual void linearizeOplus();

    //	const Eigen::Matrix3d JRg, JVg, JPg;
    //	const Eigen::Matrix3d JVa, JPa;
    DVLGroPreIntegration *mpInt;
    const double dt;

};

class EdgeDvlIMUInit: public g2o::BaseMultiEdge<3, Eigen::Matrix<double, 3, 1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeDvlIMUInit(DVLGroPreIntegration *pInt);

    virtual bool read(std::istream &is)
    { return false; }
    virtual bool write(std::ostream &os) const
    { return false; }

    void computeError();
    //	virtual void linearizeOplus();

    //	const Eigen::Matrix3d JRg, JVg, JPg;
    //	const Eigen::Matrix3d JVa, JPa;
    DVLGroPreIntegration *mpInt;
    const double dt;

};

class EdgeDVLBeamCalibration1: public g2o::BaseUnaryEdge<4, DVLGroPreIntegration *, VertexDVLBeamOritenstion>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDVLBeamCalibration1()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexDVLBeamOritenstion
			*V = static_cast<const VertexDVLBeamOritenstion *>(_vertices[0]);
//		const Eigen::Vector3d velocity_est(_measurement->v_debug.x,_measurement->v_debug.y,_measurement->v_debug.z);
		const Eigen::Vector3d
			velocity_est(_measurement->v_dk_visual.x(), _measurement->v_dk_visual.y(), _measurement->v_dk_visual.z());

		//r[0] alpha in paper, rotation from horizental plane
		//r[1] beta in paper, rotation arround z-axis
		Eigen::Matrix<double, 8, 1> r = V->estimate();
		std::vector<double> v_beam_est;

		Eigen::Vector4d err(0, 0, 0, 0);
//		r(1) = 45.0 / 180 * M_PI;

		for (int id = 0; id < 4; id++) {
			if (id == 0) {
				double v_beam = -velocity_est.x() * cos(r(1)) * cos(r(0)) + velocity_est.y() * sin(r(1)) * cos(r(0))
					+ sin(r(0)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.x() = e;
			}
			else if (id == 1) {
				double v_beam = -velocity_est.x() * cos(r(3)) * cos(r(2)) - velocity_est.y() * sin(r(3)) * cos(r(2))
					+ sin(r(2)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.y() = e;

			}
			else if (id == 2) {
				double v_beam = velocity_est.x() * cos(r(5)) * cos(r(4)) - velocity_est.y() * sin(r(5)) * cos(r(4))
					+ sin(r(4)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.z() = e;

			}
			else if (id == 3) {
				double v_beam = velocity_est.x() * cos(r(7)) * cos(r(6)) + velocity_est.y() * sin(r(7)) * cos(r(6))
					+ sin(r(6)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.w() = e;
			}
		}


		_error << err;
//		_error << Eigen::Matrix<double,12,1>::Identity();
	}

//	virtual void linearizeOplus();

//	Eigen::Matrix<double, 12, 12> GetHessian()
//	{
//		linearizeOplus();
//		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
//	}

public:

};

class EdgeDVLBeamCalibration2: public g2o::BaseUnaryEdge<4, DVLGroPreIntegration *, VertexDVLBeamOritenstion>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDVLBeamCalibration2()
	{}

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError()
	{
		const VertexDVLBeamOritenstion
			*V = static_cast<const VertexDVLBeamOritenstion *>(_vertices[0]);
//		const Eigen::Vector3d velocity_est(_measurement->v_debug.x,_measurement->v_debug.y,_measurement->v_debug.z);
		const Eigen::Vector3d
			velocity_est(_measurement->v_dk_dvl.x, _measurement->v_dk_dvl.y, _measurement->v_dk_dvl.z);

		//r[0] alpha in paper, rotation from horizental plane
		//r[1] beta in paper, rotation arround z-axis
		Eigen::Matrix<double, 8, 1> r = V->estimate();
		std::vector<double> v_beam_est;

		Eigen::Vector4d err(0, 0, 0, 0);
//		r(1) = 45.0 / 180 * M_PI;

		for (int id = 0; id < 4; id++) {
			if (id == 0) {
				double v_beam = -velocity_est.x() * cos(r(1)) * cos(r(0)) + velocity_est.y() * sin(r(1)) * cos(r(0))
					+ sin(r(0)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.x() = e;
			}
			else if (id == 1) {
				double v_beam = -velocity_est.x() * cos(r(3)) * cos(r(2)) - velocity_est.y() * sin(r(3)) * cos(r(2))
					+ sin(r(2)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.y() = e;

			}
			else if (id == 2) {
				double v_beam = velocity_est.x() * cos(r(5)) * cos(r(4)) - velocity_est.y() * sin(r(5)) * cos(r(4))
					+ sin(r(4)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.z() = e;

			}
			else if (id == 3) {
				double v_beam = velocity_est.x() * cos(r(7)) * cos(r(6)) + velocity_est.y() * sin(r(7)) * cos(r(6))
					+ sin(r(6)) * velocity_est.z();
				double e = _measurement->mBeams[id] - v_beam;
				err.w() = e;
			}
		}


		_error << err;
//		_error << Eigen::Matrix<double,12,1>::Identity();
	}

//	virtual void linearizeOplus();

//	Eigen::Matrix<double, 12, 12> GetHessian()
//	{
//		linearizeOplus();
//		return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
//	}

public:

};

class EdgeDvlGyroBA: public g2o::BaseMultiEdge<6, Eigen::Matrix<float, 6, 1>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeDvlGyroBA(DVLGroPreIntegration *pInt);

	virtual bool read(std::istream &is)
	{ return false; }
	virtual bool write(std::ostream &os) const
	{ return false; }

	void computeError();
	//	virtual void linearizeOplus();

	//	const Eigen::Matrix3d JRg, JVg, JPg;
	//	const Eigen::Matrix3d JVa, JPa;
	DVLGroPreIntegration *mpInt;
	const double dt;

};

class EdgeTrajAlign: public g2o::BaseMultiEdge<6, Eigen::Matrix<double, 6, 1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeTrajAlign();

    virtual bool read(std::istream &is)
    { return false; }
    virtual bool write(std::ostream &os) const
    { return false; }

    void computeError();

};

} // namespace ORB_SLAM3

#endif // G2OTYPES_H
