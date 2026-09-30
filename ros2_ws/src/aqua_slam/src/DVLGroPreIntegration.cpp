//
// Created by da on 25/01/2021.
//

#include "DVLGroPreIntegration.h"
#include<iostream>
#include <opencv2/core/eigen.hpp>
#include "ros1_compat.h"
#include <boost/serialization/export.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace std;


DVLGroPreIntegration::DVLGroPreIntegration(const Bias &b_, const Calib &calib, bool bDVL) : bDVL(bDVL)
{
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    mVelocityThreshold = 0.6f;
    Initialize(b_);
    mCalib = calib;
    calib.mT_gyro_dvl.rowRange(0, 3).colRange(0, 3).copyTo(mR_g_d);
    mR_g_d.convertTo(mR_g_d, CV_64F);
}

DVLGroPreIntegration::DVLGroPreIntegration(const Bias &b_, const Calib &calib, const cv::Point3d &v_di,
                                           bool bDVL) : bDVL(bDVL)
{
    calib.mT_gyro_dvl.rowRange(0, 3).colRange(0, 3).copyTo(mR_g_d);
    mR_g_d.convertTo(mR_g_d, CV_64F);
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(b_);
    mCalib = calib;
    SetVelocity(v_di);
    mVelocityThreshold = 0.6f;
}

DVLGroPreIntegration::DVLGroPreIntegration(const Bias &b_, const Calib &calib, const cv::Point3d &v_di,
                                           const Eigen::Vector4d &alpha, const Eigen::Vector4d &beta, bool bDVL)
        : bDVL(bDVL), mAlpha(alpha), mBeta(beta)
{
    calib.mT_gyro_dvl.rowRange(0, 3).colRange(0, 3).copyTo(mR_g_d);
    mR_g_d.convertTo(mR_g_d, CV_64F);
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(b_);
    SetVelocity(v_di);
    mCalib = calib;
    mVelocityThreshold = 0.6f;
    mE << -cos(beta(0)) * cos(alpha(0)), sin(beta(0)) * cos(alpha(0)), sin(alpha(0)), -cos(beta(1)) *
                                                                                      cos(alpha(1)),
            -sin(beta(1)) * cos(alpha(1)), sin(alpha(1)), cos(beta(2)) * cos(alpha(2)), -sin(beta(2)) *
                                                                                        cos(alpha(2)), sin(
            alpha(2)), cos(beta(3)) * cos(alpha(3)), sin(beta(3)) * cos(alpha(3)), sin(alpha(3));
    mETEInv = (mE.transpose() * mE).inverse();
}

DVLGroPreIntegration::DVLGroPreIntegration(const Bias &b_, const Calib &calib, const cv::Point3d &v_di,
                                           double velocity_threshold, bool bDVL) : bDVL(bDVL),
                                                                                   mVelocityThreshold(
                                                                                           velocity_threshold)
{
    calib.mT_gyro_dvl.rowRange(0, 3).colRange(0, 3).copyTo(mR_g_d);
    mR_g_d.convertTo(mR_g_d, CV_64F);
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(b_);
    mCalib = calib;
    SetVelocity(v_di);

}

// Copy constructor
DVLGroPreIntegration::DVLGroPreIntegration(DVLGroPreIntegration* pDVLPre) : dT(pDVLPre->dT),
                                                                            C(pDVLPre->C.clone()),
                                                                            Info(pDVLPre->Info.clone()),
                                                                            Nga(pDVLPre->Nga.clone()),
                                                                            NgaWalk(pDVLPre->NgaWalk.clone()),
                                                                            mb(pDVLPre->mb),
                                                                            mCalib(pDVLPre->mCalib),
                                                                            dR(pDVLPre->dR.clone()),
                                                                            dV(pDVLPre->dV.clone()),
                                                                            dDeltaV(pDVLPre->dDeltaV.clone()),
                                                                            dP_dvl(pDVLPre->dP_dvl.clone()),
                                                                            dP_acc(pDVLPre->dP_acc.clone()),
                                                                            JRg(pDVLPre->JRg.clone()),
                                                                            JVg(pDVLPre->JVg.clone()),
                                                                            JVa(pDVLPre->JVa.clone()),
                                                                            JPg(pDVLPre->JPg.clone()),
                                                                            JPa(pDVLPre->JPa.clone()),
                                                                            JPv(pDVLPre->JPv.clone()),
                                                                            avgA(pDVLPre->avgA.clone()),
                                                                            avgW(pDVLPre->avgW.clone()),
                                                                            bu(pDVLPre->bu),
                                                                            db(pDVLPre->db.clone()),
                                                                            mvMeasurements(
                                                                                    pDVLPre->mvMeasurements),
                                                                            mvMeasurements2(
                                                                                    pDVLPre->mvMeasurements2),
                                                                            bDVL(pDVLPre->bDVL),
                                                                            mAlpha(pDVLPre->mAlpha),
                                                                            mBeta(pDVLPre->mBeta),
                                                                            mE(pDVLPre->mE),
                                                                            mETEInv(pDVLPre->mETEInv),
                                                                            mR_g_d(pDVLPre->mR_g_d),
                                                                            mVelocity(pDVLPre->mVelocity.clone())
{

}

DVLGroPreIntegration::DVLGroPreIntegration(const DVLGroPreIntegration &integration) : dT(integration.dT),
                                                                                      C(integration.C.clone()),
                                                                                      Info(integration.Info.clone()),
                                                                                      Nga(integration.Nga.clone()),
                                                                                      NgaWalk(integration.NgaWalk.clone()),
                                                                                      mb(integration.mb),
                                                                                      mCalib(integration.mCalib),
                                                                                      dR(integration.dR.clone()),
                                                                                      dV(integration.dV.clone()),
                                                                                      dDeltaV(integration.dDeltaV.clone()),
                                                                                      dP_dvl(integration.dP_dvl.clone()),
                                                                                      dP_acc(integration.dP_acc.clone()),
                                                                                      JRg(integration.JRg.clone()),
                                                                                      JVg(integration.JVg.clone()),
                                                                                      JVa(integration.JVa.clone()),
                                                                                      JPg(integration.JPg.clone()),
                                                                                      JPa(integration.JPa.clone()),
                                                                                      JPv(integration.JPv.clone()),
                                                                                      avgA(integration.avgA.clone()),
                                                                                      avgW(integration.avgW.clone()),
                                                                                      bu(integration.bu),
                                                                                      db(integration.db.clone()),
                                                                                      mvMeasurements(
                                                                                              integration.mvMeasurements),
                                                                                      mvMeasurements2(
                                                                                              integration.mvMeasurements2),
                                                                                      bDVL(integration.bDVL),
                                                                                      mAlpha(integration.mAlpha),
                                                                                      mBeta(integration.mBeta),
                                                                                      mE(integration.mE),
                                                                                      mETEInv(integration.mETEInv),
                                                                                      mR_g_d(integration.mR_g_d),
                                                                                      mVelocity(
                                                                                              integration.mVelocity.clone())
{

}

void DVLGroPreIntegration::Initialize(const Bias &b_)
{
    dR = cv::Mat::eye(3, 3, CV_64F);
    dV = cv::Mat::zeros(3, 1, CV_64F);
    dDeltaV = cv::Mat::zeros(3, 1, CV_64F);
    mVelocity = cv::Mat::zeros(3, 1, CV_64F);
    mAngV = cv::Point3d(0, 0, 0);
    //	mR_g_d = cv::Mat::eye(3, 3, CV_64F);
    dP_dvl = cv::Mat::zeros(3, 1, CV_64F);
    dP_acc = cv::Mat::zeros(3, 1, CV_64F);
    JRg = cv::Mat::zeros(3, 3, CV_64F);
    JVg = cv::Mat::zeros(3, 3, CV_64F);
    JVa = cv::Mat::zeros(3, 3, CV_64F);
    JPg = cv::Mat::zeros(3, 3, CV_64F);
    JPa = cv::Mat::zeros(3, 3, CV_64F);
    JPv = cv::Mat::zeros(3, 3, CV_64F);
    C = cv::Mat::zeros(15, 15, CV_64F);
    Info = cv::Mat();
    db = cv::Mat::zeros(6, 1, CV_64F);
    mb = b_;
    mCalib = Calib();
    bu = b_;
    avgA = cv::Mat::zeros(3, 1, CV_64F);
    avgW = cv::Mat::zeros(3, 1, CV_64F);
    dT = 0.0f;
    mvMeasurements.clear();
    mvMeasurements2.clear();
}

void DVLGroPreIntegration::ReintegrateWithBias(const Bias &b)
{
    std::lock_guard<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements2;
    cv::Mat v = dV.clone();
    Initialize(b);
    SetVelocity(cv::Point3d(v.at<double>(0), v.at<double>(1), v.at<double>(2)));
    mb = b;
    for (size_t i = 0; i < aux2.size(); i++) {
        if (aux2[i].v.x == 0 && aux2[i].v.y == 0 && aux2[i].v.z == 0) {
            // IntegrateGroMeasurement(aux[i].w, aux[i].t);
            IntegrateGroAccMeasurement(aux2[i].a, aux2[i].w, aux2[i].t);
        }
        else if (aux2[i].w.x == 0 && aux2[i].w.y == 0 && aux2[i].w.z == 0) {
            IntegrateDVLMeasurement2(aux2[i].v_beam, aux2[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }
    // mvMeasurements = aux;
}

void DVLGroPreIntegration::ReintegrateWithVelocity()
{

    //	std::unique_lock<std::mutex> lock(mMutex);
    if (mMutex.try_lock()) {
        const std::vector<integrable> aux = mvMeasurements;
        cv::Mat v = dV.clone();
        Initialize(bu);
        //todo_tightly
        // save mVelocity
        SetVelocity(cv::Point3d(v.at<double>(0), v.at<double>(1), v.at<double>(2)));
        for (size_t i = 0; i < aux.size(); i++)
            //		IntegrateNewMeasurement(aux[i].a, aux[i].w, aux[i].t);
            IntegrateGroMeasurement(aux[i].w, aux[i].t);
        mMutex.unlock();
    }
    else {
        const std::vector<integrable> aux = mvMeasurements;
        cv::Mat v = dV.clone();
        Initialize(bu);
        SetVelocity(cv::Point3d(v.at<double>(0), v.at<double>(1), v.at<double>(2)));
        //		dV = v.clone();
        for (size_t i = 0; i < aux.size(); i++)
            //		IntegrateNewMeasurement(aux[i].a, aux[i].w, aux[i].t);
            IntegrateGroMeasurement(aux[i].w, aux[i].t);
    }

}

void DVLGroPreIntegration::ReintegrateWithBiasAndRotation(const Bias &b, const cv::Mat &R_g_d)
{
    std::lock_guard<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    //	ROS_INFO_STREAM("reintegration measurment: "<<aux.size()<<endl);
    //	cout<<"reintegration measurment: "<<aux.size()<<endl;
    //	ROS_INFO_STREAM("R_g_d reintegration: "<<R_g_d);
    cv::Mat v = dV.clone();
    Initialize(bu);
    //	dV=v.clone();
    //	SetVelocity(cv::Point3d(v.at<double>(0), v.at<double>(1), v.at<double>(2)));
    mb = b;
    mR_g_d = R_g_d.clone();
    mR_g_d.convertTo(mR_g_d, CV_64F);
    for (size_t i = 0; i < aux.size(); i++) {
        if (aux[i].a.x == 0 && aux[i].a.y == 0 && aux[i].a.z == 0) {
            IntegrateGroMeasurement(aux[i].w, aux[i].t);
        }
        else if (aux[i].w.x == 0 && aux[i].w.y == 0 && aux[i].w.z == 0) {
            IntegrateDVLMeasurement(aux[i].a, aux[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }

}

void DVLGroPreIntegration::ReintegrateWithVelocity(const Eigen::Vector3d &velocity)
{

    std::lock_guard<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements2;
    //	ROS_INFO_STREAM("reintegration measurment: "<<aux.size()<<endl);
    //	cout<<"reintegration measurment: "<<aux.size()<<endl;
    //	ROS_INFO_STREAM("velocity reintegration, v: "<<velocity.transpose());
    cv::Mat v;
    cv::eigen2cv(velocity, v);
    v.convertTo(v, CV_64F);
    Initialize(bu);
    //	dV=v.clone();
    //	SetVelocity(cv::Point3d(v.at<double>(0), v.at<double>(1), v.at<double>(2)));

    for (size_t i = 0; i < aux.size(); i++) {
        if (aux[i].a.x == 0 && aux[i].a.y == 0 && aux[i].a.z == 0) {
            IntegrateGroMeasurement(aux[i].w, aux[i].t);
        }
        else if (aux[i].w.x == 0 && aux[i].w.y == 0 && aux[i].w.z == 0) {
            // i == k for the first frame
            IntegrateDVLMeasurement(cv::Point3d(v.at<double>(0), v.at<double>(1), v.at<double>(2)), aux[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }
    // save measurment2
    mvMeasurements2 = aux2;

}

void DVLGroPreIntegration::ReintegrateWithBiasRotationBeamOri(const Bias &b, const cv::Mat &R_g_d,
                                                              const Eigen::Vector4d &alpha,
                                                              const Eigen::Vector4d &beta)
{
    std::lock_guard<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements2;
    cv::Mat v = dV.clone();
    Initialize(bu);
    //	dV=v.clone();
    //	SetVelocity(cv::Point3d(v.at<double>(0), v.at<double>(1), v.at<double>(2)));
    mb = b;
    mR_g_d = R_g_d.clone();
    mR_g_d.convertTo(mR_g_d, CV_64F);
    SetBeamOrientation(alpha, beta);
    for (size_t i = 0; i < aux.size(); i++) {
        if (aux[i].v_beam.x() == 0 && aux[i].v_beam.y() == 0 && aux[i].v_beam.z() == 0 &&
            aux[i].v_beam.w() == 0) {
            IntegrateGroMeasurement(aux[i].w, aux[i].t);
        }
        else if (aux[i].w.x == 0 && aux[i].w.y == 0 && aux[i].w.z == 0) {
            IntegrateDVLMeasurement2(aux[i].v_beam, aux[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }

}

void DVLGroPreIntegration::IntegrateGroMeasurement(const cv::Point3d &angVel, const double &dt)
{
    // if (dV.at<double>(0) == 0 && dV.at<double>(1) == 0 && dV.at<double>(2) == 0) {
    //     ROS_ERROR_STREAM("velocity is not initialized");
    //     exit(-1);
    // }
    mvMeasurements.push_back(integrable(cv::Point3d(0, 0, 0), angVel, dt));
    mvMeasurements2.push_back(
            integrable(cv::Point3d(0, 0, 0), angVel, cv::Point3d(0, 0, 0), Eigen::Vector4d::Zero(), dt));

    cv::Mat accW = (cv::Mat_<double>(3, 1) << angVel.x - mb.bwx, angVel.y - mb.bwy, angVel.z - mb.bwz);
    mAngV = angVel;

    //3*1
    //	avgW = (dT * avgW + accW * dt) / (dT + dt);

    // Total integrated time
    dT += dt;

    // assume V_di constant
    //	dP = dP + dV*dt;

    // assume V_dk constant
    //R_d_g * R_gi_gk * R_g_d * V_dk
    dV = mR_g_d.t() * dR * mR_g_d * mVelocity;
    //	dV =  dR * mVelocity;
    dP_dvl = dP_dvl + dV * dt;
    //	if (dP.at<double>(0,0)!=0)
    //		cout<<"dP: "<<dP<<endl;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    cv::Mat v_k_hat = (cv::Mat_<double>(3, 3) << 0, -dV.at<double>(2), dV.at<double>(1), dV.at<double>(
            2), 0, -dV.at<double>(0), -dV.at<double>(1), dV.at<double>(0), 0);

    //todo_tightly
    //	add velocity jacobians
    //  Update position and velocity jacobians wrt bias correction
    JPg = JPg - dR * dt * v_k_hat * JRg;



    // Update delta rotation
    IntegratedRotation dRi(angVel, mb, dt);
    dR = NormalizeRotation(dR * dRi.deltaR);


    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.t() * JRg - dRi.rightJ * dt;

}

void DVLGroPreIntegration::IntegrateGroAccMeasurement(const cv::Point3d &acc, const cv::Point3d &angVel,
                                                      const double &dt)
{
    mvMeasurements.push_back(integrable(acc, angVel, dt));
    mvMeasurements2.push_back(integrable(acc, angVel, cv::Point3d(0, 0, 0), Eigen::Vector4d::Zero(), dt));

    // acc measurement without bias in body frame
    cv::Mat acc_b = (cv::Mat_<double>(3, 1) << acc.x - mb.bax, acc.y - mb.bay, acc.z - mb.baz);
    mAngV = angVel;

    //3*1
    //	avgW = (dT * avgW + accW * dt) / (dT + dt);

    // Total integrated time
    dT += dt;


    // assume V_di constant
    //	dP = dP + dV*dt;

    // update V_di at time k according to latest gyros and DVL
    //V_di_k = R_d_g * R_gi_gk * R_g_d * V_dk
    dV = mR_g_d.t() * dR * mR_g_d * mVelocity;
    // update P_di_di_dk according to gyros and DVL
    // P_di_di_dk = P_di_di_dk-i + V_di * t
    dP_dvl = dP_dvl + dV * dt;
    // P_bi_bi_bk = P_bi_bi_bk-i + V_di * t
    dP_acc = dP_acc + dDeltaV*dt + 0.5*dR*acc_b*dt*dt;

    //update DeltaV_bi_bj according to acc and gyros
    // DeltaV_bi_bk = DeltaV_bi_bk-1 +
    dDeltaV = dDeltaV + dR * (acc_b * dt);
    //	if (dP.at<double>(0,0)!=0)
    //		cout<<"dP: "<<dP<<endl;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    cv::Mat v_k_hat = (cv::Mat_<double>(3, 3) << 0, -dV.at<double>(2), dV.at<double>(1), dV.at<double>(
            2), 0, -dV.at<double>(0), -dV.at<double>(1), dV.at<double>(0), 0);
    cv::Mat Wacc = (cv::Mat_<double>(3,3) << 0, -acc_b.at<double>(2), acc_b.at<double>(1),
            acc_b.at<double>(2), 0, -acc_b.at<double>(0),
            -acc_b.at<double>(1), acc_b.at<double>(0), 0);

    //todo_tightly
    //	add velocity jacobians
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;
    JPv = JPv + dR*mR_g_d*dt;

    //matrxi related with covariance
    cv::Mat A = cv::Mat::eye(9,9,CV_64F);
    cv::Mat B = cv::Mat::zeros(9,6,CV_64F);

    // computation related with covariance update
    A.rowRange(3,6).colRange(0,3) = -dR*dt*Wacc;
    A.rowRange(6,9).colRange(0,3) = -0.5f*dR*dt*dt*Wacc;
    A.rowRange(6,9).colRange(3,6) = cv::Mat::eye(3,3,CV_64F)*dt;
    B.rowRange(3,6).colRange(3,6) = dR*dt;
    B.rowRange(6,9).colRange(3,6) = 0.5f*dR*dt*dt;



    // Update delta rotation
    IntegratedRotation dRi(angVel, mb, dt);
    dR = NormalizeRotation(dR * dRi.deltaR);

    // Compute rotation parts of matrices A and B
    A.rowRange(0,3).colRange(0,3) = dRi.deltaR.t();
    B.rowRange(0,3).colRange(0,3) = dRi.rightJ*dt;
    // Update covariance
    // cv::Mat ACA = ;
    cv::Mat noiseGA,walk_noiseGA;
    Nga.convertTo(noiseGA, C.type());
    NgaWalk.convertTo(walk_noiseGA,C.type());
    C.rowRange(0,9).colRange(0,9) = A*C.rowRange(0,9).colRange(0,9)*A.t() + B * noiseGA * B.t();
    C.rowRange(9,15).colRange(9,15) = C.rowRange(9,15).colRange(9,15) + walk_noiseGA;


    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.t() * JRg - dRi.rightJ * dt;

}

void DVLGroPreIntegration::IntegrateDVLMeasurement(const cv::Point3d &v_dk, const double &dt)
{
    // V_dk
    Eigen::Vector3d V_dk = Eigen::Vector3d::Zero();
    V_dk.x() = v_dk.x;
    V_dk.y() = v_dk.y;
    V_dk.z() = v_dk.z;

    cv::Mat_<double> v(3, 1);
    v << V_dk.x(), V_dk.y(), V_dk.z();
    //	if (cv::norm(v)>(double)mVelocityThreshold)
    //		return;
    // mVelocity = v;
    bDVL = true;
    //	ReintegrateWithVelocity();
    if (dV.at<double>(0) == 0 && dV.at<double>(1) == 0 && dV.at<double>(2) == 0) {
        //		dV = dR * mR_g_d* v;
        //		dV = dR * v;
        dV = mR_g_d.t() * dR * mR_g_d * v;
        mVelocity = v.clone();
        //		cout<<"dR:"<<dR<<endl;
        //		cout<<"dV:"<<dV<<endl;
        ROS_INFO_STREAM("reintegration with velocity");
        ReintegrateWithVelocity();
    }
    else {
        //		dV = dR * v;
        // mVelocity = (v+mVelocity)/2;
        mVelocity = v.clone();
        dV = mR_g_d.t() * dR * mR_g_d * mVelocity;
    }
    // update pose from Last gyro frame to current dvl frame
    cv::Mat accW = (cv::Mat_<double>(3, 1) << mAngV.x - mb.bwx, mAngV.y - mb.bwy, mAngV.z - mb.bwz);

    cv::Mat acc_b;
    if(mvMeasurements2.size()>0){
        auto  acc = mvMeasurements2.back().a;
        acc_b = (cv::Mat_<double>(3, 1) << acc.x - mb.bax, acc.y - mb.bay, acc.z - mb.baz);
    }
    else
        acc_b = cv::Mat::zeros(3,1,CV_64F);


    // Total integrated time
    dT += dt;

    //	dV =  dR * mVelocity;
    dP_dvl = dP_dvl + dV * dt;
    //	if (dP.at<double>(0,0)!=0)
    //		cout<<"dP: "<<dP<<endl;
    dP_acc = dP_acc + dDeltaV*dt + 0.5*dR*acc_b*dt*dt;
    dDeltaV = dDeltaV + dR * (acc_b * dt);

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    cv::Mat v_k_hat = (cv::Mat_<double>(3, 3) << 0, -dV.at<double>(2), dV.at<double>(1), dV.at<double>(
            2), 0, -dV.at<double>(0), -dV.at<double>(1), dV.at<double>(0), 0);
    cv::Mat Wacc = (cv::Mat_<double>(3,3) << 0, -acc_b.at<double>(2), acc_b.at<double>(1),
            acc_b.at<double>(2), 0, -acc_b.at<double>(0),
            -acc_b.at<double>(1), acc_b.at<double>(0), 0);

    //todo_tightly
    //	add velocity jacobians
    //  Update position and velocity jacobians wrt bias correction
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JPv = JPv + dR*mR_g_d*dt;
    JVg = JVg - dR*dt*Wacc*JRg;


    // Update delta rotation
    IntegratedRotation dRi(mAngV, mb, dt);
    dR = NormalizeRotation(dR * dRi.deltaR);


    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.t() * JRg - dRi.rightJ * dt;
    Eigen::Vector4d velocity_beam = Eigen::Vector4d::Zero();
    mvMeasurements2.push_back(
            integrable(cv::Point3d(0, 0, 0), cv::Point3d(0, 0, 0), cv::Point3d(V_dk.x(), V_dk.y(), V_dk.z()),
                       velocity_beam, dt));
    mvMeasurements.push_back(integrable(cv::Point3d(V_dk.x(), V_dk.y(), V_dk.z()), cv::Point3d(0, 0, 0), dt));
    mBeams.push_back(velocity_beam(0));
    mBeams.push_back(velocity_beam(1));
    mBeams.push_back(velocity_beam(2));
    mBeams.push_back(velocity_beam(3));
}

void DVLGroPreIntegration::IntegrateDVLMeasurement2(const Eigen::Vector4d &velocity_beam, const double &dt)
{

    // V_dk
    Eigen::Vector3d V_dk = mETEInv * mE.transpose() * velocity_beam;
    // V_dk = v_di_dvl;

    cv::Mat_<double> v(3, 1);
    v << V_dk.x(), V_dk.y(), V_dk.z();
    //	if (cv::norm(v)>(double)mVelocityThreshold)
    //		return;
    // mVelocity = v;
    bDVL = true;
    //	ReintegrateWithVelocity();
    if (dV.at<double>(0) == 0 && dV.at<double>(1) == 0 && dV.at<double>(2) == 0) {
        //		dV = dR * mR_g_d* v;
        //		dV = dR * v;
        dV = mR_g_d.t() * dR * mR_g_d * v;
        mVelocity = v.clone();
        //		cout<<"dR:"<<dR<<endl;
        //		cout<<"dV:"<<dV<<endl;
        ROS_INFO_STREAM("reintegration with velocity");
        ReintegrateWithVelocity();
    }
    else {
        //		dV = dR * v;
        // mVelocity = (v+mVelocity)/2;
        mVelocity = v.clone();
        dV = mR_g_d.t() * dR * mR_g_d * mVelocity;
    }

    //matrxi related with covariance
    cv::Mat A = cv::Mat::eye(9,9,CV_64F);
    cv::Mat B = cv::Mat::zeros(9,6,CV_64F);

    // update pose from Last gyro frame to current dvl frame
    cv::Mat accW = (cv::Mat_<double>(3, 1) << mAngV.x - mb.bwx, mAngV.y - mb.bwy, mAngV.z - mb.bwz);

    cv::Mat acc_b;
    if(mvMeasurements2.size()>0){
        auto  acc = mvMeasurements2.back().a;
        acc_b = (cv::Mat_<double>(3, 1) << acc.x - mb.bax, acc.y - mb.bay, acc.z - mb.baz);
    }
    else
        acc_b = cv::Mat::zeros(3,1,CV_64F);


    // Total integrated time
    dT += dt;

    //	dV =  dR * mVelocity;
    dP_dvl = dP_dvl + dV * dt;
    //	if (dP.at<double>(0,0)!=0)
    //		cout<<"dP: "<<dP<<endl;
    dP_acc = dP_acc + dDeltaV*dt + 0.5*dR*acc_b*dt*dt;
    dDeltaV = dDeltaV + dR * (acc_b * dt);

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    cv::Mat v_k_hat = (cv::Mat_<double>(3, 3) << 0, -dV.at<double>(2), dV.at<double>(1), dV.at<double>(
            2), 0, -dV.at<double>(0), -dV.at<double>(1), dV.at<double>(0), 0);
    cv::Mat Wacc = (cv::Mat_<double>(3,3) << 0, -acc_b.at<double>(2), acc_b.at<double>(1),
            acc_b.at<double>(2), 0, -acc_b.at<double>(0),
            -acc_b.at<double>(1), acc_b.at<double>(0), 0);

    // computation related with covariance update
    A.rowRange(3,6).colRange(0,3) = -dR*dt*Wacc;
    A.rowRange(6,9).colRange(0,3) = -0.5f*dR*dt*dt*Wacc;
    A.rowRange(6,9).colRange(3,6) = cv::Mat::eye(3,3,CV_64F)*dt;
    B.rowRange(3,6).colRange(3,6) = dR*dt;
    B.rowRange(6,9).colRange(3,6) = 0.5f*dR*dt*dt;

    //todo_tightly
    //	add velocity jacobians
    //  Update position and velocity jacobians wrt bias correction
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JPv = JPv + dR*mR_g_d*dt;
    JVg = JVg - dR*dt*Wacc*JRg;


    // Update delta rotation
    IntegratedRotation dRi(mAngV, mb, dt);
    dR = NormalizeRotation(dR * dRi.deltaR);

    // Compute rotation parts of matrices A and B
    A.rowRange(0,3).colRange(0,3) = dRi.deltaR.t();
    B.rowRange(0,3).colRange(0,3) = dRi.rightJ*dt;
    // Update covariance
    // cv::Mat ACA = ;
    cv::Mat noiseGA,walk_noiseGA;
    Nga.convertTo(noiseGA, C.type());
    NgaWalk.convertTo(walk_noiseGA,C.type());
    C.rowRange(0,9).colRange(0,9) = A*C.rowRange(0,9).colRange(0,9)*A.t() + B * noiseGA * B.t();
    C.rowRange(9,15).colRange(9,15) = C.rowRange(9,15).colRange(9,15) + walk_noiseGA;


    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.t() * JRg - dRi.rightJ * dt;
    mvMeasurements2.push_back(
            integrable(cv::Point3d(0, 0, 0), cv::Point3d(0, 0, 0), cv::Point3d(V_dk.x(), V_dk.y(), V_dk.z()),
                       velocity_beam, dt));
    mvMeasurements.push_back(integrable(cv::Point3d(V_dk.x(), V_dk.y(), V_dk.z()), cv::Point3d(0, 0, 0), dt));
    mBeams.push_back(velocity_beam(0));
    mBeams.push_back(velocity_beam(1));
    mBeams.push_back(velocity_beam(2));
    mBeams.push_back(velocity_beam(3));
}

void DVLGroPreIntegration::SetVelocity(const cv::Point3d &v_di)
{
    // V_di
    cv::Mat_<double> v(3, 1);
    v << v_di.x, v_di.y, v_di.z;



    //		dV = dR * v;
    dV = v;
    // R_d_g * R_gj_gi * R_g_d = R_dj_di
    mVelocity =  v.clone();
    // cv::Point3d v_dj(mVelocity.at<double>(0), mVelocity.at<double>(1), mVelocity.at<double>(2));

    //	mvMeasurements.push_back(integrable(v_dj, cv::Point3d(0, 0, 0), 0));
}

void DVLGroPreIntegration::MergePrevious(DVLGroPreIntegration* pPrev)
{
    if (pPrev == this) {
        return;
    }

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable> aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    Initialize(bav);
    for (size_t i = 0; i < aux1.size(); i++) {
        if (aux1[i].a.x == 0 && aux1[i].a.y == 0 && aux1[i].a.z == 0) {
            IntegrateGroMeasurement(aux1[i].w, aux1[i].t);
        }
        else if (aux1[i].w.x == 0 && aux1[i].w.y == 0 && aux1[i].w.z == 0) {
            IntegrateDVLMeasurement(aux1[i].a, aux1[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }
    for (size_t i = 0; i < aux2.size(); i++) {
        if (aux2[i].a.x == 0 && aux2[i].a.y == 0 && aux2[i].a.z == 0) {
            IntegrateGroMeasurement(aux2[i].w, aux2[i].t);
        }
        else if (aux2[i].w.x == 0 && aux2[i].w.y == 0 && aux2[i].w.z == 0) {
            IntegrateDVLMeasurement(aux2[i].a, aux2[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }

}

void DVLGroPreIntegration::MergePrevious2(DVLGroPreIntegration* pPrev)
{
    if (pPrev == this) {
        return;
    }

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable> aux1 = pPrev->mvMeasurements2;
    const std::vector<integrable> aux2 = mvMeasurements2;

    Initialize(bav);
    for (size_t i = 0; i < aux1.size(); i++) {
        // velocity = 0
        if (aux1[i].v.x == 0 && aux1[i].v.y == 0 && aux1[i].v.z == 0) {
            IntegrateGroMeasurement(aux1[i].w, aux1[i].t);
        }
        else if (aux1[i].w.x == 0 && aux1[i].w.y == 0 && aux1[i].w.z == 0) {
            IntegrateDVLMeasurement2(aux1[i].v_beam, aux1[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }
    for (size_t i = 0; i < aux2.size(); i++) {
        if (aux2[i].v.x == 0 && aux2[i].v.y == 0 && aux2[i].v.z == 0) {
            IntegrateGroMeasurement(aux2[i].w, aux2[i].t);
        }
        else if (aux2[i].w.x == 0 && aux2[i].w.y == 0 && aux2[i].w.z == 0) {
            IntegrateDVLMeasurement2(aux2[i].v_beam, aux2[i].t);
        }
        else {
            cout << "found error in measurement" << endl;
        }
    }
}

void DVLGroPreIntegration::SetNewBias(const Bias &bu_)
{
    ReintegrateWithBias(bu_);
}

IMU::Bias DVLGroPreIntegration::GetDeltaBias(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    return IMU::Bias(b_.bax - mb.bax, b_.bay - mb.bay, b_.baz - mb.baz, b_.bwx - mb.bwx, b_.bwy - mb.bwy,
                     b_.bwz - mb.bwz);
}

cv::Mat DVLGroPreIntegration::GetDeltaRotation(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<double>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
    cv::Mat R = NormalizeRotation(dR*ExpSO3(JRg*dbg));
    // R.convertTo(R, CV_32F);
    return R;
}

cv::Mat DVLGroPreIntegration::GetDeltaRotation(const Bias &b, const cv::Mat &R_g_d)
{
    //	std::unique_lock<std::mutex> lock(mMutex);
    ReintegrateWithBiasAndRotation(b, R_g_d);
    return NormalizeRotation(dR);
}

cv::Mat DVLGroPreIntegration::GetDeltaVelocity(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<double>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
    cv::Mat dba = (cv::Mat_<double>(3, 1) << b_.bax - mb.bax, b_.bay - mb.bay, b_.baz - mb.baz);
    // cv::Mat V = dDeltaV  + JVa * dba;
    cv::Mat V = dDeltaV  + JVa * dba + JVg * dbg;
    // V.convertTo(V, CV_32F);
    return V;
}

cv::Mat DVLGroPreIntegration::GetDVLPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<double>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
    cv::Mat dba = (cv::Mat_<double>(3, 1) << b_.bax - mb.bax, b_.bay - mb.bay, b_.baz - mb.baz);
    cv::Mat P = dP_dvl.clone();
    P.convertTo(P, CV_32F);
    return P;
}

cv::Mat DVLGroPreIntegration::GetDVLPosition(const Bias &b_, const Eigen::Vector3d &velocity)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat v_new ;
    cv::eigen2cv(velocity,v_new);
    cv::Mat v_delta = v_new - dV;
    cv::Mat dbg = (cv::Mat_<double>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
    cv::Mat dba = (cv::Mat_<double>(3, 1) << b_.bax - mb.bax, b_.bay - mb.bay, b_.baz - mb.baz);
    cv::Mat P = dP_dvl.clone()  + JPv*v_delta;
    // ROS_INFO_STREAM("current v: "<<dV<<" new v: "<<v_new<<"delta p"<<JPv*v_delta);
    P.convertTo(P, CV_32F);
    return P;
}

cv::Mat DVLGroPreIntegration::GetDeltaPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat dbg = (cv::Mat_<double>(3, 1) << b_.bwx - mb.bwx, b_.bwy - mb.bwy, b_.bwz - mb.bwz);
    cv::Mat dba = (cv::Mat_<double>(3, 1) << b_.bax - mb.bax, b_.bay - mb.bay, b_.baz - mb.baz);
    cv::Mat P = dP_acc + JPa * dba + JPg * dbg;
    // cv::Mat P = dP_acc + JPa * dba;
    // P.convertTo(P, CV_32F);
    return P;
}

cv::Mat DVLGroPreIntegration::GetDeltaPosition(const Bias &b, const cv::Mat &R_g_d)
{
    //	std::unique_lock<std::mutex> lock(mMutex);
    ReintegrateWithBiasAndRotation(b, R_g_d);
    return dP_dvl;
}

cv::Mat DVLGroPreIntegration::GetUpdatedDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat R = NormalizeRotation(dR * ExpSO3(JRg * db.rowRange(0, 3)));
    R.convertTo(R, CV_32F);
    return R;
}

cv::Mat DVLGroPreIntegration::GetUpdatedDeltaVelocity()
{
    //todo_tightly
    //	complete update of JVg
    //  not used currently, maybe for further use
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat V = dV + JVg * db.rowRange(0, 3) + JVa * db.rowRange(3, 6);
    V.convertTo(V, CV_32F);
    return V;
}

cv::Mat DVLGroPreIntegration::GetUpdatedDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    cv::Mat P = dP_dvl + JPg * db.rowRange(0, 3) + JPa * db.rowRange(3, 6);
    P.convertTo(P, CV_32F);
    return P;
}

cv::Mat DVLGroPreIntegration::GetOriginalDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dR.clone();
}

cv::Mat DVLGroPreIntegration::GetOriginalDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV.clone();
}

cv::Mat DVLGroPreIntegration::GetOriginalDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP_dvl.clone();
}

Bias DVLGroPreIntegration::GetOriginalBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return mb;
}

Bias DVLGroPreIntegration::GetUpdatedBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return bu;
}

cv::Mat DVLGroPreIntegration::GetDeltaBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return db.clone();
}

Eigen::Matrix<double, 15, 15> DVLGroPreIntegration::GetInformationMatrix()
{
    std::unique_lock<std::mutex> lock(mMutex);
    if (Info.empty()) {
        Info = cv::Mat::zeros(15, 15, CV_64F);
        Info.rowRange(0, 9).colRange(0, 9) = C.rowRange(0, 9).colRange(0, 9).inv(cv::DECOMP_SVD);
        for (int i = 9; i < 15; i++)
            Info.at<double>(i, i) = 1.0f / C.at<double>(i, i);
    }

    Eigen::Matrix<double, 15, 15> EI;
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            EI(i, j) = Info.at<double>(i, j);
    return EI;
}

void DVLGroPreIntegration::output()
{
    Eigen::Matrix3d R_gi_gj;
    cv::cv2eigen(dR, R_gi_gj);
    Eigen::Quaterniond q(R_gi_gj);
    //	ROS_INFO_STREAM("integrated oritentation: x:"<<q.x()<<" y:"<<q.y()<<" z:"<<q.z()<<" w:"<<q.w());
}

Eigen::Isometry3d DVLGroPreIntegration::getDVLPose()
{
    cv::Mat R_gi_gj, R_di_dj, t_di_didj;
    dR.copyTo(R_gi_gj);
    dP_dvl.copyTo(t_di_didj);
    R_di_dj = mR_g_d.t() * R_gi_gj * mR_g_d;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    cv::cv2eigen(R_di_dj, R);
    cv::cv2eigen(t_di_didj, t);

    Eigen::Isometry3d T_di_dj = Eigen::Isometry3d::Identity();
    T_di_dj.pretranslate(t);
    T_di_dj.rotate(R);
    return T_di_dj;
}

void DVLGroPreIntegration::SetBeamOrientation(const Eigen::Vector4d &alpha, const Eigen::Vector4d &beta)
{
    //	std::lock_guard lock(mMutex);
    mAlpha = alpha;
    mBeta = beta;
    mE << -cos(beta(0)) * cos(alpha(0)), sin(beta(0)) * cos(alpha(0)), sin(alpha(0)), -cos(beta(1)) *
                                                                                      cos(alpha(1)),
            -sin(beta(1)) * cos(alpha(1)), sin(alpha(1)), cos(beta(2)) * cos(alpha(2)), -sin(beta(2)) *
                                                                                        cos(alpha(2)), sin(
            alpha(2)), cos(beta(3)) * cos(alpha(3)), sin(beta(3)) * cos(alpha(3)), sin(alpha(3));
    mETEInv = (mE.transpose() * mE).inverse();
}

void DVLGroPreIntegration::SetDVLDebugVelocity(const cv::Point3d &v_dk)
{
    Eigen::Matrix3d R_gi_gk, R_g_d;
    cv::cv2eigen(dR, R_gi_gk);
    cv::cv2eigen(mR_g_d, R_g_d);
    Eigen::Vector3d V_dk(v_dk.x, v_dk.y, v_dk.z);
    Eigen::Vector3d V_di = R_g_d.inverse() * R_gi_gk * R_g_d * V_dk;
    v_di_dvl = V_di;
}

template<class Archive>
void DVLGroPreIntegration::serialize(Archive &ar, const unsigned int version)
{
    ar & dT;
    // serializeMatrix(ar, C, version);
    // serializeMatrix(ar, Info, version);
    // serializeMatrix(ar, Nga, version);
    // serializeMatrix(ar, NgaWalk, version);
    ar & mb;
    ar & mCalib;
    serializeMatrix(ar, dR, version);
    serializeMatrix(ar, dV, version);
    serializeMatrix(ar, dP_dvl, version);
    serializeMatrix(ar, mVelocity, version);
    serializeMatrix(ar, dP_acc, version);
    serializeMatrix(ar, dDeltaV, version);
    serializeMatrix(ar, mR_g_d, version);
    serializePoint3d(ar, mAngV, version);
    serializeEigenMatrix(ar, mAlpha, version);
    serializeEigenMatrix(ar, mBeta, version);
    serializeEigenMatrix(ar, mE, version);
    serializeEigenMatrix(ar, mETEInv, version);
    serializeMatrix(ar, JRg, version);
    serializeMatrix(ar, JVg, version);
    serializeMatrix(ar, JVa, version);
    serializeMatrix(ar, JPg, version);
    serializeMatrix(ar, JPa, version);
    serializeMatrix(ar, JPv, version);
    // serializeMatrix(ar, avgA, version);
    // serializeMatrix(ar, avgW, version);

    ar & bDVL;
    ar & mVelocityThreshold;
    serializePoint3d(ar, v_dk_dvl, version);
    // serializeEigenMatrix(ar, v_di_dvl, version);
    // serializeEigenMatrix(ar, v_dk_visual, version);
    //		serializeEigenV3d(ar, v_dk_visual, version);
    ar & mBeams;


    ar & bu;
    serializeMatrix(ar, db, version);
    ar & mvMeasurements;
    ar & mvMeasurements2;
}
BOOST_CLASS_EXPORT_IMPLEMENT(DVLGroPreIntegration)
template void DVLGroPreIntegration::serialize(boost::archive::text_oarchive & ar, const unsigned int version);
template void DVLGroPreIntegration::serialize(boost::archive::text_iarchive & ar, const unsigned int version);
template void DVLGroPreIntegration::serialize(boost::archive::binary_oarchive & ar, const unsigned int version);
template void DVLGroPreIntegration::serialize(boost::archive::binary_iarchive & ar, const unsigned int version);