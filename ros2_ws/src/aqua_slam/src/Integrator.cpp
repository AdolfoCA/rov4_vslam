//
// Created by da on 24/01/23.
//

#include "Integrator.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "ros1_compat.h"
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>


//void Integrator::CreateNewIntFromKF_C2C(const Bias &b, const Calib &c)
//{
//     DeleteInt(mpIntFromKF_C2C);
//    mpIntFromKF_C2C = new DVLGroPreIntegration(b, c);
//}

void Integrator::CreateNewIntFromKF_C2C(const Bias &b, const Calib &c, const Eigen::Vector4d &alpha,
                                        const Eigen::Vector4d &beta)
{
    // DeleteInt(mpIntFromKF_C2C);
    mpIntFromKF_C2C = new DVLGroPreIntegration(b, c, cv::Point3d(0, 0, 0), alpha, beta);
}

void Integrator::CreateNewIntFromF_C2C(const DVLGroPreIntegration &IntFromKF)
{
    DeleteInt(mpIntFromF_C2C);
    mpIntFromF_C2C = new DVLGroPreIntegration(IntFromKF.mb, IntFromKF.mCalib,
                                              cv::Point3d(IntFromKF.dV.at<double>(0),
                                                          IntFromKF.dV.at<double>(1),
                                                          IntFromKF.dV.at<double>(2)), IntFromKF.mAlpha,
                                              IntFromKF.mBeta);
}

void Integrator::CreateNewIntFromF_C2C(const Bias &b, const Calib &c, const Eigen::Vector4d &alpha,
                                       const Eigen::Vector4d &beta, const cv::Point3d &v)
{
    DeleteInt(mpIntFromF_C2C);
    mpIntFromF_C2C = new DVLGroPreIntegration(b, c, v, alpha, beta);
}

void Integrator::CreateNewIntFromKF_D2D(const Bias &b, const Calib &c, const cv::Point3d &v)
{
    // DeleteInt(mpIntFromKF_D2D);
    mpIntFromKF_D2D = new DVLGroPreIntegration(b, c, v);
}

void Integrator::CreateNewIntFromKF_D2D(const Bias &b, const Calib &c, const Eigen::Vector4d &alpha,
                                        const Eigen::Vector4d &beta, const cv::Point3d &v)
{
    // DeleteInt(mpIntFromKF_D2D);
    mpIntFromKF_D2D = new DVLGroPreIntegration(b, c, v, alpha, beta);
}

void Integrator::CreateNewIntFromKFBeforeLoss_D2D(const DVLGroPreIntegration &integration)
{
    // DeleteInt(mpIntFromKFBeforeLost_C2C);
    mpIntFromKFBeforeLost_C2C = new DVLGroPreIntegration(integration);
}


void Integrator::IntegrateMeasurements(Frame &cur_F, std::list<IMU::GyroDvlPoint> &DvlGyroDataQueue,
                                       std::mutex &queue_mutex, std::vector<IMU::GyroDvlPoint> &measurements)
{
    auto cur_time = cur_F.mTimeStamp;
    auto pre_time = cur_F.mpPrevFrame->mTimeStamp;
    if (pre_time == 0) {
        ROS_WARN_STREAM(fixed<<setprecision(6)<<"non prev frame, frame timestamp: "<<cur_time);
        //		Verbose::PrintMess(, Verbose::VERBOSITY_NORMAL);
        cur_F.setIntegrated();
        return;
    }
    else if (DvlGyroDataQueue.size() == 0) {
        ROS_WARN_STREAM("Not IMU data in mlQueueDVLGyroData!!");
        cur_F.setIntegrated();
        return;
    }

    // cout << "start loop. Total meas:" << mlQueueImuData.size() << endl;

    measurements.clear();
    measurements.reserve(DvlGyroDataQueue.size());

    // put Gyros DVL Meas from mlQueueDVLGyroData to mvGyroDVLFromLastFrame
    while (true) {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(queue_mutex);
            if (!DvlGyroDataQueue.empty()) {
                IMU::GyroDvlPoint* m = &DvlGyroDataQueue.front();
                //				cout<<"IMU Meas acc: "<<m->a<<" timestamp: "<< m->t <<endl;
                cout.precision(17);
                // IMU measurement is before Frame_i
                if (m->t < cur_F.mpPrevFrame->mTimeStamp - 0.00001l) {
                    DvlGyroDataQueue.pop_front();
                }
                    // IMU measurement is between Frame_i and Frame_j
                else if (m->t <= cur_F.mTimeStamp + 0.00001l) {
                    //					cout<<"push_back IMU Meas acc: "<<m->a<<" timestamp: "<< m->t <<endl;
                    measurements.push_back(*m);
                    DvlGyroDataQueue.pop_front();
                }
                    // IMU measurement is after Frame_j
                else {
                    //					cout << "Error! IMU measurement is after Frame_j!!!" << endl;
                    measurements.push_back(*m);
                    break;
                }
            }
            else {
                break;
                bSleep = true;
            }
        }
        if (bSleep) {
            usleep(500);
        }
    }

    int n = measurements.size();
    if (n == 0) {
        ROS_WARN_STREAM("no measurement to integrate!");
        return;
    }


    //check pointer
    // KF
    if (!mpIntFromKF_C2C) {
        ROS_ERROR_STREAM("null poniter mpIntFromKF_C2C");
        assert(0);
    }

    // get velocity from DVL
    // cv::Point3d v(0, 0, 0);
    // for (auto m: measurements) {
    //     if ((m.v.x != 0) && (m.v.y != 0) && (m.v.z != 0)) {
    //         v = m.v;
    //         break;
    //     }
    // }
    // set initial velocity of Frame integration
    if (mpIntFromKF_C2C->bDVL) {
        CreateNewIntFromF_C2C(mpIntFromKF_C2C);
    }
    else if ((mLastVelocity.x == 0) && (mLastVelocity.y == 0) && (mLastVelocity.z == 0)) {
        // initialization velocity
        bool isDVL_there = false;
        for (auto m: measurements) {
            if ((m.v.x != 0) && (m.v.y != 0) && (m.v.z != 0)) {
                mpIntFromKF_C2C->SetVelocity(m.v);
                CreateNewIntFromF_C2C(mpIntFromKF_C2C);
                isDVL_there = true;
            }
        }
        if(!isDVL_there){
            ROS_WARN_STREAM("wait init, skip integration");
            return;
        }
    }
    else {
        mpIntFromKF_C2C->SetVelocity(mLastVelocity);
        CreateNewIntFromF_C2C(mpIntFromKF_C2C);
    }


    if (mpIntFromF_C2C->dV.at<double>(0) == 0) {
        ROS_ERROR_STREAM("Velocity is not initiialzed!");
        assert(mpIntFromF_C2C->dV.at<double>(0) != 0);
    }

    // whether dvl measurement has been integrated in the current time interval
    bool bDVL_integrated = false;
    for (int i = 0; i < n; i++) {
        float tstep;
        cv::Point3d v_d, angVel, acc;
        Eigen::Vector4d v_beam;
        // first IMU meas after Frame_i
        // and this IMU meas is not the last two
        if ((i == 0) && (i < (n - 1))) {
            // delta_t between IMU meas
            float tab = measurements[i + 1].t - measurements[i].t;
            if (tab == 0) {
                tab = 0.001;
            }
            v_d = measurements[i].v;
            // estimate the angVel from Frame_i to second IMU meas
            angVel = measurements[i].angular_v;
            acc = measurements[i].acc;
            v_beam = measurements[i].vb;
            tstep = measurements[i + 1].t - pre_time;
        }
            // not first and not last, the middle IMU meas
        else if (i < (n - 1)) {
            v_d = measurements[i].v;
            // estimate the angVel from Frame_i to second IMU meas
            acc = measurements[i].acc;
            angVel = measurements[i].angular_v;
            v_beam = measurements[i].vb;
            tstep = measurements[i + 1].t - measurements[i].t;
        }
            // last two IMU meas before Frame_j
            // and this IMU meas is not the first IMU meas after Frame_i
        else if ((i > 0) && (i == (n - 1))) {
            float tab = measurements[i + 1].t - measurements[i].t;
            if (tab == 0) {
                tab = 0.001;
            }
            float tend = measurements[i + 1].t - cur_time;
            // estimate the acc from the last second IMU meas to Frame_j
            v_d = measurements[i].v;
            // estimate the angVel from Frame_i to second IMU meas
            acc = measurements[i].acc;
            angVel = measurements[i].angular_v;
            v_beam = measurements[i].vb;
            tstep = cur_time - measurements[i].t;
        }
            // last two IMU meas before Frame_j
            // and this IMU meas is also the first IMU meas after Frame_i
        else if ((i == 0) && (i == (n - 1))) {
            v_d = measurements[i].v;
            // estimate the angVel from Frame_i to second IMU meas
            angVel = measurements[i].angular_v;
            acc = measurements[i].acc;
            v_beam = measurements[i].vb;
            tstep = cur_time - pre_time;
        }

        // imu measurement
        if ((angVel.x != 0) && (angVel.y != 0) && (angVel.z != 0)) {
            /*// KF
            mpIntFromKF_C2C->IntegrateGroMeasurement(angVel, tstep);
            if(mpIntFromKF_D2D->bDVL){
                mpIntFromKF_D2D->IntegrateGroMeasurement(angVel, tstep);
            }
            // Loss
            if (mbDoLossIntegration) { 
                mpIntFromKFBeforeLost_D2D->IntegrateGroMeasurement(angVel, tstep);
            }
            // F
            mpIntFromF_C2C->IntegrateGroMeasurement(angVel, tstep);*/


            // KF
            mpIntFromKF_C2C->IntegrateGroAccMeasurement(acc, angVel, tstep);
            /*// KF DVL to DVL
            if (mpIntFromKF_D2D->bDVL) {
                // start DVL to DVL integration
                if (!bDVL_integrated) {
                    // haven't reach DVL measurement
                    mpIntFromKF_D2D->IntegrateGroAccMeasurement(acc, angVel, tstep);
                }
                else {
                    // handle others after DVL measurement
                    mpIntFromKF_D2D->IntegrateGroAccMeasurement(acc, angVel, tstep);
                }
            }
            else{
                // handle measurement before DVL measurement
                mpIntFromKF_D2D->IntegrateGroAccMeasurement(acc, angVel, tstep);
            }*/
            // Loss
            if (mbDoLossIntegration) {
                mpIntFromKFBeforeLost_C2C->IntegrateGroAccMeasurement(acc, angVel, tstep);
            }
            // F
            mpIntFromF_C2C->IntegrateGroAccMeasurement(acc, angVel, tstep);
        }
        // dvl measurement
        if (v_d.x != 0 && v_d.y != 0 && v_d.z != 0) {
            // if (mpIntFromKF_D2D->bDVL) {
            //     bDVL_integrated = true;
            // }

            //KF
            mpIntFromKF_C2C->v_dk_dvl = v_d;
            mpIntFromKF_C2C->SetDVLDebugVelocity(v_d);
            if(v_beam.x()!=0&&v_beam.y()!=0&&v_beam.z()!=0&&v_beam.w()!=0){
                mpIntFromKF_C2C->IntegrateDVLMeasurement2(v_beam, tstep);
            }
            else{
                mpIntFromKF_C2C->IntegrateDVLMeasurement(v_d, tstep);
            }


            mLastVelocity = cv::Point3d(mpIntFromKF_C2C->mVelocity.at<double>(0),
                                        mpIntFromKF_C2C->mVelocity.at<double>(1),
                                        mpIntFromKF_C2C->mVelocity.at<double>(2));
            BOOST_LOG_TRIVIAL(info) << "integrate DVL beam velocity" << mLastVelocity << "\n";
            BOOST_LOG_TRIVIAL(info) << "DVL output velocity" << v_d << "\n";
            // KF DVL to DVL
            // mpIntFromKF_D2D->IntegrateDVLMeasurement2(v_beam, tstep);
            // mpIntFromKF_D2D->v_dk_dvl = v_d;
            // mpIntFromKF_D2D->SetDVLDebugVelocity(v_d);
            // F
            mpIntFromF_C2C->v_dk_dvl = v_d;
            mpIntFromF_C2C->SetDVLDebugVelocity(v_d);
            if(v_beam.x()!=0&&v_beam.y()!=0&&v_beam.z()!=0&&v_beam.w()!=0){
                mpIntFromKF_C2C->IntegrateDVLMeasurement2(v_beam, tstep);
            }
            else{
                mpIntFromKF_C2C->IntegrateDVLMeasurement(v_d, tstep);
            }

            // Loss
            if (mbDoLossIntegration) {
                mpIntFromKFBeforeLost_C2C->v_dk_dvl = v_d;
                mpIntFromKFBeforeLost_C2C->SetDVLDebugVelocity(v_d);
                if(v_beam.x()!=0&&v_beam.y()!=0&&v_beam.z()!=0&&v_beam.w()!=0){
                    mpIntFromKFBeforeLost_C2C->IntegrateDVLMeasurement2(v_beam, tstep);
                }
                else{
                    mpIntFromKFBeforeLost_C2C->IntegrateDVLMeasurement(v_d, tstep);
                }

            }
        }
    }
}

void Integrator::SetDoLossIntegration(bool doLossInt)
{
    mbDoLossIntegration = doLossInt;

}

bool Integrator::GetDoLossIntegration()
{
    return mbDoLossIntegration;
}

void Integrator::SetLossRefKF(KeyFrame* pKF)
{
    ROS_INFO_STREAM(fixed<<setprecision(6)<<"set Loss Ref KF["<<pKF->mnId<<"] "<<pKF->mTimeStamp);
    mpLossRefKF = pKF;
}

KeyFrame* Integrator::GetLossRefKF()
{
    return mpLossRefKF;
}
