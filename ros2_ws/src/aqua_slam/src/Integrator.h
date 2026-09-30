//
// Created by da on 24/01/23.
//

#ifndef SRC_INTEGRATOR_H
#define SRC_INTEGRATOR_H

#include "DVLGroPreIntegration.h"
#include <list>
//class DVLGroPreIntegration;

namespace ORB_SLAM3
{
    class Frame;

    class KeyFrame;

    class Integrator
    {
    public:
        Integrator() : mpIntFromKF_D2D(nullptr), mpIntFromKF_C2C(nullptr), mpIntFromKFBeforeLost_C2C(nullptr),
                       mbDoLossIntegration(false), mpIntFromF_C2C(nullptr), mpLossRefKF(nullptr),
                       mLastVelocity(cv::Point3d(0, 0, 0)) {}

        virtual ~Integrator()
        {
            if (mpIntFromKF_D2D) {
                delete mpIntFromKF_D2D;
                mpIntFromKF_D2D = nullptr;
            }
            if (mpIntFromKF_C2C) {
                delete mpIntFromKF_C2C;
                mpIntFromKF_C2C = nullptr;
            }
            if (mpIntFromKFBeforeLost_C2C) {
                delete mpIntFromKFBeforeLost_C2C;
                mpIntFromKFBeforeLost_C2C = nullptr;
            }
            if (mpIntFromF_C2C) {
                delete mpIntFromF_C2C;
                mpIntFromF_C2C = nullptr;
            }
        }

//        void CreateNewIntFromKF_C2C(const IMU::Bias &b, const IMU::Calib &c);

        void CreateNewIntFromKF_C2C(const IMU::Bias &b, const IMU::Calib &c, const Eigen::Vector4d &alpha,
                                    const Eigen::Vector4d &beta);

        /***
        * create new Frame intrahtion from a KeyFrame integration,
        * only get the velocity
        * @param IntFromKF KeyFrame integration
        */
        void CreateNewIntFromF_C2C(const DVLGroPreIntegration &IntFromKF);

        void CreateNewIntFromF_C2C(const IMU::Bias &b, const IMU::Calib &c, const Eigen::Vector4d &alpha,
                                   const Eigen::Vector4d &beta, const cv::Point3d &v = cv::Point3d(0, 0, 0));

        /***
         *
         * @param b bias
         * @param c extrinsic parameters
         * @param v velocity in DVL frame
         */
        void CreateNewIntFromKF_D2D(const IMU::Bias &b, const IMU::Calib &c, const cv::Point3d &v);

        /***
         *
         * @param b bias
         * @param c extrinsic parameters
         * @param v velocity in DVL frame
         * @param alpha orientation of DVL transducer
         * @param beta orientation of DVL transducer
         */
        void CreateNewIntFromKF_D2D(const IMU::Bias &b, const IMU::Calib &c, const Eigen::Vector4d &alpha,
                                    const Eigen::Vector4d &beta, const cv::Point3d &v = cv::Point3d(0, 0, 0));

        /***
        * create a Loss integration from a KeyFrame Integration
        * use copy constructor to copy all data from the KeyFrame Integration
        * @param IntFromKF KeyFrame integration
        */
        void CreateNewIntFromKFBeforeLoss_D2D(const DVLGroPreIntegration &integration);


        void SetDoLossIntegration(bool doLossInt);

        bool GetDoLossIntegration();

        void SetLossRefKF(KeyFrame* pKF);

        KeyFrame* GetLossRefKF();

        void IntegrateMeasurements(Frame &cur_F, std::list<IMU::GyroDvlPoint> &DvlGyroDataQueue,
                                   std::mutex &queue_mutex, std::vector<IMU::GyroDvlPoint> &measurements);

    protected:
        inline void DeleteInt(DVLGroPreIntegration* pInt)
        {
            if (pInt) {
                delete pInt;
                pInt = nullptr;
            }
        }

    public:
        DVLGroPreIntegration* mpIntFromKF_D2D;
        DVLGroPreIntegration* mpIntFromKF_C2C;

        DVLGroPreIntegration* mpIntFromKFBeforeLost_C2C;
        bool mbDoLossIntegration;
        DVLGroPreIntegration* mpIntFromF_C2C;

        cv::Point3d mLastVelocity;

        KeyFrame* mpLossRefKF;


    };
}


#endif //SRC_INTEGRATOR_H
