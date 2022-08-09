/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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


#ifndef MOVS_IMU_FRAME_H
#define MOVS_IMU_FRAME_H

#include<vector>

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"

#include "Thirdparty/Sophus/sophus/geometry.hpp"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include "Converter.h"
#include "Settings.h"

#include <mutex>
#include <opencv2/opencv.hpp>

#include "Eigen/Core"
#include "sophus/se3.hpp"

namespace ORB_SLAM3
{

    class IMUFrame
    {
    public:
        IMUFrame();

        // Copy constructor.
        IMUFrame(const IMUFrame& frame);

        // Constructor for Monocular cameras.
        IMUFrame(const float& bf, const float& thDepth, const IMU::Calib& ImuCalib = IMU::Calib());

        // Destructor
        // ~Frame();

        // Set IMU velocity
        void SetVelocity(Eigen::Vector3f Vw);

        Eigen::Vector3f GetVelocity() const;

        // Set IMU pose and velocity (implicitly changes camera pose)
        void SetImuPoseVelocity(const Eigen::Matrix3f& Rwb, const Eigen::Vector3f& twb, const Eigen::Vector3f& Vwb);

        Eigen::Matrix<float, 3, 1> GetImuPosition() const;
        Eigen::Matrix<float, 3, 3> GetImuRotation();
        Sophus::SE3<float> GetImuPose();

        void SetNewBias(const IMU::Bias& b);

        ConstraintPoseImu* mpcpi;

        bool imuIsPreintegrated();
        void setIntegrated();

        bool isSet() const;

        inline Eigen::Matrix3f GetRwc() const {
            return mRwc;
        }

        inline Eigen::Vector3f GetOw() const {
            return mOw;
        }

        inline bool HasVelocity() const {
            return mbHasVelocity;
        }



    private:
        //Sophus/Eigen migration
        Sophus::SE3<float> mTcw;
        Eigen::Matrix<float, 3, 3> mRwc;
        Eigen::Matrix<float, 3, 1> mOw;
        Eigen::Matrix<float, 3, 3> mRcw;
        Eigen::Matrix<float, 3, 1> mtcw;
        bool mbHasPose;

        // IMU linear velocity
        Eigen::Vector3f mVw;
        bool mbHasVelocity;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Frame timestamp.
        double mTimeStamp;

        IMU::Bias mPredBias;

        // IMU bias
        IMU::Bias mImuBias;

        // Imu calibration
        IMU::Calib mImuCalib;

        // Imu preintegration from last keyframe
        IMU::Preintegrated* mpImuPreintegrated;
        KeyFrame* mpLastKeyFrame;

        // Pointer to previous frame
        IMU::Preintegrated* mpImuPreintegratedFrame;

        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId;

        // Reference Keyframe.
        KeyFrame* mpReferenceKF;

        // Scale pyramid info.
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvInvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;

        // Undistorted Image Bounds (computed once).
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

        map<long unsigned int, cv::Point2f> mmProjectPoints;
        map<long unsigned int, cv::Point2f> mmMatchedInImage;

        string mNameFile;

        int mnDataset;

#ifdef REGISTER_TIMES
        double mTimeORB_Ext;
        double mTimeStereoMatch;
#endif

    private:

        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).
        void UndistortKeyPoints();

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat& imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        bool mbIsSet;

        bool mbImuPreintegrated;

        std::mutex* mpMutexImu;

    public:
        GeometricCamera* mpCamera, * mpCamera2;

        //Number of KeyPoints extracted in the left and right images
        int Nleft, Nright;
        //Number of Non Lapping Keypoints
        int monoLeft, monoRight;

        //For stereo matching
        std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

        //For stereo fisheye matching
        static cv::BFMatcher BFmatcher;

        //Triangulated stereo observations using as reference the left camera. These are
        //computed during ComputeStereoFishEyeMatches
        std::vector<Eigen::Vector3f> mvStereo3Dpoints;

        //Grid for the right image
        std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        Frame(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, Sophus::SE3f& Tlr, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib& ImuCalib = IMU::Calib());

        //Stereo fisheye
        void ComputeStereoFishEyeMatches();

        bool isInFrustumChecks(MapPoint* pMP, float viewingCosLimit, bool bRight = false);

        Eigen::Vector3f UnprojectStereoFishEye(const int& i);

        cv::Mat imgLeft, imgRight;

        void PrintPointDistribution() {
            int left = 0, right = 0;
            int Nlim = (Nleft != -1) ? Nleft : N;
            for (int i = 0; i < N; i++) {
                if (mvpMapPoints[i] && !mvbOutlier[i]) {
                    if (i < Nlim) left++;
                    else right++;
                }
            }
            cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << endl;
        }

        Sophus::SE3<double> T_test;
    };

}// namespace ORB_SLAM

#endif // FRAME_H
