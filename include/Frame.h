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


#ifndef MOVS_FRAME_H
#define MOVS_FRAME_H

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
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for Monocular cameras.
    Frame(const double &timeStamp, Frame* pPrevF = static_cast<Frame*>(NULL));

    // Destructor
    // ~Frame();

    // Set the camera pose. (Imu pose is not modified!)
    void SetPose(const Sophus::SE3<float> &Tcw);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    Eigen::Vector3f inRefCoordinates(Eigen::Vector3f pCw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns inverse of rotation
    inline Eigen::Matrix3f GetRotationInverse(){
        return mRwc;
    }

    inline Sophus::SE3<float> GetPose() const {
        //TODO: can the Frame pose be accsessed from several threads? should this be protected somehow?
        return mTcw;
    }

    inline Eigen::Matrix3f GetRwc() const {
        return mRwc;
    }

    inline Eigen::Vector3f GetOw() const {
        return mOw;
    }

    inline bool HasPose() const {
        return mbHasPose;
    }

    // Frame timestamp.
    double mTimeStamp;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    int mnCloseMPs;

    // Imu preintegration from last keyframe
    KeyFrame* mpLastKeyFrame;

    // Pointer to previous frame
    Frame* mpPrevFrame;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    static bool mbInitialComputations;

    string mNameFile;

    int mnDataset;

    Sophus::SE3<double> T_test;

private:
    //Sophus/Eigen migration
    Sophus::SE3<float> mTcw;
    Eigen::Matrix<float,3,3> mRwc;
    Eigen::Matrix<float,3,1> mOw;
    Eigen::Matrix<float,3,3> mRcw;
    Eigen::Matrix<float,3,1> mtcw;
    bool mbHasPose;
};

}// namespace ORB_SLAM

#endif // FRAME_H
