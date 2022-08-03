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


#ifndef STEREO_IMU_KEYFRAME_H
#define STEREO_IMU_KEYFRAME_H

#include "MapPoint.h"
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"

#include "GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

class StereoIMUKeyFrame : public KeyFrame
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mnId;
        ar & const_cast<long unsigned int&>(mnFrameId);
        ar & const_cast<double&>(mTimeStamp);
        // Grid
        ar & const_cast<int&>(mnGridCols);
        ar & const_cast<int&>(mnGridRows);
        ar & const_cast<float&>(mfGridElementWidthInv);
        ar & const_cast<float&>(mfGridElementHeightInv);

        // Variables of tracking
        //ar & mnTrackReferenceForFrame;
        //ar & mnFuseTargetForKF;
        // Variables of local mapping
        //ar & mnBALocalForKF;
        //ar & mnBAFixedForKF;
        //ar & mnNumberOfOpt;
        // Variables used by KeyFrameDatabase
        //ar & mnLoopQuery;
        //ar & mnLoopWords;
        //ar & mLoopScore;
        //ar & mnRelocQuery;
        //ar & mnRelocWords;
        //ar & mRelocScore;
        //ar & mnMergeQuery;
        //ar & mnMergeWords;
        //ar & mMergeScore;
        //ar & mnPlaceRecognitionQuery;
        //ar & mnPlaceRecognitionWords;
        //ar & mPlaceRecognitionScore;
        //ar & mbCurrentPlaceRecognition;
        // Variables of loop closing
        //serializeMatrix(ar,mTcwGBA,version);
        //serializeMatrix(ar,mTcwBefGBA,version);
        //serializeMatrix(ar,mVwbGBA,version);
        //serializeMatrix(ar,mVwbBefGBA,version);
        //ar & mBiasGBA;
        //ar & mnBAGlobalForKF;
        // Variables of Merging
        //serializeMatrix(ar,mTcwMerge,version);
        //serializeMatrix(ar,mTcwBefMerge,version);
        //serializeMatrix(ar,mTwcBefMerge,version);
        //serializeMatrix(ar,mVwbMerge,version);
        //serializeMatrix(ar,mVwbBefMerge,version);
        //ar & mBiasMerge;
        //ar & mnMergeCorrectedForKF;
        //ar & mnMergeForKF;
        //ar & mfScaleMerge;
        //ar & mnBALocalForMerge;

        // Scale
        ar & mfScale;
        // Calibration parameters
        ar & const_cast<float&>(fx);
        ar & const_cast<float&>(fy);
        ar & const_cast<float&>(invfx);
        ar & const_cast<float&>(invfy);
        ar & const_cast<float&>(cx);
        ar & const_cast<float&>(cy);
        ar & const_cast<float&>(mbf);
        ar & const_cast<float&>(mb);
        ar & const_cast<float&>(mThDepth);
        serializeMatrix(ar, mDistCoef, version);
        // Number of Keypoints
        ar & const_cast<int&>(N);
        // KeyPoints
        serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
        ar & const_cast<vector<float>& >(mvuRight);
        ar & const_cast<vector<float>& >(mvDepth);
        serializeMatrix<Archive>(ar,mDescriptors,version);
        // BOW
        ar & mBowVec;
        ar & mFeatVec;
        // Pose relative to parent
        serializeSophusSE3<Archive>(ar, mTcp, version);
        // Scale
        ar & const_cast<int&>(mnScaleLevels);
        ar & const_cast<float&>(mfScaleFactor);
        ar & const_cast<float&>(mfLogScaleFactor);
        ar & const_cast<vector<float>& >(mvScaleFactors);
        ar & const_cast<vector<float>& >(mvLevelSigma2);
        ar & const_cast<vector<float>& >(mvInvLevelSigma2);
        // Image bounds and calibration
        ar & const_cast<int&>(mnMinX);
        ar & const_cast<int&>(mnMinY);
        ar & const_cast<int&>(mnMaxX);
        ar & const_cast<int&>(mnMaxY);
        ar & boost::serialization::make_array(mK_.data(), mK_.size());
        // Pose
        serializeSophusSE3<Archive>(ar, mTcw, version);
        // MapPointsId associated to keypoints
        ar & mvBackupMapPointsId;
        // Grid
        ar & mGrid;
        // Connected KeyFrameWeight
        ar & mBackupConnectedKeyFrameIdWeights;
        // Spanning Tree and Loop Edges
        ar & mbFirstConnection;
        ar & mBackupParentId;
        ar & mvBackupChildrensId;
        ar & mvBackupLoopEdgesId;
        ar & mvBackupMergeEdgesId;
        // Bad flags
        ar & mbNotErase;
        ar & mbToBeErased;
        ar & mbBad;

        ar & mHalfBaseline;

        ar & mnOriginMapId;

        // Camera variables
        ar & mnBackupIdCamera;
        ar & mnBackupIdCamera2;

        // Fisheye variables
        ar & mvLeftToRightMatch;
        ar & mvRightToLeftMatch;
        ar & const_cast<int&>(NLeft);
        ar & const_cast<int&>(NRight);
        serializeSophusSE3<Archive>(ar, mTlr, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
        ar & mGridRight;

        // Inertial variables
        ar & mImuBias;
        ar & mBackupImuPreintegrated;
        ar & mImuCalib;
        ar & mBackupPrevKFId;
        ar & mBackupNextKFId;
        ar & bImu;
        ar & boost::serialization::make_array(mVw.data(), mVw.size());
        ar & boost::serialization::make_array(mOwb.data(), mOwb.size());
        ar & mbHasVelocity;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StereoIMUKeyFrame();
    StereoIMUKeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    virtual void SetPose(const Sophus::SE3f& Tcw);

    virtual void PreSave(set<KeyFrame*>& spKF, set<MapPoint*>& spMP, set<GeometricCamera*>& spCam);
    virtual void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId);

    Eigen::Vector3f GetVelocity();
    void SetVelocity(const Eigen::Vector3f& Vw_);
    bool isVelocitySet();

    Eigen::Vector3f GetImuPosition();
    Eigen::Matrix3f GetImuRotation();
    Sophus::SE3f GetImuPose();

    void SetNewBias(const IMU::Bias &b);
    Eigen::Vector3f GetGyroBias();

    Eigen::Vector3f GetAccBias();

    IMU::Bias GetImuBias();

    bool bImu;

    // The following variables are accesed from only 1 thread or never change (no mutex needed).

    // Preintegrated IMU measurements from previous keyframe
    KeyFrame* mPrevKF;
    KeyFrame* mNextKF;

    IMU::Bias mBiasGBA;

    IMU::Bias mBiasMerge;

    IMU::Preintegrated* mpImuPreintegrated;
    IMU::Calib mImuCalib;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // IMU position
    Eigen::Vector3f mOwb;
    // Velocity (Only used for inertial SLAM)
    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    // Imu bias
    IMU::Bias mImuBias;

    // Backup variables for inertial
    long long int mBackupPrevKFId;
    long long int mBackupNextKFId;
    IMU::Preintegrated mBackupImuPreintegrated;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
