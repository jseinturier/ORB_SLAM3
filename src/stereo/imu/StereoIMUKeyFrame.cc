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

#include "stereo/imu/StereoIMUKeyFrame.h"
#include "Converter.h"
#include "ImuTypes.h"
#include<mutex>

namespace ORB_SLAM3
{

StereoIMUKeyFrame::StereoIMUKeyFrame(): KeyFrame() 
{

}

StereoIMUKeyFrame::StereoIMUKeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB): KeyFrame(F, pMap, pKFDB),
                                                                                    bImu{ pMap->isImuInitialized() }, mPrevKF{ NULL }, mNextKF{ NULL }, mpImuPreintegrated{ F.mpImuPreintegrated },
                                                                                    mImuCalib{ F.mImuCalib }, mbHasVelocity{ false }
{

    if(!F.HasVelocity()) {
        mVw.setZero();
        mbHasVelocity = false;
    }
    else
    {
        mVw = F.GetVelocity();
        mbHasVelocity = true;
    }

    mImuBias = F.mImuBias;
}

Eigen::Vector3f StereoIMUKeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return mVw;
}

void StereoIMUKeyFrame::SetVelocity(const Eigen::Vector3f& Vw)
{
    unique_lock<mutex> lock(mMutexPose);
    mVw = Vw;
    mbHasVelocity = true;
}

bool StereoIMUKeyFrame::isVelocitySet()
{
    unique_lock<mutex> lock(mMutexPose);
    return mbHasVelocity;
}

Eigen::Vector3f StereoIMUKeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return mOwb;
}

Eigen::Matrix3f StereoIMUKeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Sophus::SE3f StereoIMUKeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc * mImuCalib.mTcb;
}

void StereoIMUKeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

Eigen::Vector3f StereoIMUKeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

Eigen::Vector3f StereoIMUKeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias StereoIMUKeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void StereoIMUKeyFrame::SetPose(const Sophus::SE3f& Tcw)
{
    KeyFrame::SetPose(Tcw);

    unique_lock<mutex> lock(mMutexPose);

    if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
    }
}


void StereoIMUKeyFrame::PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP, set<GeometricCamera*>& spCam)
{

    KeyFrame::PreSave(spKF, spMP, spCam);

    //Inertial data
    mBackupPrevKFId = -1;
    if(mPrevKF && spKF.find(mPrevKF) != spKF.end())
        mBackupPrevKFId = mPrevKF->mnId;

    mBackupNextKFId = -1;
    if(mNextKF && spKF.find(mNextKF) != spKF.end())
        mBackupNextKFId = mNextKF->mnId;

    if(mpImuPreintegrated)
        mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
}

void StereoIMUKeyFrame::PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId){
    
    KeyFrame::PostLoad(mpKFid, mpMPid, mpCamId);

    //Inertial data
    if(mBackupPrevKFId != -1)
    {
        mPrevKF = mpKFid[mBackupPrevKFId];
    }
    if(mBackupNextKFId != -1)
    {
        mNextKF = mpKFid[mBackupNextKFId];
    }
    mpImuPreintegrated = &mBackupImuPreintegrated;
}

} //namespace ORB_SLAM
