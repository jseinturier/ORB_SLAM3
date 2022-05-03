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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <mutex>

namespace ORB_SLAM3
{


/*

# Map points list begining
@PTSB 

# Map points list ending
@PTSB 

*/

MapDrawer::MapDrawer(Atlas* pAtlas):mpAtlas(pAtlas)
{

}

void MapDrawer::DrawMapPoints(std::ostream& stream)
{

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    stream << "@PTSB " << pActiveMap->GetId() << " " << vpMPs.size() << " " << vpRefMPs.size() << std::endl;
/*
    for (set<MapPoint*>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
    {
        MapPoint* point = *sit;

        if (point->isBad())
            continue;
        Eigen::Matrix<float, 3, 1> pos = point->GetWorldPos();
        stream << "@PT R " << point->mnId << " " << point->GetReferenceKeyFrame()->mnId << " " << pos(0) << " " << pos(1) << " " << pos(2) << std::endl;
    }

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        MapPoint* point = vpMPs[i];

        if(point->isBad() || spRefMPs.count(point))
            continue;
        Eigen::Matrix<float,3,1> pos = point->GetWorldPos();
        stream << "@PT A " << point->mnId << " " << point->GetReferenceKeyFrame()->mnId << " " << pos(0) << " " << pos(1) << " " << pos(2) << std::endl;
    }
*/
    std::cout << "@PTSE" << std::endl;
}

void MapDrawer::DrawKeyFrames()
{
    std::cout << "[SLAMEventHandler][DrawKeyFrames()] start" << std::endl;

    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if(!pActiveMap)
        return;

    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

    // Display the Key Frames 3D positions as 4x4 matrix (Row Major)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
        unsigned int index_color = pKF->mnOriginMapId;

        std::cout << "[SLAMEventHandler][DrawKeyFrames()]   KeyFrame " << pKF->mnId;

        if(!pKF->GetParent()) // It is the first KF in the map
        {
            std::cout << " (first)";
        }
        else
        {

            if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
            {
                std::cout << " (optimized)";
            }
            else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
            {
                std::cout << " (fixed)";
            }
            else
            {
                std::cout << "(idle)";
            }
        }

        std::cout << " Pos: [mat4x4_RM][" << Twc(0, 0) << " " << Twc(0, 1) << " " << Twc(0, 2) << " " << Twc(0, 3)
                                          << Twc(1, 0) << " " << Twc(1, 1) << " " << Twc(1, 2) << " " << Twc(1, 3)
                                          << Twc(2, 0) << " " << Twc(2, 1) << " " << Twc(2, 2) << " " << Twc(2, 3)
                                          << Twc(3, 0) << " " << Twc(3, 1) << " " << Twc(3, 2) << " " << Twc(3, 3) << "]" << std::endl;


        // Covisibilty graph
        std::cout << "[SLAMEventHandler][DrawKeyFrames()]     Covisibility: ";
        const vector<KeyFrame*> vCovKFs = pKF->GetCovisiblesByWeight(100);
        Eigen::Vector3f Ow = pKF->GetCameraCenter();
        if (!vCovKFs.empty())
        {
            for (vector<KeyFrame*>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++)
            {
                std::cout << (*vit)->mnId << ", ";
            }
            std::cout << std::endl;
        }
        else {
            std::cout << "empty" << std::endl;
        }

        // Spanning tree
        std::cout << "[SLAMEventHandler][DrawKeyFrames()]     Tree parent: ";
        KeyFrame* pParent = pKF->GetParent();
        if (pParent)
        {
            std::cout << pParent->mnId;
        }
        else {
            std::cout << "N";
        }
        std::cout << std::endl;

        // Loops
        std::cout << "[SLAMEventHandler][DrawKeyFrames()]     Loops: ";
        set<KeyFrame*> sLoopKFs = pKF->GetLoopEdges();
        for (set<KeyFrame*>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
        {
            std::cout << (*sit)->mnId << ", ";
        }
        std::cout << std::endl;

        // IMU graph
        if (pActiveMap->isImuInitialized()) {
            std::cout << "[SLAMEventHandler][DrawKeyFrames()]     IMU Link: ";
            KeyFrame* pKFi = pKFi;
            KeyFrame* pNext = pKFi->mNextKF;
            if (pNext)
            {
                std::cout << pNext->mnId << std::endl;
            }
            else {
                std::cout << " none" << std::endl;
            }
        }
    }
/*
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == pActiveMap)
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat*)Twc.data());

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
*/
}

void MapDrawer::DrawCurrentCamera(cv::Matx44f& Twc)
{
    std::cout << "[SLAMEventHandler][DrawCurrentCamera()] Camera : [" << Twc(0, 0) << " " << Twc(0, 1) << " " << Twc(0, 2) << " " << Twc(0, 3) 
                                                                      << Twc(1, 0) << " " << Twc(1, 1) << " " << Twc(1, 2) << " " << Twc(1, 3) 
                                                                      << Twc(2, 0) << " " << Twc(2, 1) << " " << Twc(2, 2) << " " << Twc(2, 3) 
                                                                      << Twc(3, 0) << " " << Twc(3, 1) << " " << Twc(3, 2) << " " << Twc(3, 3) << "]" << std::endl;
}


void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();

    //std::cout << "[MapDrawer][SetCurrentCameraPose()] camera pose: " << mCameraPose.matrix() << std::endl;
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(cv::Matx44f& M, cv::Matx44f& MOw)
{
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (size_t row = 0; row < 4; row++) {
        for (size_t col = 0; col < 4; col++) {
            M(row, col) = Twc(row, col);
        }
    }

    cv::setIdentity(MOw);

    MOw(0, 3) = Twc(0, 3);
    MOw(0, 3) = Twc(1, 3);
    MOw(0, 3) = Twc(2, 3);
}


} //namespace ORB_SLAM
