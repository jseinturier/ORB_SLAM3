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

MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings):mpAtlas(pAtlas)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::DrawMapPoints()
{
    std::cout << "@PTSB" << std::endl;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    
    std::cout << "[SLAMEventHandler][DrawMapPoints()]   - All" << std::endl;
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        std::cout << "[SLAMEventHandler][DrawMapPoints()]     Point (" << pos(0) << ", " << pos(1) << ", " << pos(2) << ")" << std::endl;
    }
   
    std::cout << "[SLAMEventHandler][DrawMapPoints()]   - Reference" << std::endl;
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        std::cout << "[SLAMEventHandler][DrawMapPoints()]     Point (" << pos(0) << ", " << pos(1) << ", " << pos(2) << ")" << std::endl;

    }
    std::cout << "@PTSE" << std::endl;
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba)
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

        // Loops
        std::cout << "[SLAMEventHandler][DrawKeyFrames()]     Loops: ";
        set<KeyFrame*> sLoopKFs = pKF->GetLoopEdges();
        for (set<KeyFrame*>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
        {
            std::cout << (*sit)->mnId << ", ";
        }

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

void MapDrawer::DrawCurrentCamera(cv::Mat &Twc)
{
    std::cout << "[SLAMEventHandler][DrawCurrentCamera()] Camera : [" << Twc.at<double>(0, 0) << " " << Twc.at<double>(0, 1) << " " << Twc.at<double>(0, 2) << " " << Twc.at<double>(0, 3) 
                                                                      << Twc.at<double>(1, 0) << " " << Twc.at<double>(1, 1) << " " << Twc.at<double>(1, 2) << " " << Twc.at<double>(1, 3) 
                                                                      << Twc.at<double>(2, 0) << " " << Twc.at<double>(2, 1) << " " << Twc.at<double>(2, 2) << " " << Twc.at<double>(2, 3) 
                                                                      << Twc.at<double>(3, 0) << " " << Twc.at<double>(3, 1) << " " << Twc.at<double>(3, 2) << " " << Twc.at<double>(3, 3) << "]" << std::endl;
}


void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}
} //namespace ORB_SLAM
