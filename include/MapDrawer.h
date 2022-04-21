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


#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include "Settings.h"

#include<mutex>

namespace ORB_SLAM3
{

class Settings;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas);

    void newParameterLoader(Settings* settings);

    Atlas* mpAtlas;

    void DrawMapPoints(std::ostream& stream=std::cout);
    void DrawKeyFrames();
    void DrawCurrentCamera(cv::Matx44f& Twc);
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(cv::Matx44f& M, cv::Matx44f& MOw);

    inline bool integrate_graph() { return menuShowGraph; };
    inline void integrate_graph(bool integrate) { menuShowGraph = integrate; };

    inline bool integrate_key_frames() { return menuShowKeyFrames; };
    inline void integrate_key_frames(bool integrate) { menuShowKeyFrames = integrate; };

    inline bool integrate_intertial_graph() { return menuShowInertialGraph; };
    inline void integrate_intertial_graph(bool integrate) { menuShowInertialGraph = integrate; };

    inline bool integrate_opt_lba() { return menuShowOptLba; };
    inline void integrate_opt_lba(bool integrate) { menuShowOptLba = integrate; };

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    bool menuShowGraph = true;
    bool menuShowKeyFrames = true;
    bool menuShowInertialGraph = true;
    bool menuShowOptLba = true;

    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
