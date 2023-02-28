#include <processor/SLAMProcessorStream.hpp>

// Heritage
void ORB_SLAM3::SLAMProcessorStream::handleTrackingResetActiveMapImpl() {
    m_stream << "@TR_ACTIVE_MAP_RESET" << std::endl;
}

void ORB_SLAM3::SLAMProcessorStream::handleTrackingResetImpl() {
    m_stream << "@TR_RESET" << std::endl;
}

void ORB_SLAM3::SLAMProcessorStream::handleTrackingUpdateImpl(ORB_SLAM3::Tracking* tracker) {
    m_stream << "@TR_UPDATE" << std::endl;
    if (mpFrameDrawer != NULL) {
        mpFrameDrawer->Update(tracker);
    }

    //process();
}

void ORB_SLAM3::SLAMProcessorStream::handleFrameUpdateImpl(ORB_SLAM3::Tracking* tracker, Frame* frame) {
    m_stream << "@F_UPDATE " << frame->mnId << std::endl;

    // Update camera pose
    unique_lock<mutex> lock(m_camera_pose_mutex);
    m_camera_pose = frame->GetPose().inverse().matrix();

    mpFrameDrawer->Update(tracker);
}

void ORB_SLAM3::SLAMProcessorStream::handleKeyFrameUpdateImpl(ORB_SLAM3::Tracking* tracker, KeyFrame* frame) {
    m_stream << "@KF_UPDATE " << frame->mnId << std::endl;

    // Update camera pose
    unique_lock<mutex> lock(m_camera_pose_mutex);
    m_camera_pose = frame->GetPoseInverse().matrix();
}

void ORB_SLAM3::SLAMProcessorStream::processImpl()
{
    bool mbFinished = false;
    bool mbStopped = false;

    unsigned int fps = 5;

    float trackedImageScale = mpTracker->GetImageScale();

    /*
        mpMapDrawer->DrawCurrentCamera(Twc);

        if (m_process_keyframes || m_process_graph || m_process_inertial_graph || m_process_opt_lba)
            mpMapDrawer->DrawKeyFrames();
    */
    
    // *DEBUG* - START - Draw image frame 
    if (m_process_points)
        stream_map_points();

    cv::Mat toShow;
    cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

    if (m_process_images_Both) {
        cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
        cv::hconcat(im, imRight, toShow);
    }
    else {
        toShow = im;
    }

    cv::namedWindow("ORB-SLAM3: Current Frame");
    cv::resize(toShow, toShow, m_image_display_size);

    cv::imshow("ORB-SLAM3: Current Frame", toShow);
    cv::waitKey(1e3 / fps);
    // *DEBUG* - END

    if (mpSystem->isFinished()) {
        m_process_running = false;
    }
}

void ORB_SLAM3::SLAMProcessorStream::stream_map_points() {
    Map* pActiveMap = mpSystem->atlas()->GetCurrentMap();

    if (!pActiveMap)
        return;

    const vector<MapPoint*>& vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*>& vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty())
        return;

    m_stream << "@MAP_PTS_S " << pActiveMap->GetId() << " " << vpMPs.size() << " " << vpRefMPs.size() << std::endl;
        
    for (set<MapPoint*>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
    {
        MapPoint* point = *sit;

        if (point->isBad())
            continue;
        
        Eigen::Matrix<float, 3, 1> pos = point->GetWorldPos();
        m_stream << "@PT R " << point->mnId << " " << point->GetReferenceKeyFrame()->mnId << " " << pos(0) << " " << pos(1) << " " << pos(2) << std::endl;
    }

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        MapPoint* point = vpMPs[i];

        if(point->isBad() || spRefMPs.count(point))
            continue;
        
        Eigen::Matrix<float,3,1> pos = point->GetWorldPos();
        m_stream << "@PT A " << point->mnId << " " << point->GetReferenceKeyFrame()->mnId << " " << pos(0) << " " << pos(1) << " " << pos(2) << std::endl;
    }
        
    m_stream << "@MAP_PTS_E" << std::endl;
}

void ORB_SLAM3::SLAMProcessorStream::stream_camera_current_transform() {
    
    m_stream << "@CAM " << m_camera_pose(0, 0) << " " << m_camera_pose(0, 1) << " " << m_camera_pose(0, 2) << " " << m_camera_pose(0, 3)
                        << m_camera_pose(1, 0) << " " << m_camera_pose(1, 1) << " " << m_camera_pose(1, 2) << " " << m_camera_pose(1, 3)
                        << m_camera_pose(2, 0) << " " << m_camera_pose(2, 1) << " " << m_camera_pose(2, 2) << " " << m_camera_pose(2, 3)
                        << m_camera_pose(3, 0) << " " << m_camera_pose(3, 1) << " " << m_camera_pose(3, 2) << " " << m_camera_pose(3, 3) << std::endl;
}

