#include <processor/SLAMProcessorStream.hpp>

// Heritage
void ORB_SLAM3::SLAMProcessorStream::handleTrackingResetActiveMapImpl() {
    std::cout << "[handleTrackingResetActiveMapImpl()]" << std::endl;
}

void ORB_SLAM3::SLAMProcessorStream::handleTrackingResetImpl() {
    std::cout << "[handleTrackingResetImpl()]" << std::endl;
}

void ORB_SLAM3::SLAMProcessorStream::handleTrackingUpdateImpl(ORB_SLAM3::Tracking* tracker) {
    std::cout << "[handleTrackingUpdateImpl(Tracking*)]" << std::endl;
    if (mpFrameDrawer != NULL) {
        mpFrameDrawer->Update(tracker);
    }

    //process();
}

void ORB_SLAM3::SLAMProcessorStream::handleFrameUpdateImpl(ORB_SLAM3::Tracking* tracker, Frame* frame) {
    std::cout << "[handleFrameUpdateImpl(Tracking*, Frame*)] " << frame->mnId << std::endl;
    if (mpMapDrawer != NULL) {
        mpMapDrawer->SetCurrentCameraPose(frame->GetPose());
        mpFrameDrawer->Update(tracker);
    }
}

void ORB_SLAM3::SLAMProcessorStream::handleKeyFrameUpdateImpl(ORB_SLAM3::Tracking* tracker, KeyFrame* frame) {
    std::cout << "[handleKeyFrameUpdateImpl(Tracking*, KeyFrame*)] " << frame->mnId << std::endl;
    if (mpMapDrawer != NULL) {
        mpMapDrawer->SetCurrentCameraPose(frame->GetPose());
    }
}

void ORB_SLAM3::SLAMProcessorStream::processImpl()
{
    bool mbFinished = false;
    bool mbStopped = false;

    cv::Matx44f Twc = cv::Matx44f::eye();
    cv::Matx44f Twr;
    cv::Matx44f Ow = cv::Matx44f::eye();

    cv::namedWindow("ORB-SLAM3: Current Frame");

    unsigned int fps = 5;

    float trackedImageScale = mpTracker->GetImageScale();

    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Ow);
/*
    mpMapDrawer->DrawCurrentCamera(Twc);

    if (m_process_keyframes || m_process_graph || m_process_inertial_graph || m_process_opt_lba)
        mpMapDrawer->DrawKeyFrames();
*/
    if (m_process_points)
        mpMapDrawer->DrawMapPoints();

    cv::Mat toShow;
    cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

    if (m_process_images_Both) {
        cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
        cv::hconcat(im, imRight, toShow);
    }
    else {
        toShow = im;
    }

    cv::resize(toShow, toShow, m_image_display_size);

    cv::imshow("ORB-SLAM3: Current Frame", toShow);
    cv::waitKey(1e3 / fps);

    if (mpSystem->isFinished()) {
        m_process_running = false;
    }

}
