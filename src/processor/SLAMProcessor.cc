#include <processor/SLAMProcessor.hpp>

// Heritage
void ORB_SLAM3::SLAMProcessor::handleTrackingResetActiveMapImpl() {

}

void ORB_SLAM3::SLAMProcessor::handleTrackingResetImpl() {

}

void ORB_SLAM3::SLAMProcessor::handleTrackingUpdateImpl(ORB_SLAM3::Tracking* tracker) {
    if (mpFrameDrawer != NULL) {
        mpFrameDrawer->Update(tracker);
    }
}

void ORB_SLAM3::SLAMProcessor::handleCameraPoseUpdateImpl(ORB_SLAM3::Tracking* tracker, Sophus::SE3f pose) {
    if (mpMapDrawer != NULL) {
        mpMapDrawer->SetCurrentCameraPose(pose);
    }
}

void ORB_SLAM3::SLAMProcessor::process()
{
    bool mbFinished = false;
    bool mbStopped = false;

    cv::Matx44f Twc = cv::Matx44f::eye();
    cv::Matx44f Twr;
    cv::Matx44f Ow = cv::Matx44f::eye();

    cv::namedWindow("ORB-SLAM3: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    unsigned int fps = 5;

    float trackedImageScale = mpTracker->GetImageScale();

    cout << "Starting the Viewer" << endl;
    while (1)
    {
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Ow);

        if (bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
        }
        else
        {
            mpSystem->DeactivateLocalizationMode();
        }

        if (bStepByStep)
        {
            mpTracker->SetStepByStep(true);
        }
        else
        {
            mpTracker->SetStepByStep(false);
        }

        mpMapDrawer->DrawCurrentCamera(Twc);

        if (m_process_keyframes || m_process_graph || m_process_inertial_graph || m_process_opt_lba)
            mpMapDrawer->DrawKeyFrames();

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
/*
        if (menuReset)
        {
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if (bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            mpSystem->ResetActiveMap();
            menuReset = false;
        }
*/
        if (mpSystem->isFinished()) {
            m_process_running = false;
        }
/*
                if (menuStop)
                {
                    if (bLocalizationMode)
                        mpSystem->DeactivateLocalizationMode();

                    // Stop all threads
                    mpSystem->Shutdown();

                    // Save camera trajectory
                    mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
                    mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
                    menuStop = false;
                }
*/
    }
}
    void ORB_SLAM3::SLAMProcessor::run() {
        m_process_running = true;

        while (m_process_running) {
            process();
        }
    }
