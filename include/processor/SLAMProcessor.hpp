#ifndef SLAM_PROCESSOR
#define SLAM_PROCESSOR

#include<opencv2/core/core.hpp>

#include<System.h>
#include<MapDrawer.h>
#include<FrameDrawer.h>

#include<processor/SLAMEventHandler.hpp>

#include <atomic>

namespace ORB_SLAM3
{

    class SLAMProcessor : public SLAMEventHandler {

    public:

        SLAMProcessor(ORB_SLAM3::System* system, ORB_SLAM3::Tracking* tracker, ORB_SLAM3::MapDrawer* map_drawer, ORB_SLAM3::FrameDrawer* frame_drawer, cv::Size imageDisplaySize)
            : mpSystem{ system }, mpTracker{ tracker }, mpMapDrawer{ map_drawer }, mpFrameDrawer{ frame_drawer },
            m_image_display_size{ imageDisplaySize }{};

        void process();

        void run();

    private:

        // Heritage
        virtual void handleTrackingResetActiveMapImpl();
        virtual void handleTrackingResetImpl();
        virtual void handleTrackingUpdateImpl(Tracking* tracker);
        virtual void handleCameraPoseUpdateImpl(Tracking* tracker, Sophus::SE3f pose);

        ORB_SLAM3::System* mpSystem = NULL;
        ORB_SLAM3::Tracking* mpTracker = NULL;
        ORB_SLAM3::MapDrawer* mpMapDrawer = NULL;
        ORB_SLAM3::FrameDrawer* mpFrameDrawer = NULL;

        atomic<bool> m_process_graph = true;
        atomic<bool> m_process_keyframes = true;
        atomic<bool> m_process_inertial_graph = true;
        atomic<bool> m_process_opt_lba = true;
        atomic<bool> m_process_points = true;
        atomic<bool> m_process_images = true;
        atomic<bool> m_process_images_Both = false;

        cv::Size m_image_display_size;

        atomic<bool> m_process_running = false;
    };
}
#endif