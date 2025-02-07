#ifndef SLAM_PROCESSOR
#define SLAM_PROCESSOR

#include<opencv2/core/core.hpp>

#include<System.h>
#include<MapDrawer.h>
#include<FrameDrawer.h>

#include<processor/SLAMEventHandler.hpp>

#include <atomic>

#include<mutex>

namespace ORB_SLAM3
{

    class SLAMProcessor : public SLAMEventHandler {

    public:

        SLAMProcessor(ORB_SLAM3::System* system, ORB_SLAM3::Tracking* tracker, ORB_SLAM3::MapDrawer* map_drawer, ORB_SLAM3::FrameDrawer* frame_drawer, cv::Size imageDisplaySize)
            : mpSystem{ system }, mpTracker{ tracker }, mpMapDrawer{ map_drawer }, mpFrameDrawer{ frame_drawer },
            m_image_display_size{ imageDisplaySize }{};

        void process() { processImpl(); };

        void run(){
        m_process_running = true;

        while (m_process_running) {
            process();
        };
    }

	protected:
        ORB_SLAM3::System* mpSystem = NULL;
        ORB_SLAM3::Tracking* mpTracker = NULL;
        ORB_SLAM3::MapDrawer* mpMapDrawer = NULL;
        ORB_SLAM3::FrameDrawer* mpFrameDrawer = NULL;

        Eigen::Matrix4f m_camera_pose;

        std::mutex m_camera_pose_mutex;

        atomic<bool> m_process_graph = true;
        atomic<bool> m_process_keyframes = true;
        atomic<bool> m_process_inertial_graph = true;
        atomic<bool> m_process_opt_lba = true;
        atomic<bool> m_process_points = true;
        atomic<bool> m_process_images = true;
        atomic<bool> m_process_images_Both = false;

        cv::Size m_image_display_size;

        atomic<bool> m_process_running = false;

    private:

        // Heritage
        virtual void handleTrackingResetActiveMapImpl() = 0;
        virtual void handleTrackingResetImpl() = 0;
        virtual void handleTrackingUpdateImpl(Tracking* tracker) = 0;
        virtual void handleFrameUpdateImpl(Tracking* tracker, Frame* frame) = 0;
        virtual void handleKeyFrameUpdateImpl(Tracking* tracker, KeyFrame* frame) = 0;

        virtual void processImpl() = 0;
    };
}
#endif