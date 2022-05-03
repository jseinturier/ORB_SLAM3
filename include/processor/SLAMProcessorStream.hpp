#ifndef SLAM_PROCESSOR_STREAM
#define SLAM_PROCESSOR_STREAM

#include<processor/SLAMProcessor.hpp>

namespace ORB_SLAM3
{
    class SLAMProcessorStream : public SLAMProcessor {

    public:

        SLAMProcessorStream(ORB_SLAM3::System* system, ORB_SLAM3::Tracking* tracker, ORB_SLAM3::MapDrawer* map_drawer, ORB_SLAM3::FrameDrawer* frame_drawer, cv::Size imageDisplaySize)
            : SLAMProcessor(system, tracker, map_drawer, frame_drawer, imageDisplaySize) {};

    private:

        // Heritage
        virtual void handleTrackingResetActiveMapImpl();
        virtual void handleTrackingResetImpl();
        virtual void handleTrackingUpdateImpl(Tracking* tracker);
        virtual void handleFrameUpdateImpl(Tracking* tracker, Frame* frame);
        virtual void handleKeyFrameUpdateImpl(Tracking* tracker, KeyFrame* frame);

        virtual void processImpl();
    };
}

#endif