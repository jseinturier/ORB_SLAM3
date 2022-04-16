#ifndef SLAM_EVENT_HANDLER_H
#define SLAM_EVENT_HANDLER_H

#include<Tracking.h>

namespace ORB_SLAM3
{

	class Tracking;

	class System;

	class SLAMEventHandler {
	public:
		void handleTrackingResetActiveMap() { handleTrackingResetActiveMapImpl(); };
		void handleTrackingReset() { handleTrackingResetImpl(); };

		void handleTrackingUpdate(Tracking* tracker) { handleTrackingUpdateImpl(tracker); };

		void handleCameraPoseUpdate(Tracking* tracker, Sophus::SE3f pose) { handleCameraPoseUpdateImpl(tracker, pose); };

	private:
		virtual void handleTrackingResetActiveMapImpl() =0;

		virtual void handleTrackingResetImpl() = 0;

		virtual void handleTrackingUpdateImpl(Tracking* tracker) = 0;

		virtual void handleCameraPoseUpdateImpl(Tracking* tracker, Sophus::SE3f pose) = 0;
	};
}
#endif