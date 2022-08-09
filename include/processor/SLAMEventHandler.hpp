#ifndef SLAM_EVENT_HANDLER_H
#define SLAM_EVENT_HANDLER_H

#include<Tracking.h>

namespace ORB_SLAM3
{

	class Tracking;

	class System;

	class SLAMEventHandler {
	public:
		void handleMapCreated(Atlas* atlas, Map* map) {};

		void handleTrackingResetActiveMap() { handleTrackingResetActiveMapImpl(); };

		void handleTrackingReset() { handleTrackingResetImpl(); };

		void handleTrackingUpdate(Tracking* tracker) { handleTrackingUpdateImpl(tracker); };

		void handleFrameUpdate(Tracking* tracker, Frame* frame) { handleFrameUpdateImpl(tracker, frame); };

		void handleKeyFrameUpdate(Tracking* tracker, KeyFrame* frame) { handleKeyFrameUpdateImpl(tracker, frame); };

	private:
		virtual void handleTrackingResetActiveMapImpl() =0;

		virtual void handleTrackingResetImpl() = 0;

		virtual void handleTrackingUpdateImpl(Tracking* tracker) = 0;

		virtual void handleFrameUpdateImpl(Tracking* tracker, Frame* frame) = 0;

		virtual void handleKeyFrameUpdateImpl(Tracking* tracker, KeyFrame* frame) = 0;
	};
}
#endif