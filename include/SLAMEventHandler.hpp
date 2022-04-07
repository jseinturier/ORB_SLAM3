#ifndef SLAM_EVENT_HANDLER_H
#define SLAM_EVENT_HANDLER_H

namespace ORB_SLAM3
{

	class SLAMEventHandler {
	public:
		void handleTrackingResetActiveMap() { handleTrackingResetActiveMapImpl(); };
		void handleTrackingReset() { handleTrackingResetImpl(); };

	private:
		virtual void handleTrackingResetActiveMapImpl() =0;

		virtual void handleTrackingResetImpl() = 0;
	};
}
#endif