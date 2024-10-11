/*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __VSC_PROCESS_INCLUDED__
#define __VSC_PROCESS_INCLUDED__

/**
 * ROS2 Includes
 */
#include <rclcpp/rclcpp.hpp>
#include <hri_interfaces/srv/emergency_stop.hpp>
#include <hri_interfaces/srv/key_value.hpp>
#include <hri_interfaces/srv/key_string.hpp>

/**
 * HRI_COMMON Includes
 */
#include "MsgHandler.h"
#include "VehicleMessages.h"
#include "VehicleInterface.h"

namespace hri_safety_sense {

	// Diagnostics
	struct ErrorCounterType {
		uint32_t sendErrorCount;
		uint32_t invalidRxMsgCount;
	};

	/**
	 * Local Definitions
	 */
	const unsigned int VSC_INTERFACE_RATE = 50; /* 50 Hz */
	const unsigned int VSC_HEARTBEAT_RATE = 20; /* 20 Hz */

	class VscProcess {
	   public:
		  VscProcess();
		  ~VscProcess();

		  // Main loop
		  void processOneLoop();

        bool EmergencyStop(const std::shared_ptr<hri_interfaces::srv::EmergencyStop::Request> &req,
                           const std::shared_ptr<hri_interfaces::srv::EmergencyStop::Response> &res);
        // ROS 2 Callback's
		  bool KeyValue(const std::shared_ptr<hri_interfaces::srv::KeyValue::Request> &req,
                        const std::shared_ptr<hri_interfaces::srv::KeyValue::Response> &res);
        bool KeyString(const std::shared_ptr<hri_interfaces::srv::KeyString::Request> &req,
                         const std::shared_ptr<hri_interfaces::srv::KeyString::Response> &res);

	   private:

		  void readFromVehicle();
		  int handleHeartbeatMsg(VscMsgType& recvMsg);

		  // Local State
		  uint32_t 				myEStopState;
		  ErrorCounterType 		errorCounts;

		  // ROS
		  rclcpp::Node::SharedPtr rosNode;
//		  ros::Timer 	  		mainLoopTimer;
          rclcpp::TimerBase::SharedPtr mainLoopTimer;
//		  ros::ServiceServer    estopServ, keyValueServ, keyStringServ;
          rclcpp::Service<hri_interfaces::srv::EmergencyStop>::SharedPtr estopServ;
          rclcpp::Service<hri_interfaces::srv::KeyValue>::SharedPtr keyValueServ;
          rclcpp::Service<hri_interfaces::srv::KeyString>::SharedPtr keyStringServ;
		  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr estopPub;
		  rclcpp::Time 			lastDataRx, lastTxTime;

		  // Message Handlers
          std::shared_ptr<hri_safety_sense::MsgHandler> joystickHandler;
//		  MsgHandler			*joystickHandler;

		  /* File descriptor for VSC Interface */
		  VscInterfaceType		*vscInterface;

	};

} // namespace


#endif
