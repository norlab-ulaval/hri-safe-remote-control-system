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

/**
 * ROS Includes
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

/**
 * System Includes
 */
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/resource.h>

/**
 * Includes
 */
#include <VscProcess.h>
#include <JoystickHandler.h>
#include <VehicleInterface.h>
#include <VehicleMessages.h>

using namespace hri_safety_sense;

VscProcess::VscProcess(const std::shared_ptr<rclcpp::Node> &node) :
	myEStopState(0)
{
    rosNode = node;

	std::string serialPort;
	int serialSpeed;
	bool set_priority;
    rosNode->declare_parameter<std::string>("port", "/dev/ttyACM0");
    rosNode->declare_parameter<int>("serial_speed", 115200);
    rosNode->declare_parameter<bool>("set_priority", false);

	// Get params
    if (rosNode->get_parameter("port", serialPort)) {
        RCLCPP_INFO(rosNode->get_logger(), "Serial Port updated to: %s", serialPort.c_str());
    }

    if (rosNode->get_parameter("serial_speed", serialSpeed)) {
        RCLCPP_INFO(rosNode->get_logger(), "Serial Port Speed updated to: %d", serialSpeed);
    }

	/* Open VSC Interface */
	vscInterface = vsc_initialize(serialPort.c_str(),serialSpeed);
	if (vscInterface == NULL) {
		RCLCPP_FATAL(rosNode->get_logger(), "Cannot open serial port! (%s, %i)",serialPort.c_str(),serialSpeed);
	} else {
		RCLCPP_INFO(rosNode->get_logger(), "Connected to VSC on %s : %i",serialPort.c_str(),serialSpeed);
	}

	// Attempt to Set priority
    if (rosNode->get_parameter("set_priority", set_priority)) {
		RCLCPP_INFO(rosNode->get_logger(), "Set priority updated to:  %i", set_priority);
    }

	if(set_priority) {
		if(setpriority(PRIO_PROCESS, 0, -19) == -1) {
			RCLCPP_ERROR(rosNode->get_logger(), "UNABLE TO SET PRIORITY OF PROCESS! (%i, %s)",errno,strerror(errno));
		}
	}

	// Create Message Handlers
	joystickHandler = std::make_shared<JoystickHandler>(node);

	// EStop callback
    estopServ = rosNode->create_service<hri_interfaces::srv::EmergencyStop>(
        "emergency_stop",
        std::bind(&VscProcess::EmergencyStop, this, std::placeholders::_1, std::placeholders::_2)
    );

    // KeyValue callbacks
    keyValueServ = rosNode->create_service<hri_interfaces::srv::KeyValue>(
        "key_value",
        std::bind(&VscProcess::KeyValue, this, std::placeholders::_1, std::placeholders::_2)
    );
    keyStringServ = rosNode->create_service<hri_interfaces::srv::KeyString>(
        "key_string",
        std::bind(&VscProcess::KeyString, this, std::placeholders::_1, std::placeholders::_2)
    );

// Publish Emergency Stop Status
estopPub = rosNode->create_publisher<std_msgs::msg::UInt32>("emergency_stop", 10);


	// Main Loop Timer Callback
//	mainLoopTimer = rosNode.createTimer(ros::Duration(1.0/VSC_INTERFACE_RATE), &VscProcess::processOneLoop, this);
    mainLoopTimer = rosNode->create_wall_timer(
            std::chrono::duration<double>(1.0 / VSC_INTERFACE_RATE),
            std::bind(&VscProcess::processOneLoop, this)
        );
	// Init last time to now
	lastDataRx = rosNode->now();

	// Clear all error counters
	memset(&errorCounts, 0, sizeof(errorCounts));
}

VscProcess::~VscProcess()
{
    // Destroy vscInterface
	vsc_cleanup(vscInterface);
}

bool VscProcess::EmergencyStop(const std::shared_ptr<hri_interfaces::srv::EmergencyStop::Request> &req,
                           const std::shared_ptr<hri_interfaces::srv::EmergencyStop::Response> &res)
{
	myEStopState = (uint32_t) req->emergency_stop;

	RCLCPP_WARN(rosNode->get_logger(), "VscProcess::EmergencyStop: to 0x%x", myEStopState);

	return true;
}

bool VscProcess::KeyValue(const std::shared_ptr<hri_interfaces::srv::KeyValue::Request> &req,
                        const std::shared_ptr<hri_interfaces::srv::KeyValue::Response> &res)
{
	// Send heartbeat message to vehicle in every state
	vsc_send_user_feedback(vscInterface, req->key, req->value);

	RCLCPP_INFO(rosNode->get_logger(), "VscProcess::KeyValue: 0x%x, 0x%x", req->key, req->value);

	return true;
}

bool VscProcess::KeyString(const std::shared_ptr<hri_interfaces::srv::KeyString::Request> &req,
                         const std::shared_ptr<hri_interfaces::srv::KeyString::Response> &res)
{
	// Send heartbeat message to vehicle in every state
	vsc_send_user_feedback_string(vscInterface, req->key, req->value.c_str());

	RCLCPP_INFO(rosNode->get_logger(), "VscProcess::KeyValue: 0x%x, %s", req->key, req->value.c_str());

	return true;
}


void VscProcess::processOneLoop()
{
	// Send heartbeat message to vehicle in every state
	vsc_send_heartbeat(vscInterface, myEStopState);

	// Check for new data from vehicle in every state
	readFromVehicle();
}

int VscProcess::handleHeartbeatMsg(VscMsgType& recvMsg)
{
	int retVal = 0;

	if(recvMsg.msg.length == sizeof(HeartbeatMsgType)) {
		RCLCPP_DEBUG(rosNode->get_logger(), "Received Heartbeat from VSC");

		HeartbeatMsgType *msgPtr = (HeartbeatMsgType*)recvMsg.msg.data;

		// Publish E-STOP Values
		std_msgs::msg::UInt32 estopValue;
		estopValue.data = msgPtr->EStopStatus;
		estopPub->publish(estopValue);

		if(msgPtr->EStopStatus > 0) {
			RCLCPP_WARN(rosNode->get_logger(),"Received ESTOP from the vehicle!!! 0x%x",msgPtr->EStopStatus);
		}

	} else {
		RCLCPP_WARN(rosNode->get_logger(),"RECEIVED HEARTBEAT WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
				(unsigned int)sizeof(HeartbeatMsgType), recvMsg.msg.length);
		retVal = 1;
	}

	return retVal;
}

void VscProcess::readFromVehicle()
{
	VscMsgType recvMsg;

	/* Read all messages */
	while (vsc_read_next_msg(vscInterface, &recvMsg) > 0) {
		/* Read next Vsc Message */
		switch (recvMsg.msg.msgType) {
		case MSG_VSC_HEARTBEAT:
			if(handleHeartbeatMsg(recvMsg) == 0) {
				lastDataRx = rosNode->now();
			}

			break;
		case MSG_VSC_JOYSTICK:
			if(joystickHandler->handleNewMsg(recvMsg) == 0) {
				lastDataRx = rosNode->now();
			}

			break;

		case MSG_VSC_NMEA_STRING:
//			handleGpsMsg(&recvMsg);

			break;
		case MSG_USER_FEEDBACK:
//			handleFeedbackMsg(&recvMsg);

			break;
		default:
			errorCounts.invalidRxMsgCount++;
			RCLCPP_ERROR(rosNode->get_logger(), "Receive Error.  Invalid MsgType (0x%02X)",recvMsg.msg.msgType);
			break;
		}
	}

	// Log warning when no data is received
//	ros::Duration noDataDuration = ros::Time::now() - lastDataRx;
    rclcpp::Duration noDataDuration = this->rosNode->now() - lastDataRx;
	if(noDataDuration > rclcpp::Duration::from_seconds(0.25)) {
		RCLCPP_WARN_THROTTLE(
                this->rosNode->get_logger(),
                *this->rosNode->get_clock(),
                500, "No Data Received in %.2f seconds", noDataDuration.seconds()
            );
	}

}

