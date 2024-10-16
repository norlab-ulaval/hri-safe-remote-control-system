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
 * ROS2 Includes
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <VehicleMessages.h>
#include <JoystickHandler.h>

using namespace hri_safety_sense;

JoystickHandler::JoystickHandler(const std::shared_ptr<rclcpp::Node> &node)
{
    this->rosNode = node;
	// Joystick Pub
	rawLeftPub = this->rosNode->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
}

JoystickHandler::~JoystickHandler()
{
}

float JoystickHandler::getStickValue(JoystickType joystick)
{
	int32_t magnitude = (joystick.magnitude<<2) + joystick.mag_lsb;

	float magnitude_f = magnitude / 1023.0; // TODO replace this constant with a parameter
	if(joystick.neutral_status == STATUS_SET) {
		return 0;
	} else if(joystick.negative_status == STATUS_SET) {
		return -1 * magnitude_f;
	} else if(joystick.positive_status == STATUS_SET) {
		return magnitude_f;
	}

	// Error case
	return 0;
}

int32_t JoystickHandler::getButtonValue(uint8_t button)
{
	if(button == STATUS_SET) {
		return 1;
	}

	// Error case
	return 0;
}

uint32_t JoystickHandler::handleNewMsg(const VscMsgType &incomingMsg)
{
	int retval = 0;

	if(incomingMsg.msg.length == sizeof(JoystickMsgType)) {

		JoystickMsgType *joyMsg = (JoystickMsgType*)incomingMsg.msg.data;

		// Broadcast Left Joystick
		sensor_msgs::msg::Joy sendLeftMsg;

		sendLeftMsg.header.stamp = this->rosNode->now();
		sendLeftMsg.header.frame_id = "/srcs"; // TODO parametrize this

		sendLeftMsg.axes.push_back(getStickValue(joyMsg->leftX));
		sendLeftMsg.axes.push_back(getStickValue(joyMsg->leftY));
		sendLeftMsg.axes.push_back(getStickValue(joyMsg->leftZ));

		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.home));
		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.first));
		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.second));
		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->leftSwitch.third));

		sendLeftMsg.axes.push_back(getStickValue(joyMsg->rightX));
		sendLeftMsg.axes.push_back(getStickValue(joyMsg->rightY));
		sendLeftMsg.axes.push_back(getStickValue(joyMsg->rightZ));

		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.home));
		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.first));
		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.second));
		sendLeftMsg.buttons.push_back(getButtonValue(joyMsg->rightSwitch.third));

		rawLeftPub->publish(sendLeftMsg);

	} else {
		retval = -1;

		RCLCPP_WARN(this->rosNode->get_logger(), "RECEIVED PTZ COMMANDS WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
				(unsigned int)sizeof(JoystickMsgType), incomingMsg.msg.length);
	}

	return retval;
}


