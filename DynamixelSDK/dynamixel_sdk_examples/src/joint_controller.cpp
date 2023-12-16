#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

#include "dynamixel_sdk_custom_interfaces/msg/joints_position.hpp" // NEWLY ADDED BY XIN ZHI!!! :)


#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_joints_position.hpp" // NEWLY ADDED BY XIN ZHI!!! :)

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "joint_controller.hpp"

// Control table address 
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36
#define ADDR_SET_SPEED 32


// Protocol version
#define PROTOCOL_VERSION 1.0  

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL A series

#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint16_t goal_position = 0;
const uint16_t joint_speed = 30; // set motor speed limit

int dxl_comm_result = COMM_TX_FAIL;



JointController::JointController()
: Node("joint_controller")
{
  RCLCPP_INFO(this->get_logger(), "Run joint controller");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  joints_position_subscriber_ =
    this->create_subscription<JointsPosition>(
    "joints_position",
    QOS_RKL10V,
    [this](const JointsPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // int index = 0;

      for (int index = 0; index < 6; index++){
        if (index == 4) {
          ;
        }
        else{

        
        // Position Value of X series is 4 byte data.
        // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
          uint32_t goal_position = (unsigned int)msg->position[index];  // Convert int32 -> uint32

          dxl_comm_result =
          packetHandler->write2ByteTxRx(
            portHandler,
            (uint8_t) msg->id[index],
            ADDR_SET_SPEED,
            joint_speed,
            &dxl_error
          );

          if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
          } else if (dxl_error != 0) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
          } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [joint speed: %d]", msg->id[index], joint_speed);
          }        
          ;        

          // Write Goal Position (length : 4 bytes)
          // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
          dxl_comm_result =
          packetHandler->write2ByteTxRx(
            portHandler,
            (uint8_t) msg->id[index],
            ADDR_GOAL_POSITION,
            goal_position,
            &dxl_error
          );


          if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
          } else if (dxl_error != 0) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
          } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id[index], msg->position[index]);
          }        
          ;
      }
      }

    }
    ); // end of subscriber




  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };
  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);

  auto get_present_joints =
    [this](
    const std::shared_ptr<GetJointsPosition::Request> request,
    std::shared_ptr<GetJointsPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      std::vector<int32_t> present_joints_positions(6);
      std::vector<int8_t> indexs(6);


      for (uint8_t index = 0; index < 6; index++){
        if (request->get){;}
        dxl_comm_result = packetHandler->read4ByteTxRx(
          portHandler,
          index,
          ADDR_PRESENT_POSITION,
          reinterpret_cast<uint32_t *>(&present_position),
          &dxl_error
        );

        RCLCPP_INFO(
          this->get_logger(),
          "Get [ID: %d] [Present Position: %d]",
          index,
          present_position
        );
        present_joints_positions[index] = present_position;
        indexs[index] = index;
      }
      
      response->position = present_joints_positions;
      response->id = indexs;
    };

  get_joints_position_server_ = create_service<GetJointsPosition>("get_joints_position", get_present_joints);
}  // end of joint controller definition



JointController::~JointController()
{
}

void setupDynamixel(uint8_t dxl_id)
{

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("joint_controller"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("joint_controller"), "Succeeded to enable torque.");
  }
}


int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("joint_controller"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("joint_controller"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("joint_controller"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("joint_controller"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto jointcontroller = std::make_shared<JointController>();
  rclcpp::spin(jointcontroller);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
