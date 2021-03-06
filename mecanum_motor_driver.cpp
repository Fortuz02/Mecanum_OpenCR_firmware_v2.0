/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "mecanum_motor_driver.h"

MecanumMotorDriver::MecanumMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_rear_wheel_id_(DXL_LEFT_REAR_ID), right_rear_wheel_id_(DXL_RIGHT_REAR_ID),
  left_front_wheel_id_(DXL_LEFT_FRONT_ID), right_front_wheel_id_(DXL_RIGHT_FRONT_ID)
{
}

MecanumMotorDriver::~MecanumMotorDriver()
{
  closeDynamixel();
}

bool MecanumMotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setTorque(left_rear_wheel_id_, true);
  setTorque(right_rear_wheel_id_, true);
  setTorque(left_front_wheel_id_, true);
  setTorque(right_front_wheel_id_, true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);

  return true;
}

bool MecanumMotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void MecanumMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_rear_wheel_id_, false);
  setTorque(right_rear_wheel_id_, false);
  setTorque(left_front_wheel_id_, false);
  setTorque(right_front_wheel_id_, false);

  // Close port
  portHandler_->closePort();
}

bool MecanumMotorDriver::controlMotor(int64_t left_rear_wheel_value, int64_t right_rear_wheel_value, int64_t left_front_wheel_value, int64_t right_front_wheel_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(left_rear_wheel_id_, (uint8_t*)&left_rear_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(right_rear_wheel_id_, (uint8_t*)&right_rear_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(left_front_wheel_id_, (uint8_t*)&left_front_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(right_front_wheel_id_, (uint8_t*)&right_front_wheel_value);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}
