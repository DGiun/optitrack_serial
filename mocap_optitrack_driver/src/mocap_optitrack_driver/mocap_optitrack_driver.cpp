// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>

#include <string>
#include <vector>
#include <memory>

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

#include "mocap_optitrack_driver/mocap_optitrack_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

boost::asio::io_service io;
boost::asio::serial_port ser(io);

namespace mocap_optitrack_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

OptitrackDriverNode::OptitrackDriverNode()
: ControlledLifecycleNode("mocap_optitrack_driver_node")
{
  // Set Parameter
  declare_parameter<std::string>("connection_type", "Unicast");
  declare_parameter<std::string>("server_address", "000.000.000.000");
  declare_parameter<std::string>("local_address", "000.000.000.000");
  declare_parameter<std::string>("multicast_address", "000.000.000.000");
  declare_parameter<uint16_t>("server_command_port", 0);
  declare_parameter<uint16_t>("server_data_port", 0);
  declare_parameter<bool>("marker_pub", true);
  declare_parameter<bool>("rigid_pub", true);

  // Serial Port
  declare_parameter<bool>("serial", false);
  declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  declare_parameter<int>("serial_buad", 57600);

  client = new NatNetClient();
  client->SetFrameReceivedCallback(process_frame_callback, this);
}

OptitrackDriverNode::~OptitrackDriverNode()
{
  if(ser.is_open())  ser.close();
}

void OptitrackDriverNode::set_settings_optitrack()
{
  if (connection_type_ == "Multicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Multicast;
    client_params.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown connection type -- options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  client_params.serverAddress = server_address_.c_str();
  client_params.localAddress = local_address_.c_str();
  client_params.serverCommandPort = server_command_port_;
  client_params.serverDataPort = server_data_port_;
}

bool OptitrackDriverNode::stop_optitrack()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");

  return true;
}

void
OptitrackDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
  trigger_transition(
    rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

void
OptitrackDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
  trigger_transition(
    rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData)
{
  static_cast<OptitrackDriverNode *>(pUserData)->process_frame(data);
}

std::chrono::nanoseconds OptitrackDriverNode::get_optitrack_system_latency(sFrameOfMocapData * data)
{
  const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

  if (bSystemLatencyAvailable) {
    const double clientLatencySec =
      client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
    const double clientLatencyMillisec = clientLatencySec * 1000.0;
    const double transitLatencyMillisec =
      client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    const double largeLatencyThreshold = 100.0;
    if (clientLatencyMillisec >= largeLatencyThreshold) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 500,
        "Optitrack system latency >%.0f ms: [Transmission: %.0fms, Total: %.0fms]",
        largeLatencyThreshold, transitLatencyMillisec, clientLatencyMillisec);
    }

    return round<std::chrono::nanoseconds>(std::chrono::duration<float>{clientLatencySec});
  } else {
    RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
    return std::chrono::nanoseconds::zero();
  }
}

void
OptitrackDriverNode::process_frame(sFrameOfMocapData * data)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  frame_number_++;
  rclcpp::Duration frame_delay = rclcpp::Duration(get_optitrack_system_latency(data));

  std::map<int, std::vector<mocap4r2_msgs::msg::Marker>> marker2rb;

  // Markers
  if (mocap_markers_pub_->get_subscription_count() > 0 && f_marker_pub_) {
    mocap4r2_msgs::msg::Markers msg;
    msg.header.stamp = now() - frame_delay;
    msg.header.frame_id = "map";
    msg.frame_number = frame_number_;

    for (int i = 0; i < data->nLabeledMarkers; i++) {
      bool Unlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bool ActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);
      sMarker & marker_data = data->LabeledMarkers[i];
      int modelID, markerID;
      NatNet_DecodeID(marker_data.ID, &modelID, &markerID);

      mocap4r2_msgs::msg::Marker marker;
      marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = marker_data.x;
      marker.translation.y = marker_data.y;
      marker.translation.z = marker_data.z;
      if (ActiveMarker || Unlabeled) {
        msg.markers.push_back(marker);
      } else {
        marker2rb[modelID].push_back(marker);
      }
    }
    mocap_markers_pub_->publish(msg);
  }

  if (mocap_rigid_body_pub_->get_subscription_count() > 0 && f_rigid_pub_) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.stamp = now() - frame_delay;
    // msg_rb.header.stamp = now();
    msg_rb.header.frame_id = "map";
    msg_rb.frame_number = frame_number_;

    for (int i = 0; i < data->nRigidBodies; i++) {
      mocap4r2_msgs::msg::RigidBody rb;

      rb.rigid_body_name = std::to_string(data->RigidBodies[i].ID);
      rb.pose.position.x = data->RigidBodies[i].x;
      rb.pose.position.y = data->RigidBodies[i].y;
      rb.pose.position.z = data->RigidBodies[i].z;
      rb.pose.orientation.x = data->RigidBodies[i].qx;
      rb.pose.orientation.y = data->RigidBodies[i].qy;
      rb.pose.orientation.z = data->RigidBodies[i].qz;
      rb.pose.orientation.w = data->RigidBodies[i].qw;
      rb.markers = marker2rb[data->RigidBodies[i].ID];

      msg_rb.rigidbodies.push_back(rb);
    }

    mocap_rigid_body_pub_->publish(msg_rb);
  }

  // Serial Port
  if (mocap_rigid_body_pub_->get_subscription_count() > 0 && f_serial_){
    // uint16_t head = 0xfb01; // 0xFB: Header, 0x01: Pose
    uint8_t idx=0;
    uint8_t temp_c;
    float temp_f;

    // Header - 2 byte
    // memcpy(packet,&head,sizeof(uint16_t));
    // idx += sizeof(uint16_t);

    uint8_t head=0xfb;
    memcpy(packet,&head,sizeof(uint8_t));
    idx += sizeof(uint8_t);
    uint8_t ID=0x01;
    memcpy(packet+idx,&ID,sizeof(uint8_t));
    idx += sizeof(uint8_t);
    
    // Data Size - 1 byte , Data Size = 3 +(number of rigid * 25) +1
    temp_c = static_cast<uint8_t>( 4 +(data->nRigidBodies*25) );
    memcpy(packet+idx,&temp_c,sizeof(uint8_t));
    idx += sizeof(uint8_t);
    
    // Payload - number of rigid * 25 byte
    for(size_t i=0; i<static_cast<uint8_t>(data->nRigidBodies); ++i){
      // ID - 1 byte
      temp_c = static_cast<uint8_t>(data->RigidBodies[i].ID);
      memcpy(packet+idx,&temp_c,sizeof(uint8_t));
      idx += sizeof(uint8_t);

      // Position - 12 byte
      memcpy(packet+idx,&(data->RigidBodies[i].x),sizeof(float));
      idx += sizeof(float);
      memcpy(packet+idx,&(data->RigidBodies[i].y),sizeof(float));
      idx += sizeof(float);
      memcpy(packet+idx,&(data->RigidBodies[i].z),sizeof(float));
      idx += sizeof(float);

      // Quaternion(Vector) - 12 byte
      if(data->RigidBodies[i].qw >= 0){
        memcpy(packet + idx, &(data->RigidBodies[i].qx), sizeof(float));
        idx += sizeof(float);
        memcpy(packet + idx, &(data->RigidBodies[i].qy), sizeof(float));
        idx += sizeof(float);
        memcpy(packet + idx, &(data->RigidBodies[i].qz), sizeof(float));
        idx += sizeof(float);
      }
      else{
        temp_f = -1*data->RigidBodies[i].qx;
        memcpy(packet + idx, &temp_f, sizeof(float));
        idx += sizeof(float);
        temp_f = -1*data->RigidBodies[i].qy;
        memcpy(packet + idx, &temp_f, sizeof(float));
        idx += sizeof(float);
        temp_f = -1*data->RigidBodies[i].qz;
        memcpy(packet + idx, &temp_f, sizeof(float));
        idx += sizeof(float);
      }
    }

    // CRC - 1 byte
    temp_c = 0x00;
    for(size_t i=0; i<idx-1; ++i){
      temp_c ^= *(packet + i);
    }
    memcpy(packet + idx, &temp_c, sizeof(uint8_t));
    idx += sizeof(uint8_t);

    // Debug
    // for(size_t i=0; i<idx; ++i){
    //   std::cout <<std::to_string(packet[i])<<' ';
    // }
    // std::cout<<std::endl;

    ser.write_some(boost::asio::buffer(&packet, idx));
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  initParameters();

  mocap_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap_rigid_body_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));

  connect_optitrack();

  // Open Serial Port
  if (f_serial_)  {
    ser.open(port_);
    ser.set_option(boost::asio::serial_port_base::baud_rate(serial_buad_));
    if (ser.is_open())    {
      RCLCPP_INFO(get_logger(), "Serial Port initialized\n");
      std::cout<<"Port: "<<port_<<std::endl;
      std::cout<<"Buad: "<<serial_buad_<<std::endl;
    }
    else    {
      RCLCPP_INFO(get_logger(), "Fail Serial Port initialized\n");
    }
  }

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap_markers_pub_->on_activate();
  mocap_rigid_body_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap_markers_pub_->on_deactivate();
  mocap_rigid_body_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_cleanup(state);
  } else {
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_shutdown(state);
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  disconnect_optitrack();

  return ControlledLifecycleNode::on_error(state);
}

bool
OptitrackDriverNode::connect_optitrack()
{
  RCLCPP_INFO(
    get_logger(),
    "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

  client->Disconnect();
  set_settings_optitrack();

  if (client->Connect(client_params) == ErrorCode::ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "... connected!");

    memset(&server_description, 0, sizeof(server_description));
    client->GetServerDescription(&server_description);
    if (!server_description.HostPresent) {
      RCLCPP_DEBUG(get_logger(), "Unable to connect to server. Host not present.");
      return false;
    }

    if (client->GetDataDescriptionList(&data_descriptions) != ErrorCode_OK || !data_descriptions) {
      RCLCPP_DEBUG(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
    }

    RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
    RCLCPP_INFO(
      get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
      server_description.szHostApp, server_description.HostAppVersion[0],
      server_description.HostAppVersion[1], server_description.HostAppVersion[2],
      server_description.HostAppVersion[3]);
    RCLCPP_INFO(
      get_logger(), "NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0],
      server_description.NatNetVersion[1],
      server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
    RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params.localAddress);
    RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params.serverAddress);
    RCLCPP_INFO(get_logger(), "Server Name:%s\n", server_description.szHostComputerName);

    void * pResult;
    int nBytes = 0;

    if (client->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK) {
      float fRate = *(static_cast<float *>(pResult));
      RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", fRate);
    } else {
      RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
    }
  } else {
    RCLCPP_INFO(get_logger(), "... not connected :( ");
    return false;
  }

  return true;
}

bool
OptitrackDriverNode::disconnect_optitrack()
{
  void * response;
  int nBytes;
  if (client->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK) {
    client->Disconnect();
    RCLCPP_INFO(get_logger(), "[Client] Disconnected");
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
    return false;
  }
}

void
OptitrackDriverNode::initParameters()
{
  get_parameter<std::string>("connection_type", connection_type_);
  get_parameter<std::string>("server_address", server_address_);
  get_parameter<std::string>("local_address", local_address_);
  get_parameter<std::string>("multicast_address", multicast_address_);
  get_parameter<uint16_t>("server_command_port", server_command_port_);
  get_parameter<uint16_t>("server_data_port", server_data_port_);
  get_parameter<bool>("marker_pub", f_marker_pub_);
  get_parameter<bool>("rigid_pub", f_rigid_pub_);

  // Serial Port
  get_parameter<bool>("serial", f_serial_);
  get_parameter<std::string>("serial_port", port_);
  get_parameter<int>("serial_buad", serial_buad_);
}

}  // namespace mocap_optitrack_driver
