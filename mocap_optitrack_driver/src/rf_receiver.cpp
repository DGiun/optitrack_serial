#include <opti_serial/rf_receiver.hpp>

// rclcpp::Clock steady_clock(RCL_STEADY_TIME);
// rclcpp::Clock ros_clock(RCL_ROS_TIME);
boost::asio::io_service io;
boost::asio::serial_port ser(io);

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<opti_serial_pub>());
  rclcpp::shutdown();
  return 0;
}

opti_serial_pub::opti_serial_pub()
: Node("optitrack_serial"), count_(0), ios(),port(ios)
{
    this->declare_parameter<uint8_t>("rigid_id", 1);
    this->declare_parameter<string>("topic_name", "/mavros/vision_pose/pose");
    this->declare_parameter<bool>("ridids_pub", false);
    this->declare_parameter<string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_buad", 57600);

    this->get_parameter<uint8_t>("rigid_id", my_id);
    this->get_parameter<std::string>("topic_name", topic_name);
    this->get_parameter<bool>("ridids_pub", f_poses);
    this->get_parameter<std::string>("serial_port", serial_port);
    this->get_parameter<int>("serial_buad", serial_buad);
    
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
    poses_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/opti/poses", 10);

    port.open(serial_port);
    port.set_option(boost::asio::serial_port_base::baud_rate(serial_buad));
    if (port.is_open()){
      RCLCPP_INFO(get_logger(), "Serial Port Open");
      std::cout<<"Serial Port:"<<serial_port<<std::endl;
      std::cout<<"Serial Buad:"<<serial_buad<<std::endl;
    }
    else RCLCPP_INFO(get_logger(), "Serial Port Close");

    timer_ = this->create_wall_timer(
        5ms, std::bind(&opti_serial_pub::timer_callback, this));
}

opti_serial_pub::~opti_serial_pub()
{
    if(port.is_open())  port.close();
}

void opti_serial_pub::timer_callback()
{
    uint n = port.read_some(boost::asio::buffer(buffer, 512));
    if (n != 0){
        for (size_t i=0; i<n; ++i){
            EnQueue(buffer[i]);
        }
    }
}

void opti_serial_pub::EnQueue(uint8_t data){
  //std::cout << data << std::end;
  packet_buf.push_back(data);
  uint idx = packet_buf.size();
  //RCLCPP_INFO(get_logger(), "data: %x %d", data, idx);
  
  switch (idx)
  {
    case 1: // Header byte
    	if(!HEADER(packet_buf.back())){
            packet_id = packet_size = 0;
            packet_buf.clear();
        }
    	break;

    case 2: // Packet ID
    	if(!HEAD_ID(packet_buf.back())){
            packet_id = packet_size = 0;
            packet_buf.clear();
        }
        else{
            packet_id = static_cast<uint8_t>(packet_buf.back());
        }
    	break;

    case 3: // Data Size
    	packet_size = static_cast<uint8_t>(packet_buf.back());
    	break;
  }

  // Decode
  if (idx >= packet_size && packet_size != 0)
  {
    if(POSE_ID(packet_id)){
      Decode_poses(packet_buf);
    }

    packet_buf.clear();
  }
}

void opti_serial_pub::Decode_poses(vector<uint8_t> packet){
    uint8_t *data = &packet[3]; // pointer of Payload
    uint8_t idx = 0;
    uint8_t N_rigids = (packet[2]-4)/25;
    uint8_t rigid_ID = 0;
    geometry_msgs::msg::PoseArray multi_rigid;
    geometry_msgs::msg::Pose rigid;
    uint8_t temp_c = 0;
    float temp_f = 0.0f;

    // CRC check
    temp_c = 0;
    for(size_t i=0; i<packet.size()-2; ++i){
      temp_c ^= packet[i];
    }
    if(temp_c != packet.back()){
    RCLCPP_INFO(get_logger(), "CRC_fail: %x %x",temp_c, packet.back());
    return;
    }

    // Data Decode
    for(size_t i=0; i<N_rigids; ++i){
        // Rigid_ID
        rigid_ID = *data+idx;
        if(multi_rigid.poses.size() < rigid_ID)
          multi_rigid.poses.resize(rigid_ID);

        // Position
        rigid.position.x = *(float *)(data + idx);
        idx += sizeof(float);
        rigid.position.y = *(float *)(data + idx);
        idx += sizeof(float);
        rigid.position.z = *(float *)(data + idx);
        idx += sizeof(float);

        // Quaternion
        temp_f = 0.0f;
        rigid.orientation.x = *(float *)(data + idx);
        temp_f += pow( *(float *)(data + idx), 2 );
        idx += sizeof(float);
        rigid.orientation.y = *(float *)(data + idx);
        temp_f += pow( *(float *)(data + idx), 2 );
        idx += sizeof(float);
        rigid.orientation.z = *(float *)(data + idx);
        temp_f += pow( *(float *)(data + idx), 2 );
        idx += sizeof(float);
        rigid.orientation.w = sqrt(1.0 - temp_f);

        multi_rigid.poses[rigid_ID-1] = rigid;
    }

    geometry_msgs::msg::PoseStamped my_rigid;
    my_rigid.header.stamp = this->get_clock()->now();
    my_rigid.header.frame_id = "map";
    my_rigid.pose = multi_rigid.poses[my_id-1];
    pose_pub->publish(my_rigid);

    if(f_poses){
      multi_rigid.header.stamp = this->get_clock()->now();
      multi_rigid.header.frame_id = "map";
      poses_pub->publish(multi_rigid);
    }
}
