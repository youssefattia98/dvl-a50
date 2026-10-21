/**
 * @file   dvl-sensor.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 */

#include "dvl_a50/dvl-sensor.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>

namespace dvl_sensor {

namespace {

rclcpp::Time ros_time_from_unix_seconds(double timestamp_seconds)
{
    const auto seconds = static_cast<int64_t>(std::floor(timestamp_seconds));
    auto nanoseconds = static_cast<int64_t>(
        std::llround((timestamp_seconds - static_cast<double>(seconds)) * 1.0e9));

    if (nanoseconds >= 1000000000LL)
    {
        return rclcpp::Time(seconds + 1, 0, RCL_SYSTEM_TIME);
    }
    return rclcpp::Time(seconds, static_cast<uint32_t>(nanoseconds), RCL_SYSTEM_TIME);
}

rclcpp::Time ros_time_from_unix_microseconds(int64_t timestamp_microseconds)
{
    return rclcpp::Time(timestamp_microseconds * 1000LL, RCL_SYSTEM_TIME);
}

void fill_twist_covariance_from_json(
    const json &json_data,
    std::array<double, 36> &covariance)
{
    covariance.fill(0.0);

    if (!json_data.contains("covariance") || !json_data["covariance"].is_array())
    {
        return;
    }

    const auto &cov = json_data["covariance"];
    if (!cov.empty() && cov[0].is_array())
    {
        const size_t rows = cov.size();
        const size_t cols = cov[0].size();
        if (rows == 3 && cols >= 3)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                if (!cov[i].is_array())
                {
                    continue;
                }
                for (size_t j = 0; j < 3 && j < cov[i].size(); ++j)
                {
                    covariance[i * 6 + j] = cov[i][j].get<double>();
                }
            }
            return;
        }
        if (rows == 6 && cols >= 6)
        {
            for (size_t i = 0; i < 6; ++i)
            {
                if (!cov[i].is_array())
                {
                    continue;
                }
                for (size_t j = 0; j < 6 && j < cov[i].size(); ++j)
                {
                    covariance[i * 6 + j] = cov[i][j].get<double>();
                }
            }
            return;
        }
    }

    if (cov.size() == 9)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                covariance[i * 6 + j] = cov[i * 3 + j].get<double>();
            }
        }
        return;
    }

    const size_t n = std::min<size_t>(cov.size(), covariance.size());
    for (size_t i = 0; i < n; ++i)
    {
        covariance[i] = cov[i].get<double>();
    }
}

}  // namespace


DVL_A50::DVL_A50():
Node("dvl_a50_node")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
            qos_profile);

    timer_receive = this->create_wall_timer(std::chrono::milliseconds(1),std::bind(&DVL_A50::handle_receive, this));

    //Publishers
    dvl_pub_report = this->create_publisher<dvl_msgs::msg::DVL>("dvl/data", qos);
    dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DVLDR>("dvl/position", qos);
    dvl_pub_twist_cov = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/auv/dvl", qos);
    dvl_pub_config_status = this->create_publisher<dvl_msgs::msg::ConfigStatus>("dvl/config/status", qos);
    dvl_pub_command_response = this->create_publisher<dvl_msgs::msg::CommandResponse>("dvl/command/response", qos);
    dvl_sub_config_command = this->create_subscription<dvl_msgs::msg::ConfigCommand>("dvl/config/command", qos, std::bind(&DVL_A50::command_subscriber, this, _1));

    this->declare_parameter<std::string>("dvl_ip_address", "192.168.2.125");
    this->declare_parameter<std::string>("velocity_frame_id", "dvl_A50/velocity_link");
    this->declare_parameter<std::string>("position_frame_id", "dvl_A50/position_link");
    this->declare_parameter<bool>("enable_acoustics_on_startup", true);
    this->declare_parameter<int>("speed_of_sound", 1491);
    
    velocity_frame_id = this->get_parameter("velocity_frame_id").as_string();
    position_frame_id = this->get_parameter("position_frame_id").as_string();
    ip_address = this->get_parameter("dvl_ip_address").as_string();
    RCLCPP_INFO(get_logger(), "IP_ADDRESS: '%s'", ip_address.c_str());

    //--- TCP/IP SOCKET ---- 
    tcpSocket = new TCPSocket((char*)ip_address.c_str() , 16171);
    
    if(tcpSocket->Create() < 0)
        RCLCPP_ERROR(get_logger(), "Error creating the socket");
   
    tcpSocket->SetRcvTimeout(400);
    std::string error;
    
    int error_code = 0;
    //int fault = 1; 
    
    first_time = std::chrono::steady_clock::now();
    first_time_error = first_time;
    while(rclcpp::ok() && fault != 0)
    {
        // Use a short connect timeout so SIGINT can stop the node promptly
        // even when the DVL is powered off or disconnected.
        fault = tcpSocket->Connect(500, error, error_code);
        if(error_code == 114)
        {
            RCLCPP_WARN(get_logger(), "Is the sensor on? error_code: %d", error_code);
            for (int i = 0; i < 20 && rclcpp::ok(); ++i)
                usleep(100000);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
    	    double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
    	    if(dt >= 78.5) //Max time to set up
    	    {
    	        fault = -10;
    	        break;
    	    }
        }
        else if(error_code == 103)
        {
            RCLCPP_WARN(get_logger(), "No route to host, DVL might be booting?: error_code: %d", error_code);
            for (int i = 0; i < 20 && rclcpp::ok(); ++i)
                usleep(100000);
        }
    }  
    
    if (!rclcpp::ok())
    {
        tcpSocket->Close();
        return;
    }
    if(fault == -10)
    {
        tcpSocket->Close();
        RCLCPP_ERROR(get_logger(), "Turn the sensor on and try again!");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "DVL-A50 connected!");

        if (!tcpSocket->SetNonBlocking(true))
        {
            RCLCPP_WARN(get_logger(), "Could not set the DVL TCP socket to non-blocking mode.");
        }

        const bool enable_acoustics =
            this->get_parameter("enable_acoustics_on_startup").as_bool();
        const int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
        if (enable_acoustics || speed_of_sound > 0)
        {
            json parameters;
            if (enable_acoustics)
                parameters["acoustic_enabled"] = true;
            if (speed_of_sound > 0)
                parameters["speed_of_sound"] = speed_of_sound;
            const json command = {{"command", "set_config"}, {"parameters", parameters}};
            this->send_parameter_to_sensor(command);
            RCLCPP_INFO(
                get_logger(),
                "Configured DVL acoustics=%s, speed_of_sound=%d m/s.",
                enable_acoustics ? "enabled" : "unchanged", speed_of_sound);
        }
    }
    
}

DVL_A50::~DVL_A50() {
    tcpSocket->Close();
    delete tcpSocket;
}


void DVL_A50::handle_receive()
{
    if (fault != 0 || !tcpSocket)
    {
        return;
    }

    char temp_buffer[16384];

    // Drain all currently available TCP data in chunks. Reading one byte per
    // recv() cannot keep up with the JSON report stream and creates a growing
    // acquisition-to-publication delay.
    while (rclcpp::ok())
    {
        const int recv_len = tcpSocket->Receive(temp_buffer, sizeof(temp_buffer));
        if (recv_len > 0)
        {
            receive_buffer_.append(temp_buffer, static_cast<size_t>(recv_len));
            continue;
        }

        if (recv_len == 0)
        {
            RCLCPP_WARN(get_logger(), "Socket closed by peer.");
            return;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            break;
        }
        if (errno == EINTR)
        {
            continue;
        }

        RCLCPP_WARN(get_logger(), "Socket receive error: %d", errno);
        return;
    }

    if (!rclcpp::ok())
    {
        return;
    }

    size_t newline = std::string::npos;
    while ((newline = receive_buffer_.find('\n')) != std::string::npos)
    {
        std::string str = receive_buffer_.substr(0, newline);
        receive_buffer_.erase(0, newline + 1);
        if (!str.empty() && str.back() == '\r')
        {
            str.pop_back();
        }

        const size_t json_start = str.find_first_of("{[");
        if (json_start == std::string::npos)
        {
            continue;
        }
        if (json_start > 0)
        {
            str.erase(0, json_start);
        }

        try
        {
            json_data = json::parse(str);

            if (json_data.contains("altitude")) {
                this->publish_vel_trans_report();
            }
            else if (json_data.contains("pitch")) {
                this->publish_dead_reckoning_report();
            }
            else if (json_data.contains("response_to"))
            {
                if(json_data["response_to"] == "set_config"
                || json_data["response_to"] == "calibrate_gyro"
                || json_data["response_to"] == "reset_dead_reckoning")
                    this->publish_command_response();
                else if(json_data["response_to"] == "get_config")
                    this->publish_config_status();
            }
        }
        catch(const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "JSON parse failed: %s", e.what());
        }
    }

    if (receive_buffer_.size() > 1024 * 1024)
    {
        RCLCPP_WARN(get_logger(), "Discarding oversized incomplete DVL TCP report.");
        receive_buffer_.clear();
    }
}

/*
 * Publish velocity and transductors report
 */
void DVL_A50::publish_vel_trans_report()
{
    // DVL message struct
    dvl_msgs::msg::DVLBeam beam0;
    dvl_msgs::msg::DVLBeam beam1;
    dvl_msgs::msg::DVLBeam beam2;
    dvl_msgs::msg::DVLBeam beam3;
    dvl_msgs::msg::DVL dvl;

    const auto time_of_validity = json_data["time_of_validity"].get<int64_t>();
    dvl.header.stamp = time_of_validity > 0
        ? ros_time_from_unix_microseconds(time_of_validity)
        : Node::now();
    dvl.header.frame_id = velocity_frame_id;
		
    dvl.time = double(json_data["time"]);
    dvl.time_of_validity = time_of_validity;
    dvl.time_of_transmission = json_data["time_of_transmission"].get<int64_t>();

    dvl.velocity.x = double(json_data["vx"]);
    dvl.velocity.y = double(json_data["vy"]);
    dvl.velocity.z = double(json_data["vz"]);
    dvl.fom = double(json_data["fom"]);
    double current_altitude = double(json_data["altitude"]);
    const bool vel_ok = json_data.value("velocity_valid", false);
    dvl.velocity_valid = vel_ok;
    
    // Removed logic to add old altitude if not valid. Does not report accurately what is coming from the DVL.
    // Logic should be implemented on a case by case basis.
    dvl.altitude = current_altitude;


    dvl.status = json_data["status"];
    dvl.form = json_data["format"];

    // Add covariance from message
    std::vector<double> twistCovariance;

    if (json_data.contains("covariance") && json_data["covariance"].is_array()) {
        const auto& matrix = json_data["covariance"];
        
        for (const auto& row : matrix) {
            if (!row.is_array()) continue;
            for (const auto& value : row) {
                twistCovariance.push_back(value.get<double>());
            }
        }
    }

    dvl.covariance = twistCovariance;
			
    beam0.id = json_data["transducers"][0]["id"];
    beam0.velocity = double(json_data["transducers"][0]["velocity"]);
    beam0.distance = double(json_data["transducers"][0]["distance"]);
    beam0.rssi = double(json_data["transducers"][0]["rssi"]);
    beam0.nsd = double(json_data["transducers"][0]["nsd"]);
    beam0.valid = json_data["transducers"][0]["beam_valid"];
			
    beam1.id = json_data["transducers"][1]["id"];
    beam1.velocity = double(json_data["transducers"][1]["velocity"]);
    beam1.distance = double(json_data["transducers"][1]["distance"]);
    beam1.rssi = double(json_data["transducers"][1]["rssi"]);
    beam1.nsd = double(json_data["transducers"][1]["nsd"]);
    beam1.valid = json_data["transducers"][1]["beam_valid"];
			
    beam2.id = json_data["transducers"][2]["id"];
    beam2.velocity = double(json_data["transducers"][2]["velocity"]);
    beam2.distance = double(json_data["transducers"][2]["distance"]);
    beam2.rssi = double(json_data["transducers"][2]["rssi"]);
    beam2.nsd = double(json_data["transducers"][2]["nsd"]);
    beam2.valid = json_data["transducers"][2]["beam_valid"];
			
    beam3.id = json_data["transducers"][3]["id"];
    beam3.velocity = double(json_data["transducers"][3]["velocity"]);
    beam3.distance = double(json_data["transducers"][3]["distance"]);
    beam3.rssi = double(json_data["transducers"][3]["rssi"]);
    beam3.nsd = double(json_data["transducers"][3]["nsd"]);
    beam3.valid = json_data["transducers"][3]["beam_valid"];
		    
    dvl.beams = {beam0, beam1, beam2, beam3};
    dvl_pub_report->publish(dvl);

    // Keep the raw report available for timestamped altitude/beam ranges even
    // when bottom-lock velocity is invalid. Only the fusion velocity topic is
    // suppressed in that case.
    if (!vel_ok) {
        return;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped dvl_twist_cov;
    dvl_twist_cov.header = dvl.header;
    // Water Linked DVL velocity is FRD: x-forward, y-right, z-down.
    // Publish /auv/dvl in ROS FLU: x-forward, y-left, z-up.
    dvl_twist_cov.twist.twist.linear.x = dvl.velocity.x;
    dvl_twist_cov.twist.twist.linear.y = -dvl.velocity.y;
    dvl_twist_cov.twist.twist.linear.z = -dvl.velocity.z;
    fill_twist_covariance_from_json(json_data, dvl_twist_cov.twist.covariance);
    dvl_pub_twist_cov->publish(dvl_twist_cov);
}

/*
 * Publish dead reckoning
 */
void DVL_A50::publish_dead_reckoning_report()
{
    dvl_msgs::msg::DVLDR DVLDeadReckoning;
    //std::cout << std::setw(4) << json_data << std::endl;
    const double measurement_time = double(json_data["ts"]);
    DVLDeadReckoning.header.stamp = measurement_time > 0.0
        ? ros_time_from_unix_seconds(measurement_time)
        : Node::now();
    DVLDeadReckoning.header.frame_id = position_frame_id;
    DVLDeadReckoning.time = measurement_time;
    DVLDeadReckoning.position.x = double(json_data["x"]);
    DVLDeadReckoning.position.y = double(json_data["y"]);
    DVLDeadReckoning.position.z = double(json_data["z"]);
    DVLDeadReckoning.pos_std = double(json_data["std"]);
    DVLDeadReckoning.roll = double(json_data["roll"]);
    DVLDeadReckoning.pitch = double(json_data["pitch"]);
    DVLDeadReckoning.yaw = double(json_data["yaw"]);
    DVLDeadReckoning.type = json_data["type"];
    DVLDeadReckoning.status = json_data["status"];
    DVLDeadReckoning.format = json_data["format"];
    dvl_pub_pos->publish(DVLDeadReckoning);
}

/*
 * Publish the command response
 */
void DVL_A50::publish_command_response()
{
    dvl_msgs::msg::CommandResponse command_resp;
    command_resp.response_to = json_data["response_to"];
    command_resp.success = json_data["success"];
    command_resp.error_message = json_data["error_message"];
    command_resp.result = 0;
    command_resp.format = json_data["format"];
    command_resp.type = json_data["type"];
    dvl_pub_command_response->publish(command_resp);
}

/*
 * Publish the sensor current configuration
 */
void DVL_A50::publish_config_status()
{
    dvl_msgs::msg::ConfigStatus status_msg;
    status_msg.response_to = json_data["response_to"];
    status_msg.success = json_data["success"];
    status_msg.error_message = json_data["error_message"];
    status_msg.speed_of_sound = json_data["result"]["speed_of_sound"];
    status_msg.acoustic_enabled = json_data["result"]["acoustic_enabled"];
    status_msg.dark_mode_enabled = json_data["result"]["dark_mode_enabled"];
    status_msg.mounting_rotation_offset = json_data["result"]["mounting_rotation_offset"];
    status_msg.range_mode = json_data["result"]["range_mode"];
    status_msg.format = json_data["format"];
    status_msg.type = json_data["type"];
    dvl_pub_config_status->publish(status_msg);
}


void DVL_A50::command_subscriber(const dvl_msgs::msg::ConfigCommand::SharedPtr msg)
{
    if(msg->command == "set_config")
        this->set_json_parameter(msg->parameter_name, msg->parameter_value);
    else if(msg->command == "get_config")
    {
        json command = {
            {"command", "get_config"}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "calibrate_gyro")
    {
        json command = {
            {"command", "calibrate_gyro"}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "reset_dead_reckoning")
    {
        json command = {
            {"command", "reset_dead_reckoning"}
        };
        this->send_parameter_to_sensor(command);
    }

}

DVL_Parameters DVL_A50::resolveParameter(std::string param)
{
	if(param == "speed_of_sound")
	    return speed_of_sound;
	else if(param == "acoustic_enabled")
	    return acoustic_enabled;
	else if(param == "dark_mode_enabled")
	    return dark_mode_enabled;
	else if(param == "mountig_rotation_offset")
	    return mountig_rotation_offset;
	else if(param == "range_mode")
	    return range_mode;

	return invalid_param;
}

/*
 * Create JSON command message to set parameter in the DVL sensor
 */
void DVL_A50::set_json_parameter(const std::string name, const std::string value)
{
    json message;
    message["command"] = "set_config";

    switch (resolveParameter(name))
    {
        case speed_of_sound:
            try
            {
                message["parameters"]["speed_of_sound"] = (int)std::stoi(value);
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case acoustic_enabled:
            try
            {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["acoustic_enabled"] = data;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case dark_mode_enabled:
            try
            {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["dark_mode_enabled"] = data;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case mountig_rotation_offset:
            try
            {
                message["parameters"]["mountig_rotation_offset"] = (double)std::stod(value);
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case range_mode:
            try
            {
                message["parameters"]["range_mode"] = value;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Invalid parameter!");
            break;
    }

}

/*
 * Send parameter to the sensor using the JSON command
 */
void DVL_A50::send_parameter_to_sensor(const json &message)
{
    std::string str = message.dump();
    str.push_back('\n');
    char* c = &*str.begin();
    tcpSocket->Send(c);
}

}//end namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dvl_sensor::DVL_A50>();
    if (!rclcpp::ok()) {
        return 0;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
