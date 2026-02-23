/**
 * @file   lifecycle_dvl_component.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 *
 * Contact: pgutierrez@marum.de
 */

#include "dvl_a50/lifecycle_dvl_component.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;
using nlohmann::json;

namespace composition
{

namespace {

void fill_twist_covariance_from_json(
    const nlohmann::json & json_data,
    std::array<double, 36> & covariance)
{
    covariance.fill(0.0);

    if (!json_data.contains("covariance") || !json_data["covariance"].is_array()) {
        return;
    }

    const auto & cov = json_data["covariance"];
    if (!cov.empty() && cov[0].is_array()) {
        const size_t rows = cov.size();
        const size_t cols = cov[0].size();
        if (rows == 3 && cols >= 3) {
            for (size_t i = 0; i < 3; ++i) {
                if (!cov[i].is_array()) {
                    continue;
                }
                for (size_t j = 0; j < 3 && j < cov[i].size(); ++j) {
                    covariance[i * 6 + j] = cov[i][j].get<double>();
                }
            }
            return;
        }
        if (rows == 6 && cols >= 6) {
            for (size_t i = 0; i < 6; ++i) {
                if (!cov[i].is_array()) {
                    continue;
                }
                for (size_t j = 0; j < 6 && j < cov[i].size(); ++j) {
                    covariance[i * 6 + j] = cov[i][j].get<double>();
                }
            }
            return;
        }
    }

    if (cov.size() == 9) {
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                covariance[i * 6 + j] = cov[i * 3 + j].get<double>();
            }
        }
        return;
    }

    const size_t n = std::min<size_t>(cov.size(), covariance.size());
    for (size_t i = 0; i < n; ++i) {
        covariance[i] = cov[i].get<double>();
    }
}

}  // namespace



LifecycleDVL::LifecycleDVL(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("dvl_a50_node", options),
current_altitude(0.0),
old_altitude(0.0)
{
    this->declare_parameter<std::string>("dvl_ip_address", "192.168.2.95");   
    ip_address = this->get_parameter("dvl_ip_address").as_string();
    RCLCPP_INFO(get_logger(), "IP_ADDRESS: '%s'", ip_address.c_str());
}

LifecycleDVL::~LifecycleDVL() {
    if (tcpSocket != nullptr) {
        tcpSocket->Close();
        delete tcpSocket;
        tcpSocket = nullptr;
    }
}

/// Transition callback for state configuring
/// TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
/// TRANSITION_CALLBACK_FAILURE transitions to "inactive"
/// TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&LifecycleDVL::on_timer, this));
    dvl_pub_report = this->create_publisher<dvl_msgs::msg::DVL>("dvl/data", 10);
    dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DVLDR>("dvl/position", 10);
    dvl_pub_twist_cov = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/auv/dvl", 10);
    

    //--- TCP/IP SOCKET ---- 
    tcpSocket = new TCPSocket((char*)ip_address.c_str() , 16171);
    
    if(tcpSocket->Create() < 0)
    {
        RCLCPP_INFO(get_logger(), "Socket creation error");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
   
    tcpSocket->SetRcvTimeout(300);
    std::string error;
    
    int error_code = 0;
    //int fault = 1; 
    
    first_time_error = std::chrono::steady_clock::now();
    while(rclcpp::ok() && fault != 0)
    {
        fault = tcpSocket->Connect(5000, error, error_code);
        if(error_code == 114)
        {
            RCLCPP_INFO(get_logger(), "TCP Error: [%s]", error.c_str());
            RCLCPP_INFO(get_logger(), "Is the sensor on?");
            //usleep(2000000);
            std::this_thread::sleep_for(2s);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
    	    double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
    	    if(dt >= 78.5) //Max time to set up
    	    {
    	        RCLCPP_INFO(get_logger(), "Error time: [%6.2f]", dt);
    	        fault = -10;
    	        break;
    	    }
        }
        else if(error_code == 103)
        {
            RCLCPP_INFO(get_logger(), "TCP Error: [%s]", error.c_str());
            RCLCPP_INFO(get_logger(), "No route to host, DVL might be booting?");
            //usleep(2000000);
            std::this_thread::sleep_for(2s);
        }
    }  

    if (!rclcpp::ok()) {
        RCLCPP_INFO(get_logger(), "Shutdown requested while connecting to DVL.");
        tcpSocket->Close();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    
    //std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    //double dt = std::chrono::duration<double>(current_time - first_time).count();
    //first_time = current_time;
    //std::cout << "time: " << dt << std::endl;
    
    if(fault == -10)
    {
        tcpSocket->Close();
        RCLCPP_INFO(get_logger(), "Turn the sensor on and try again!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    else
        RCLCPP_INFO(get_logger(), "DVL-A50 connected!");

    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_activate(const rclcpp_lifecycle::State &)
{
    dvl_pub_report->on_activate();
    dvl_pub_pos->on_activate();
    dvl_pub_twist_cov->on_activate();
    first_time_loss = std::chrono::steady_clock::now();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state deactivating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_deactivate(const rclcpp_lifecycle::State &)
{

    dvl_pub_report->on_deactivate();
    dvl_pub_pos->on_deactivate();
    dvl_pub_twist_cov->on_deactivate();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleDVL::on_cleanup(const rclcpp_lifecycle::State &)
{
    fault = 1;
    if (tcpSocket != nullptr) {
        tcpSocket->Close();
    }
 
    timer_.reset();
    dvl_pub_report.reset();
    dvl_pub_pos.reset();
    dvl_pub_twist_cov.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;  
}

/// Transition callback for state shutting down
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleDVL::on_shutdown(const rclcpp_lifecycle::State & state)
{
    fault = 1;
    if (tcpSocket != nullptr) {
        tcpSocket->Close();
    }

    timer_.reset();
    dvl_pub_report.reset();
    dvl_pub_pos.reset();
    dvl_pub_twist_cov.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());
      
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}



void LifecycleDVL::on_timer()
{

  if (!rclcpp::ok()) {
      return;
  }

  if (!dvl_pub_report || !dvl_pub_pos || !dvl_pub_twist_cov || !tcpSocket) {
      return;
  }

  if(!dvl_pub_report->is_activated() || !dvl_pub_pos->is_activated())
  {
      //RCLCPP_INFO(get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
  }else {
        std::flush(std::cout);
        char temp_buffer = '\0';

        //tcpSocket->Receive(&tempBuffer[0]);
        std::string str; 
    
        //std::chrono::steady_clock::time_point current_time;
	while(rclcpp::ok())
	{
	    //current_time = std::chrono::steady_clock::now();
	    //double dt = std::chrono::duration<double>(current_time - first_time_loss).count();
            int recv_len = tcpSocket->Receive(&temp_buffer);
            if (recv_len == 1) {
                if (temp_buffer == '\n') {
                    break;
                }
                str.push_back(temp_buffer);
                continue;
            }

            if (recv_len == 0) {
                RCLCPP_WARN(get_logger(), "Socket closed by peer.");
                return;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return;
            }

            RCLCPP_WARN(get_logger(), "Socket receive error: %d", errno);
            return;
            //std::cout << "dt: " << dt << std::endl;         
	}

	if (!rclcpp::ok() || str.empty()) {
	    return;
	}
	
	//first_time_loss = current_time;
		
	try
	{
            json_data = json::parse(str);
		
            if (json_data.contains("altitude")) {
		
		dvl.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
		dvl.header.frame_id = "dvl_A50_report_link";
		//std::cout << std::setw(4) << json_data << std::endl;
		
		dvl.time = double(json_data["time"]);
		dvl.velocity.x = double(json_data["vx"]);
		dvl.velocity.y = double(json_data["vy"]);
		dvl.velocity.z = double(json_data["vz"]);
		dvl.fom = double(json_data["fom"]);
                current_altitude = double(json_data["altitude"]);
                const bool vel_ok = json_data.value("velocity_valid", false);
		dvl.velocity_valid = vel_ok;
                if (!vel_ok) {
                    return;  // skip dvl/data and /auv/dvl publish
                }
		    
		if(current_altitude >= 0.0 && dvl.velocity_valid)
		{
		    dvl.altitude = current_altitude;
		    old_altitude = current_altitude;
		}
		else
		    dvl.altitude = old_altitude;


		dvl.status = json_data["status"];
		dvl.form = json_data["format"];
			
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

        geometry_msgs::msg::TwistWithCovarianceStamped dvl_twist_cov;
        dvl_twist_cov.header.stamp = dvl.header.stamp;
        dvl_twist_cov.header.frame_id = dvl.header.frame_id;
        dvl_twist_cov.twist.twist.linear.x = dvl.velocity.x;
        dvl_twist_cov.twist.twist.linear.y = dvl.velocity.y;
        dvl_twist_cov.twist.twist.linear.z = dvl.velocity.z;
        fill_twist_covariance_from_json(json_data, dvl_twist_cov.twist.covariance);
        dvl_pub_twist_cov->publish(dvl_twist_cov);
		
            }
            else //if (json_data.contains("pitch")) 
            {
		//std::cout << std::setw(4) << json_data << std::endl;
		DVLDeadReckoning.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
		DVLDeadReckoning.header.frame_id = "dvl_A50_position_link";
		DVLDeadReckoning.time = double(json_data["ts"]);
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
    	
	}
	catch(std::exception& e)
	{
	     UNUSED(e);
            //std::cout << "Exception: " << e.what() << std::endl;
	} 
	 	    
    } 
}




}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::LifecycleDVL)
