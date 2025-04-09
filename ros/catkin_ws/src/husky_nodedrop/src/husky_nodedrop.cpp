#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <sstream>
#include <iterator>

// Callback function to handle HTTP response from cURL
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

// Function to get parameters
bool getRosParam(ros::NodeHandle &nh, std::string param_name, std::vector<std::string> &param_value) {
    if (nh.getParam(param_name, param_value)) {
        return true;
    } else {
        ROS_ERROR("No IP addresses provided in the ROS parameter server!");
        return false;
    }
}

bool getRosParam2(ros::NodeHandle &nh, const std::string &param_name, std::vector<std::string> &param_value) {
    // Try to get it directly as a vector of strings
    if (nh.getParam(param_name, param_value)) {
        return true;
    }

    // If that fails, try to get it as a string and split
    std::string param_string;
    if (nh.getParam(param_name, param_string)) {
        std::istringstream iss(param_string);
        param_value = std::vector<std::string>{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
        if (!param_value.empty()) {
            ROS_WARN("IP addresses were provided as a single string. Split into %lu entries.", param_value.size());
            return true;
        }
    }

    // If all fails
    ROS_ERROR("No valid IP addresses found in the ROS parameter server!");
    return false;
}

// Function to fetch data from a given IP address
bool fetchRSSIData(const std::string& ip, std::string& response) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        ROS_ERROR("CURL initialization failed!");
        return false;
    }

    std::string url = "http://" + ip + "/cgi-bin/luci/iwt/dashboard/get-data";
    ROS_INFO("Fetching data from: %s", url.c_str());

    // Configure cURL
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);          // Total timeout: 3 seconds
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);   // Connection timeout: 2 seconds

    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        ROS_ERROR("CURL request to %s failed: %s", ip.c_str(), curl_easy_strerror(res));
        return false;
    }

    return !response.empty();
}

// Function to process JSON and check RSSI values
bool processRSSIData(const std::string& ip, const std::string& response, double rssi_threshold) {
    Json::CharReaderBuilder reader;
    Json::Value data;
    std::string errs;

    std::istringstream s(response);
    if (!Json::parseFromStream(reader, s, &data, &errs)) {
        ROS_ERROR("Failed to parse JSON from %s: %s", ip.c_str(), errs.c_str());
        return false;
    }

    if (data["status"].asString() != "success") {
        ROS_ERROR("Error in JSON response from %s: %s", ip.c_str(), data["status"].asString().c_str());
        return false;
    }

    bool has_valid_rssi = false;
    bool any_above_threshold = false;

    for (const auto& neighbor : data["neighbors"]) {
        // ROS_INFO("Reading neighbor %s", neighbor[0].asString().c_str());
        if (neighbor.size() > 6 && neighbor[6].isString()) {
            try {
                double n_rssi = std::stod(neighbor[6].asString());
                ROS_INFO("RSSI from %s: %.3f", neighbor[0].asString().c_str(), n_rssi);
                
                if (n_rssi >= rssi_threshold) {
                    any_above_threshold = true;  // At least one RSSI is above the threshold
                }
                has_valid_rssi = true;
            } catch (const std::exception& e) {
                ROS_WARN("Failed to convert RSSI value for neighbor %s: %s", neighbor[0].asString().c_str(), e.what());
            }
        }
    }

    if (!has_valid_rssi) {
        ROS_WARN("No valid RSSI values found in response from %s", ip.c_str());
        return false;
    }

    return any_above_threshold;  // Return true if at least one RSSI is above threshold
}

// Main function to fetch and evaluate drop status
void getDropStatus(const std::vector<std::string>& ip_addresses, double rssi_threshold, ros::Publisher& drop_alert) {
    bool any_above_threshold = false;

    for (const auto& ip : ip_addresses) {
        std::string response;
        
        if (!fetchRSSIData(ip, response)) {
            continue;  // Skip this IP if the request failed
        }

        if (processRSSIData(ip, response, rssi_threshold)) {
            any_above_threshold = true;  // If any neighbor is above the threshold, cancel the drop
        }
    }

    // Publish the final decision
    std_msgs::Bool msg;
    msg.data = !any_above_threshold;  // Drop if ALL are below the threshold
    drop_alert.publish(msg);

    if (!any_above_threshold) {
        ROS_WARN("All neighbors' RSSI below threshold. Triggering node drop...");
    } else {
        ROS_INFO("At least one neighbor has an acceptable RSSI. No drop triggered.");
    }
}

// For debugging purposes only
void debugPrintParamType(const ros::NodeHandle& nh, const std::string& param_name) {
    XmlRpc::XmlRpcValue param;
    if (!nh.getParam(param_name, param)) {
        ROS_WARN("Param '%s' not found.", param_name.c_str());
        return;
    }

    switch (param.getType()) {
        case XmlRpc::XmlRpcValue::TypeBoolean:
            ROS_INFO("Param '%s' is of type: Boolean", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeInt:
            ROS_INFO("Param '%s' is of type: Integer", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeDouble:
            ROS_INFO("Param '%s' is of type: Double", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeString:
            ROS_INFO("Param '%s' is of type: String", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeArray:
            ROS_INFO("Param '%s' is of type: Array", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeStruct:
            ROS_INFO("Param '%s' is of type: Struct", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeDateTime:
            ROS_INFO("Param '%s' is of type: DateTime", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeBase64:
            ROS_INFO("Param '%s' is of type: Base64", param_name.c_str());
            break;
        case XmlRpc::XmlRpcValue::TypeInvalid:
        default:
            ROS_WARN("Param '%s' is of invalid or unknown type", param_name.c_str());
            break;
    }
}


int main(int argc, char** argv) {
    const char* topic_env = std::getenv("HUSKY_NODEDROP_TOPIC");
    std::string nodedrop_topic_name = topic_env ? std::string(topic_env) : "need_new_node_drop";
    
    ros::init(argc, argv, "husky_nodedrop_node");
    ros::NodeHandle nh("~");  // Use private namespace to read parameters

    std::vector<std::string> ip_addresses;
    double polling_rate;
    double rssi_threshold;
    
    debugPrintParamType(nh, "ip_addresses"); //debugging purposes

    // Fetch parameters from ROS param server
    if (!getRosParam2(nh, "ip_addresses", ip_addresses)) {
        return -1;  // Exit if IP addresses are missing
    }
    nh.param("polling_rate", polling_rate, 1.0);
    nh.param("rssi_threshold", rssi_threshold, -80.0);

    // Ensure at least one IP is provided
    if (ip_addresses.empty()) {
        ROS_ERROR("No IP addresses provided in the ROS parameter server!");
        return 1;
    }

    // Print loaded parameters
    ROS_INFO("Loaded IP Addresses:");
    for (const auto &ip : ip_addresses) {
        ROS_INFO("- %s", ip.c_str());
    }
    ROS_INFO("Polling Rate: %f", polling_rate);
    ROS_INFO("RSSI Threshold: %f", rssi_threshold);

    // Define the publisher for the drop status topic
    ros::Publisher drop_alert = nh.advertise<std_msgs::Bool>(nodedrop_topic_name, 10);

    ros::Rate loop_rate(polling_rate);  // Control the rate of execution

    // Main loop
    while (ros::ok()) {
        getDropStatus(ip_addresses, rssi_threshold, drop_alert);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

