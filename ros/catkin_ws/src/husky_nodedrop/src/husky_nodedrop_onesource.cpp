#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <sstream>
#include <cstdlib> // for getenv()

// Callback function to handle HTTP response
size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

void getDropStatus(const std::string& ip, double rssi_threshold, ros::Publisher& drop_alert) {
    CURL *curl;
    CURLcode res;
    std::string response_string;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if (!curl) {
        ROS_ERROR("CURL initialization failed!");
        return;
    }

    std::string url = "http://" + ip + "/cgi-bin/luci/iwt/dashboard/get-data";
    ROS_INFO("Attempting to fetch data from: %s", url.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

    // Enable verbose logging for debugging
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

    res = curl_easy_perform(curl);

    if (res != CURLE_OK) {
        ROS_ERROR("CURL request failed: %s", curl_easy_strerror(res));
    } else {
        ROS_INFO("Received response: %s", response_string.c_str());

        // If response is empty, print a warning
        if (response_string.empty()) {
            ROS_WARN("Response from %s is empty. Check if the server is running!", url.c_str());
        }

        // Parse the JSON response
        Json::Reader reader;
        Json::Value data;
        if (reader.parse(response_string, data)) {
            if (data["status"].asString() == "success") {
                double max_rssi = -1e9;
                Json::Value best_neighbor;
                
                // Loop through neighbors and find the one with the best RSSI
                for (const auto& neighbor : data["neighbors"]) {
                    double n_rssi = neighbor[6].asDouble();
                    if (n_rssi > max_rssi) {
                        max_rssi = n_rssi;
                        best_neighbor = neighbor;
                    }
    // Read environment variables or fallback to default values if not set
    const char* ip_env = std::getenv("IP_ADDRESS");
    std::string ip = (ip_env != nullptr) ? ip_env : "192.168.1.1";  // Default if not set

    const char* polling_rate_env = std::getenv("POLLING_RATE");
    double polling_rate = (polling_rate_env != nullptr) ? std::stod(polling_rate_env) : 1.0;  // Default if not set

    const char* rssi_threshold_env = std::getenv("RSSI_THRESHOLD");
    double rssi_threshold = (rssi_threshold_env != nullptr) ? std::stod(rssi_threshold_env) : -80.0;  // Default if not set

    // Use the values (ip, polling_rate, rssi_threshold) as needed
    ROS_INFO("IP Address: %s", ip.c_str());
    ROS_INFO("Polling Rate: %f", polling_rate);
    ROS_INFO("RSSI Threshold: %f", rssi_threshold);
                }

                ROS_INFO("Best Reverse RSSI: %.3f from %s", max_rssi, best_neighbor[0].asString().c_str());
                
                // Publish the result based on RSSI threshold
                std_msgs::Bool msg;
                if (max_rssi < rssi_threshold) { 
                    msg.data = true;  // RSSI is below the threshold
                    ROS_INFO("RSSI below threshold. Triggering drop...");
                } else {
                    msg.data = false;  // RSSI is above the threshold
                    ROS_INFO("RSSI good. NO NEED to trigger drop...");
                }

                // Publish the message to drop_alert topic
                drop_alert.publish(msg);
            } else {
                ROS_ERROR("Error: %s", data["status"].asString().c_str());
            }
        } else {
            ROS_ERROR("Failed to parse JSON: %s", reader.getFormattedErrorMessages().c_str());
        }
    }

    curl_easy_cleanup(curl);
    curl_global_cleanup();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "husky_nodedrop");
    ros::NodeHandle nh;
    
    // Define the publisher for the drop status topic
    ros::Publisher drop_alert = nh.advertise<std_msgs::Bool>("drop_status", 10);

    // Read environment variables or fallback to default values if not set
    const char* ip_env = std::getenv("HUSKY_NODEDROP_IP");
    std::string ip = (ip_env != nullptr) ? ip_env : "192.168.1.1:5000";  // Default if not set

    const char* polling_rate_env = std::getenv("HUSKY_POLLING_RATE");
    double polling_rate = (polling_rate_env != nullptr) ? std::stod(polling_rate_env) : 1.0;  // Default if not set

    const char* rssi_threshold_env = std::getenv("HUSKY_RSSI_THRESHOLD");
    double rssi_threshold = (rssi_threshold_env != nullptr) ? std::stod(rssi_threshold_env) : -80.0;  // Default if not set

    // Use the values (ip, polling_rate, rssi_threshold) as needed
    ROS_INFO("IP Address: %s", ip.c_str());
    ROS_INFO("Polling Rate: %f", polling_rate);
    ROS_INFO("RSSI Threshold: %f", rssi_threshold);

    ros::Rate loop_rate(polling_rate);

    while (ros::ok()) {
        getDropStatus(ip, rssi_threshold, drop_alert);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

