#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <sstream>

// Callback function to handle HTTP response data
size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *output) {
    size_t total_size = size * nmemb;  // Calculate total size of received data
    output->append((char*)contents, total_size);  // Append received data to output string
    return total_size;
}

// Function to fetch and process the drop status of the network node
void getDropStatus(const std::string& ip, double rssi_threshold) {
    CURL *curl;
    CURLcode res;
    std::string response_string;  // Stores the HTTP response

    // Initialize CURL
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if (!curl) {
        ROS_ERROR("CURL initialization failed!");
        return;  // Exit the function if CURL fails to initialize
    }

    // Construct the URL to request network node data
    std::string url = "http://" + ip + "/cgi-bin/luci/iwt/dashboard/get-data";
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());  // Set request URL
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);  // Set callback for response handling
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);  // Store response in response_string

    // Perform HTTP GET request
    res = curl_easy_perform(curl);

    if (res != CURLE_OK) {
        ROS_ERROR("Error with request: %s", curl_easy_strerror(res));  // Print CURL error if request fails
    } else {
        ROS_INFO("Received response: %s", response_string.c_str());

        // Parse the JSON response
        Json::Reader reader;
        Json::Value data;
        if (reader.parse(response_string, data)) {
            // Ensure that the response contains a success status
            if (data["status"].asString() == "success") {
                double max_rssi = -1e9;  // Initialize max RSSI to a very low value
                Json::Value best_neighbor;  // Variable to store the best neighbor node

                // Iterate through neighbors and determine the best RSSI value
                for (const auto& neighbor : data["neighbors"]) {
                    double n_rssi = neighbor[6].asDouble();  // Extract RSSI value
                    if (n_rssi > max_rssi) {
                        max_rssi = n_rssi;  // Update max RSSI
                        best_neighbor = neighbor;  // Store best neighbor
                    }
                }

                // Log the best RSSI value found
                ROS_INFO("Best Reverse RSSI: %.3f from %s", max_rssi, best_neighbor[0].asString().c_str());

                // Check if RSSI is below the defined threshold
                if (max_rssi < rssi_threshold) {
                    ROS_WARN("RSSI below threshold. Triggering drop...");  // Log warning if drop condition met
                }
            } else {
                ROS_ERROR("Error: %s", data["status"].asString().c_str());  // Log error if JSON response has an issue
            }
        } else {
            ROS_ERROR("Failed to parse JSON: %s", reader.getFormattedErrorMessages().c_str());  // Log JSON parsing errors
        }
    }

    // Cleanup CURL resources
    curl_easy_cleanup(curl);
    curl_global_cleanup();
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "husky_nodedrop");
    ros::NodeHandle nh;

    std::string ip;
    double polling_rate;
    double rssi_threshold;

    // Get parameters from the launch file or set defaults if not provided
    nh.param("ip_address", ip, std::string("192.168.1.1"));
    nh.param("polling_rate", polling_rate, 1.0);
    nh.param("rssi_threshold", rssi_threshold, -80.0);

    ros::Rate loop_rate(polling_rate);  // Set loop rate according to polling_rate

    // Main loop that continuously checks drop status
    while (ros::ok()) {
        getDropStatus(ip, rssi_threshold);  // Fetch and process drop status
        ros::spinOnce();  // Allow ROS callbacks to execute
        loop_rate.sleep();  // Maintain loop rate
    }

    return 0;
}

