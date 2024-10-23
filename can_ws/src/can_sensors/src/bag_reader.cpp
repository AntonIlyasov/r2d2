/***********************************************************************************************************************************
Описание

код чтения аднных из bag файла
 
Разработчик: -----------
Заметки
принимает в себя имя файла и имя топика для чтения всех его значений
***********************************************************************************************************************************/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <typeinfo>
#include <vector>
#include <string>
#include <cstdlib>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
// #include "rosbag_test/Pose.h"
// #include "rosbag_test/Pos2.h"
#include <algorithm>
#include "can_sensors/KeepAlive.h"




std::string replaceColon(const std::string& str) {
    std::string result = str;
    std::replace(result.begin(), result.end(), ':', ',');
    return result;
}

std::string timeToString(const ros::Time& time) {
    std::stringstream ss;
    ss << time.sec << "." << time.nsec;
    return ss.str();
}

void saveToCSV(const std::string& file_path, const std::vector<std::vector<std::string>>& data) {
    std::ofstream file(file_path);
    //file << "Time,Parameter,Value\n";
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i != row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }
    file.close();
}





    

template<typename T>
void processMessage(const T& msg, std::vector<std::vector<std::string>>& data, ros::Time timestamp) {
    std::vector<std::string> row;
    std::stringstream ss;
    std::stringstream ss_time;
    ss_time << msg << "Time," << timeToString(timestamp) <<",";
    std::string msg_str = replaceColon(ss_time.str());
    row.push_back(msg_str);
    data.push_back(row);
    std::cout<< "Time: "  << timeToString(timestamp) << std::endl; 
    std::cout<< msg << std::endl;

    
  
}


std::string sanitizeTopicName(const std::string& topic) {
    std::string sanitized = topic;
    std::replace(sanitized.begin(), sanitized.end(), '/', '_');
    return sanitized;
}






void restructureCSV(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Could not open CSV file for reading." << std::endl;
        return;
    }

    std::vector<std::vector<std::string>> data;
    std::vector<std::string> headers;
    std::map<std::string, std::vector<std::string>> values_map;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }

        if (row.size() == 2) { // assuming each row has a key and a value
            std::string key = row[0];
            std::string value = row[1];

            if (values_map.find(key) == values_map.end()) {
                headers.push_back(key);
            }
            values_map[key].push_back(value);
        }
    }
    file.close();

    std::ofstream outfile(file_path);
    if (!outfile.is_open()) {
        std::cerr << "Could not open CSV file for writing." << std::endl;
        return;
    }

    // Write headers
    for (size_t i = 0; i < headers.size(); ++i) {
        outfile << headers[i];
        if (i != headers.size() - 1) {
            outfile << ",";
        }
    }
    outfile << "\n";

    // Write values
    size_t rowCount = 0;
    for (const auto& key : headers) {
        rowCount = std::max(rowCount, values_map[key].size());
    }

    for (size_t i = 0; i < rowCount; ++i) {
        for (size_t j = 0; j < headers.size(); ++j) {
            if (i < values_map[headers[j]].size()) {
                outfile << values_map[headers[j]][i];
            }
            if (j != headers.size() - 1) {
                outfile << ",";
            }
        }
        outfile << "\n";
    }

    outfile.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_to_csv");
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <bag_file> <topic>" << std::endl;
        return 1;
    }

    std::string bag_file = argv[1];
    std::string topic = argv[2];
    std::string sanitized_topic = sanitizeTopicName(topic);

    const char* home_dir = getenv("HOME");
    if (!home_dir) {
        std::cerr << "Could not get home directory" << std::endl;
        return 1;
    }

    std::string file_path = std::string(home_dir) + "/" + sanitized_topic + ".csv";

    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagException& e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return 1;
    }

    std::vector<std::vector<std::string>> data;

    rosbag::View view(bag, rosbag::TopicQuery(topic));
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        ros::Time timestamp = m.getTime();
        //std::string timestamp_str = timeToString(timestamp);

        if (m.getDataType() == "std_msgs/String") {
            std_msgs::String::ConstPtr msg = m.instantiate<std_msgs::String>();
            if (msg != nullptr) {
                processMessage(*msg, data, timestamp);
            }
        } else if (m.getDataType() == "std_msgs/Float64") {
            std_msgs::Float64::ConstPtr msg = m.instantiate<std_msgs::Float64>();
            if (msg != nullptr) {
                processMessage(*msg, data, timestamp);
            }
        } 
        // else if (m.getDataType() == "rosbag_test/Pose") {
        //     rosbag_test::Pose::ConstPtr msg = m.instantiate<rosbag_test::Pose>();
        //     if (msg != nullptr) {
        //         processMessage(*msg, data, timestamp);
                
        //     }
        // }
        // else if (m.getDataType() == "rosbag_test/Pos2") {
        //     rosbag_test::Pos2::ConstPtr msg = m.instantiate<rosbag_test::Pos2>();
        //     if (msg != nullptr) {
        //         processMessage(*msg, data, timestamp);
                
        //     }
        // }
        else if (m.getDataType() == "can_sensors/KeepAlive") {
            can_sensors::KeepAlive::ConstPtr msg = m.instantiate<can_sensors::KeepAlive>();
            if (msg != nullptr) {
                processMessage(*msg, data, timestamp);
                
            }
        }

        else if (m.getDataType() == "can_sensors/KeepAlive") {
            can_sensors::KeepAlive::ConstPtr msg = m.instantiate<can_sensors::KeepAlive>();
            if (msg != nullptr) {
                processMessage(*msg, data, timestamp);
                
            }
        }
        
        else 
        {
            std::cerr << "Unknown message type: " << m.getDataType() << std::endl;
        }
        
    }

    saveToCSV(file_path, data);
    bag.close();
    restructureCSV(file_path);


   

    

    return 0;
}
