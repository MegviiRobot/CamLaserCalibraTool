//
// Created by heyijia on 18-12-13.
//

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <iostream>

extern std::string savePath;
extern std::string bag_path;
extern std::string scan_topic_name;
extern std::string img_topic_name;

void readParameters(std::string config_file);
#endif //PROJECT_CONFIG_H
