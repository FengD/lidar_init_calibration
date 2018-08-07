#include "yaml_editor.h"
#include <string>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

CalibrationYaml::CalibrationYaml() {

}

CalibrationYaml::~CalibrationYaml() {

}

void CalibrationYaml::yamlRead(std::string filePath) {
  YAML::Node yamlConfig = YAML::LoadFile(filePath);
  calibrationLidar.x = yamlConfig["correction_x"].as<float>();
  calibrationLidar.y = yamlConfig["correction_y"].as<float>();
  calibrationLidar.z = yamlConfig["correction_z"].as<float>();
  calibrationLidar.roll = yamlConfig["correction_roll"].as<float>();
  calibrationLidar.pitch = yamlConfig["correction_pitch"].as<float>();
  calibrationLidar.yaw = yamlConfig["correction_yaw"].as<float>();
}

void CalibrationYaml::yamlWrite(std::string filePath, Calibration_Lidar cl) {
  std::ofstream fout(filePath.c_str());
  YAML::Emitter out(fout);
  out << YAML::BeginMap;
  out << YAML::Key << "correction_x";
  out << YAML::Value << cl.x;
  out << YAML::Comment("given by hand if necessary");
  out << YAML::Key << "correction_y";
  out << YAML::Value << cl.y;
  out << YAML::Comment("given by hand if necessary");
  out << YAML::Key << "correction_z";
  out << YAML::Value << cl.z;
  out << YAML::Key << "correction_roll";
  out << YAML::Value << cl.roll;
  out << YAML::Key << "correction_pitch";
  out << YAML::Value << cl.pitch;
  out << YAML::Key << "correction_yaw";
  out << YAML::Value << cl.yaw;
  out << YAML::Comment("given by hand if necessary");
  out << YAML::EndMap;
}

Calibration_Lidar CalibrationYaml::getCalibrationLidar() {
  return calibrationLidar;
}
