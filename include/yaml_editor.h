// The header file of the yaml generators

#ifndef _YAML_EDITOR_H_
#define _YAML_EDITOR_H_

#include <string>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

// The struct of the 6D pose
struct Calibration_Lidar{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};

// The yaml class
class CalibrationYaml{

  private:
    Calibration_Lidar calibrationLidar;

  public:
    // Constructor
    CalibrationYaml();
    // Destructor
    ~CalibrationYaml();
    // yaml writer for create the calibration file and for write the value in it
    void yamlWrite(std::string filePath, Calibration_Lidar cl);
    // yaml reader for read the calibration yaml file
    void yamlRead(std::string filePath);
    // get the calibration result
    Calibration_Lidar getCalibrationLidar();
};


#endif
