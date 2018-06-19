#include "ParamHandler.hpp"
#include <iostream>

ParamHandler::ParamHandler(const std::string & file_name){
  config_ = YAML::LoadFile(file_name);
}

ParamHandler::~ParamHandler(){}

bool ParamHandler::getString(const std::string & key, std::string& str_value){
  str_value = config_[key].as<std::string>();
  return true;
}

bool ParamHandler::getVector(const std::string & key,
                             std::vector<double> & vec_value){
  vec_value = config_[key].as<std::vector<double> >();
  return true;
}

bool ParamHandler::getValue(const std::string & key, double & double_value){
  double_value = config_[key].as<double>();
  return true;
}

bool ParamHandler::getBoolean(const std::string & key, bool & bool_value){
  bool_value = config_[key].as<bool>();
  return true;
}

bool ParamHandler::getInteger(const std::string & key, int & int_value){
  int_value = config_[key].as<int>();
  return true;
}
