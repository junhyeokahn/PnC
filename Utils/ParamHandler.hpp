#ifndef PARAMETER_HANDLER
#define PARAMETER_HANDLER

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

class ParamHandler{
public:
  ParamHandler(const std::string & file_name);
  virtual ~ParamHandler();

  bool getString(const std::string & key, std::string & str_value);
  bool getVector(const std::string & key, std::vector<double> & vec_value);
  bool getValue(const std::string & key, double & double_value);
  bool getBoolean(const std::string & key, bool & bool_value);
  bool getInteger(const std::string & key, int & int_value);

protected:
  YAML::Node config_;
};
#endif
