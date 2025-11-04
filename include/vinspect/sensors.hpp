// SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef VINSPECT__SENSORS_H_
#define VINSPECT__SENSORS_H_

#include <vector>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include "vinspect/utils.hpp"

using json = nlohmann::json;

namespace vinspect
{

class Sensor
{
public:
  explicit Sensor() = default;

  int getId() const { return id_; }

  // Important! Allow overwrite of destructor
  virtual ~Sensor() = default;
protected:
  void setId(int id) {id_ = id;}
  int id_ = 0;
};

struct ValueInfo {
  std::string name;
  std::string unit;

  bool operator==(const ValueInfo& other) const {
    return (this->name == other.name and
            this->unit == other.unit);
  }

};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ValueInfo, name, unit);

class SparseSensor :  public Sensor
{
public:
  explicit SparseSensor() = default;
  explicit SparseSensor(int id, const std::vector<ValueInfo> value_infos) : value_infos_{value_infos} {setId(id);}
  std::vector<ValueInfo> getValueInfos() const { return value_infos_; }
  std::size_t numValues() const { return value_infos_.size(); }
  
private:
  std::vector<ValueInfo> value_infos_;

NLOHMANN_DEFINE_TYPE_INTRUSIVE(SparseSensor, id_, value_infos_);
};


class DenseSensor : public Sensor
{
public:
  explicit DenseSensor() = default;
  explicit DenseSensor(int id, unsigned int width, unsigned int height) : resolution_(width, height) {setId(id);}
  std::size_t numPixels() const { return resolution_.first * resolution_.second; }
  std::pair<unsigned int, unsigned int> getResolution() const {return resolution_;}
  unsigned int getWidth() const { return resolution_.first; }
  unsigned int getHeight() const { return resolution_.second; }

private:
  std::pair<unsigned int, unsigned int> resolution_;

NLOHMANN_DEFINE_TYPE_INTRUSIVE(DenseSensor, id_, resolution_)
};


}  // namespace vinspect

#endif  // VINSPECT__SENSORS_H_
