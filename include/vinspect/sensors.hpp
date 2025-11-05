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

struct SparseValueInfo {
  std::string name;
  std::string unit;

  bool operator==(const SparseValueInfo& other) const {
    return (this->name == other.name and
            this->unit == other.unit);
  }

};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SparseValueInfo, name, unit);

class DenseSensor
{
public:
  explicit DenseSensor() = default;  // We need this for the auto serialization, do not use otherwise
  explicit DenseSensor(int id, unsigned int width, unsigned int height) : id_{id}, resolution_(width, height) {};
  std::size_t numPixels() const { return resolution_.first * resolution_.second; }
  std::pair<unsigned int, unsigned int> getResolution() const {return resolution_;}
  unsigned int getWidth() const { return resolution_.first; }
  unsigned int getHeight() const { return resolution_.second; }
  int getId() const { return id_; }

private:
  int id_ = 0;
  std::pair<unsigned int, unsigned int> resolution_;

NLOHMANN_DEFINE_TYPE_INTRUSIVE(DenseSensor, id_, resolution_)
};


}  // namespace vinspect

#endif  // VINSPECT__SENSORS_H_
