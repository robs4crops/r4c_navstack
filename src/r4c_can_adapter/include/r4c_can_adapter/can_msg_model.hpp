#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace r4c_can_adapter
{
  struct CanMsgModel
  {
    uint64_t id;
    std::string name;
    std::vector<std::string> signals;
  };
}  // namespace r4c_can_adapter
