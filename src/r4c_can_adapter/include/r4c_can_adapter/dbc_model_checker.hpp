#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "dbcppp/Network.h"

#include "r4c_can_adapter/can_msg_model.hpp"

namespace r4c_can_adapter
{
  class DbcModelChecker
  {
    public:
    DbcModelChecker(const std::unordered_map<uint64_t, CanMsgModel>& dbc_model,
                    const dbcppp::INetwork& dbc);
  };
}  // namespace r4c_can_adapter
