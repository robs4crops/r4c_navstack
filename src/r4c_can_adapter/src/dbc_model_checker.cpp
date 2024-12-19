#include <filesystem>
#include <fstream>

#include "dbcppp/Message.h"
#include "dbcppp/Network.h"
#include "dbcppp/Signal.h"

#include "r4c_can_adapter/dbc_model_checker.hpp"

namespace r4c_can_adapter
{
  DbcModelChecker::DbcModelChecker(const std::unordered_map<uint64_t, CanMsgModel>& dbc_model,
                                   const dbcppp::INetwork& dbc)
  {
    auto dbc_iterable = dbc.Messages();

    // Iterate through the messages present in the dbc model.
    for(const auto& [id, can_msg_model]: dbc_model)
    {
      // Check if the can message reppresented by the model is defined in the dbc file.
      auto dbc_it = std::find_if(dbc_iterable.begin(),
                                 dbc_iterable.end(),
                                 [&can_msg_model](const dbcppp::IMessage& can_msg_def) {
                                   return can_msg_def.Id() == can_msg_model.id;
                                 });

      // auto dbc_model_it = dbc_model.find(can_msg_def.Id());

      if(dbc_it == dbc_iterable.end())
      {
        std::string can_msg_id = std::to_string(can_msg_model.id);
        constexpr std::string_view str_1{"Can msg model '"};
        constexpr std::string_view str_2{"' is not present in the dbc file"};
        constexpr auto size = str_1.size() + str_2.size();

        std::string msg;
        msg.reserve(size + can_msg_id.length());
        msg.append(str_1).append(can_msg_id).append(str_2);

        throw std::invalid_argument{msg};
      }

      const auto& can_msg_def = *dbc_it;

      // Check if the can msg found by id, has the same name in the dbc file and in the dbc model.
      if(can_msg_model.name != can_msg_def.Name())
      {
        std::string can_msg_id = std::to_string(can_msg_def.Id());

        constexpr std::string_view str_1{"Can msg '"};
        constexpr std::string_view str_2{"' is called '"};
        constexpr std::string_view str_3{" in the dbc model, but called '"};
        constexpr std::string_view str_4{"' in the dbc file"};
        constexpr auto size = str_1.size() + str_2.size() + str_3.size() + str_4.size();

        std::string msg;
        msg.reserve(size + can_msg_id.length() + can_msg_model.name.size() +
                    can_msg_def.Name().size());
        msg.append(str_1)
          .append(can_msg_id)
          .append(str_2)
          .append(can_msg_model.name)
          .append(str_3)
          .append(can_msg_def.Name())
          .append(str_4);

        throw std::invalid_argument{msg};
      }

      // Check if the can msg found by id, has the same number of signals in the dbc file and in the
      // dbc model.
      if(can_msg_model.signals.size() != can_msg_def.Signals_Size())
      {
        std::string can_msg_id = std::to_string(can_msg_def.Id());

        std::string size_1 = std::to_string(can_msg_model.signals.size());
        std::string size_2 = std::to_string(can_msg_def.Signals_Size());

        constexpr std::string_view str_1{"Can msg '"};
        constexpr std::string_view str_2{"' has '"};
        constexpr std::string_view str_3{"' signals in the dbc model, but '"};
        constexpr std::string_view str_4{"' signals in the dbc file"};
        constexpr auto size = str_1.size() + str_2.size() + str_3.size() + str_4.size();

        std::string msg;
        msg.reserve(size + can_msg_id.length() + size_1.length() + size_2.length());
        msg.append(str_1)
          .append(can_msg_id)
          .append(str_2)
          .append(size_1)
          .append(str_3)
          .append(size_2)
          .append(str_4);

        throw std::invalid_argument{msg};
      }

      // Check if the can msg found by id, has the same name for the signals in the dbc file and in
      // the dbc model.
      auto signal_iterable = can_msg_def.Signals();
      //
      // for(size_t i{0}, size = can_msg_def.Signals_Size(); i < size; ++i)
      for(size_t i{0}, size = can_msg_model.signals.size(); i < size; ++i)
      {
        // const auto& sig_name = can_msg_def.Signals_Get(i).Name();
        const auto& signal_name = can_msg_model.signals[i];

        auto signal_it = std::find_if(signal_iterable.begin(),
                                      signal_iterable.end(),
                                      [&signal_name](const dbcppp::ISignal& signal) {
                                        return signal.Name() == signal_name;
                                      });

        if(signal_it == signal_iterable.end())
        {
          std::string can_msg_id = std::to_string(can_msg_def.Id());
          constexpr std::string_view str_1{"Can msg '"};
          constexpr std::string_view str_2{"' has the signal '"};
          constexpr std::string_view str_3{"' in the dbc model, but not in the dbc file"};
          constexpr auto size = str_1.size() + str_2.size() + str_3.size();

          std::string msg;
          msg.reserve(size + can_msg_id.length() + signal_name.length());
          msg.append(str_1).append(can_msg_id).append(str_2).append(signal_name).append(str_3);

          throw std::invalid_argument{msg};
        }
      }
    }
  }
}  // namespace r4c_can_adapter
