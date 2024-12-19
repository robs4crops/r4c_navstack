#include <r4c_navigation/action/super_wait.hpp>

SuperWait::SuperWait(const std::string& name, const BT::NodeConfiguration& conf):
  BT::CoroActionNode(name, conf),
  timeout_(5.0)
{
  getInput("timeout", timeout_);
}

BT::NodeStatus SuperWait::tick()
{
  if(status() == BT::NodeStatus::IDLE)
  {
    // Reset the starting point since we're starting a new iteration
    start_ = std::chrono::high_resolution_clock::now();
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  auto now     = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  if(seconds.count() >= timeout_)
  {
    return BT::NodeStatus::SUCCESS;
  }

  return status();
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SuperWait>("SuperWait");
}
