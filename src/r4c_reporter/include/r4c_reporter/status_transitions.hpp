#include <cstdint>
#include <set>

namespace state_transitions
{

  template<typename StatusMsg>
  inline void clearErrorCodes(StatusMsg& status)
  {
    status.errors.clear();
  }

  template<typename StatusMsg>
  inline void clearWarningCodes(StatusMsg& status)
  {
    status.warnings.clear();
  }

  template<typename StatusMsg>
  inline void changeStateToOk(StatusMsg& status)
  {
    status.state = StatusMsg::STATE_OK;
    clearErrorCodes(status);
    clearWarningCodes(status);
  }

  template<typename StatusMsg>
  inline void changeStateToInactive(StatusMsg& status)
  {
    status.state = StatusMsg::STATE_INACTIVE;
    clearErrorCodes(status);
    clearWarningCodes(status);
  }

  template<typename StatusMsg>
  inline void evaluateStateChange(StatusMsg& status)
  {
    if(status.errors.size() > 0)
      status.state = StatusMsg::STATE_ERROR;
    else if(status.errors.size() == 0 && status.warnings.size() > 0)
      status.state = StatusMsg::STATE_WARNING;
    else if(status.errors.size() == 0 && status.warnings.size() == 0)
      status.state = StatusMsg::STATE_OK;
  }

  template<typename StatusMsg>
  inline void addErrorCode(const unsigned int code, StatusMsg& status)
  {
    std::set<unsigned int> error_set(status.errors.begin(), status.errors.end());
    error_set.insert(code);
    status.errors.assign(error_set.begin(), error_set.end());
    evaluateStateChange(status);
  }

  template<typename StatusMsg>
  inline void addWarningCode(const unsigned int code, StatusMsg& status)
  {
    std::set<unsigned int> warning_set(status.warnings.begin(), status.warnings.end());
    warning_set.insert(code);
    status.warnings.assign(warning_set.begin(), warning_set.end());
    evaluateStateChange(status);
  }

  template<typename StatusMsg>
  inline void removeErrorCode(const unsigned int code, StatusMsg& status)
  {
    if(status.errors.size() > 0)
    {
      std::set<unsigned int> error_set(status.errors.begin(), status.errors.end());
      error_set.erase(code);
      status.errors.assign(error_set.begin(), error_set.end());
    }
    evaluateStateChange(status);
  }

  template<typename StatusMsg>
  inline void removeWarningCode(const unsigned int code, StatusMsg& status)
  {
    if(status.warnings.size() > 0)
    {
      std::set<unsigned int> warning_set(status.warnings.begin(), status.warnings.end());
      warning_set.erase(code);
      status.warnings.assign(warning_set.begin(), warning_set.end());
    }
    evaluateStateChange(status);
  }

  template<typename StatusMsg>
  inline bool checkWarningInStatus(const unsigned int code, StatusMsg& status)
  {
    if(status.warnings.size() > 0)
    {
      auto warning = std::find(status.warnings.begin(), status.warnings.end(), code);
      if (warning != status.warnings.end())
        return true; //Found code in warning list
      else
        return false;
    }
    else
    {
      return false;
    }
  }

  template<typename StatusMsg>
  inline bool checkErrorInStatus(const unsigned int code, StatusMsg& status)
  {
    if(status.errors.size() > 0)
    {
      auto error = std::find(status.errors.begin(), status.errors.end(), code);
      if (error != status.errors.end())
        return true; //Found code in error list
      else
        return false;
    }
    else
    {
      return false;
    }
  }


}  // namespace state_transitions
