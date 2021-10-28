#ifndef NEBULA_STATUS_HPP
#define NEBULA_STATUS_HPP

namespace nebula
{
enum class Status { OK = 0, ERROR_1 = 1 };

std::string NebulaStatusToString(Status nebula_status)
{
  switch (nebula_status) {
    case Status::OK:
      return "OK";
    case Status::ERROR_1:
    default:
      return "RUNTIME STOPPED";
  }
}

}  // namespace nebula
#endif  //NEBULA_STATUS_HPP
