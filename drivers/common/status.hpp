#ifndef NEBULA_STATUS_HPP
#define NEBULA_STATUS_HPP


namespace nebula {
enum class STATUS {
  OK = 0,
  ERROR_1 = 1
};

std::string NebulaStatusToString(STATUS nebula_status) {
  switch (nebula_status) {
    case STATUS::OK:
      return "OK";
    case STATUS::ERROR_1:
      return "RUNTIME STOPPED";
  }
};

}
#endif //NEBULA_WS_STATUS_HPP
