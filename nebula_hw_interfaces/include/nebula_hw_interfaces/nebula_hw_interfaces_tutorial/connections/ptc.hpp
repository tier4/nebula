#pragma once

#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp"

#include <boost_tcp_driver/tcp_driver.hpp>
#include <nebula_common/util/expected.hpp>

#include <cstdint>
#include <iomanip>
#include <mutex>

namespace nebula
{
namespace drivers
{
namespace connections
{

class PtcConnection
{
public:
  static const uint8_t PTC_ERROR_CODE_NO_ERROR = 0x00;
  static const uint8_t PTC_ERROR_CODE_INVALID_INPUT_PARAM = 0x01;
  static const uint8_t PTC_ERROR_CODE_SERVER_CONN_FAILED = 0x02;
  static const uint8_t PTC_ERROR_CODE_INVALID_DATA = 0x03;
  static const uint8_t PTC_ERROR_CODE_OUT_OF_MEMORY = 0x04;
  static const uint8_t PTC_ERROR_CODE_UNSUPPORTED_CMD = 0x05;
  static const uint8_t PTC_ERROR_CODE_FPGA_COMM_FAILED = 0x06;
  static const uint8_t PTC_ERROR_CODE_OTHER = 0x07;

  static const uint8_t TCP_ERROR_UNRELATED_RESPONSE = 1;
  static const uint8_t TCP_ERROR_UNEXPECTED_PAYLOAD = 2;
  static const uint8_t TCP_ERROR_TIMEOUT = 4;
  static const uint8_t TCP_ERROR_INCOMPLETE_RESPONSE = 8;

  struct ptc_error_t
  {
    uint8_t error_flags = 0;
    uint8_t ptc_error_code = 0;

    bool ok() { return !error_flags && !ptc_error_code; }
  };

  using ptc_cmd_result_t = typename nebula::util::expected<std::vector<uint8_t>, ptc_error_t>;

  PtcConnection(
    std::shared_ptr<nebula::drivers::loggers::Logger> logger, const std::string & sensor_ip,
    uint16_t sensor_port, const std::string & host_ip, uint16_t host_port)
  : logger_(logger), ctx_(new boost::asio::io_context(1)), tcp_driver_(ctx_)
  {
    tcp_driver_.init_socket(sensor_ip, sensor_port, host_ip, host_port);
    if (!tcp_driver_.open()) {
      tcp_driver_.closeSync();
      throw std::runtime_error("Could not open TCP connection!");
    }
  }

  template <typename T>
  T get(const uint8_t command_id, const std::vector<uint8_t> & payload = {})
  {
    auto response_or_err = sendReceive(command_id, payload);
    auto response =
      response_or_err.value_or_throw(prettyPrintPTCError(response_or_err.error_or({})));
    return checkSizeAndParse<T>(response);
  }

  void set(const uint8_t command_id, const std::vector<uint8_t> & payload)
  {
    auto response_or_err = sendReceive(command_id, payload);
    response_or_err.value_or_throw(prettyPrintPTCError(response_or_err.error_or({})));
  }

private:
  ptc_cmd_result_t sendReceive(const uint8_t command_id, const std::vector<uint8_t> & payload)
  {
    std::lock_guard lock(mtx_inflight_tcp_request_);

    uint32_t len = payload.size();

    std::vector<uint8_t> send_buf;
    send_buf.emplace_back(PTC_COMMAND_HEADER_HIGH);
    send_buf.emplace_back(PTC_COMMAND_HEADER_LOW);
    send_buf.emplace_back(command_id);
    send_buf.emplace_back(PTC_COMMAND_DUMMY_BYTE);
    send_buf.emplace_back((len >> 24) & 0xff);
    send_buf.emplace_back((len >> 16) & 0xff);
    send_buf.emplace_back((len >> 8) & 0xff);
    send_buf.emplace_back(len & 0xff);
    send_buf.insert(send_buf.end(), payload.begin(), payload.end());

    // These are shared_ptrs so that in case of request timeout, the callback (if ever called) can
    // access valid memory
    auto recv_buf = std::make_shared<std::vector<uint8_t>>();
    auto response_complete = std::make_shared<bool>(false);

    auto error_code = std::make_shared<ptc_error_t>();

    std::stringstream ss;
    ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(command_id)
       << " (" << len << ") ";
    std::string log_tag = ss.str();

    logger_->debug(log_tag + "Entering lock");

    std::timed_mutex tm;
    tm.lock();

    if (tcp_driver_.GetIOContext()->stopped()) {
      logger_->debug(log_tag + "IOContext was stopped");
      tcp_driver_.GetIOContext()->restart();
    }

    logger_->debug(log_tag + "Sending payload");
    tcp_driver_.asyncSendReceiveHeaderPayload(
      send_buf,
      [this, log_tag, command_id, response_complete,
       error_code](const std::vector<uint8_t> & header_bytes) {
        error_code->ptc_error_code = header_bytes[3];

        size_t payload_len = (header_bytes[4] << 24) | (header_bytes[5] << 16) |
                             (header_bytes[6] << 8) | header_bytes[7];
        logger_->debug(
          log_tag + "Received header (expecting " + std::to_string(payload_len) + "B payload)");
        // If command_id in the response does not match, we got a response for another command (or
        // rubbish), probably as a result of too many simultaneous TCP connections to the sensor
        // (e.g. from GUI, Web UI, another nebula instance, etc.)
        if (header_bytes[2] != command_id) {
          error_code->error_flags |= TCP_ERROR_UNRELATED_RESPONSE;
        }
        if (payload_len == 0) {
          *response_complete = true;
        }
      },
      [this, log_tag, recv_buf, response_complete,
       error_code](const std::vector<uint8_t> & payload_bytes) {
        logger_->debug(log_tag + "Received payload");

        // Header had payload length 0 (thus, header callback processed request successfully
        // already), but we still received a payload: invalid state
        if (*response_complete == true) {
          error_code->error_flags |= TCP_ERROR_UNEXPECTED_PAYLOAD;
          return;
        }

        // Skip 8 header bytes
        recv_buf->insert(recv_buf->end(), std::next(payload_bytes.begin(), 8), payload_bytes.end());
        *response_complete = true;
      },
      [this, log_tag, &tm]() {
        logger_->debug(log_tag + "Unlocking mutex");
        tm.unlock();
        logger_->debug(log_tag + "Unlocked mutex");
      });

    ctx_->run();

    if (!tm.try_lock_for(std::chrono::seconds(1))) {
      logger_->error(log_tag + "Request did not finish within 1s");
      error_code->error_flags |= TCP_ERROR_TIMEOUT;
      return *error_code;
    }

    if (!response_complete) {
      logger_->error(log_tag + "Did not receive response");
      error_code->error_flags |= TCP_ERROR_INCOMPLETE_RESPONSE;
      return *error_code;
    }

    if (!error_code->ok()) {
      return *error_code;
    }

    logger_->debug(log_tag + "Received response");

    return *recv_buf;
  }

  /// @brief Convert an error code to a human-readable string
  /// @param error_code The error code, containing the sensor's error code (if any), along with
  /// flags such as TCP_ERROR_UNRELATED_RESPONSE etc.
  /// @return A string description of all errors in this code
  std::string prettyPrintPTCError(ptc_error_t error_code)
  {
    if (error_code.ok()) {
      return "No error";
    }

    auto ptc_error = error_code.ptc_error_code;
    auto error_flags = error_code.error_flags;
    std::stringstream ss;

    if (ptc_error) {
      ss << "Sensor error: 0x" << std::setfill('0') << std::setw(2) << std::hex
         << static_cast<int>(ptc_error) << ' ';
    }

    switch (ptc_error) {
      case PTC_ERROR_CODE_NO_ERROR:
        break;
      case PTC_ERROR_CODE_INVALID_INPUT_PARAM:
        ss << "Invalid input parameter";
        break;
      case PTC_ERROR_CODE_SERVER_CONN_FAILED:
        ss << "Failure to connect to server";
        break;
      case PTC_ERROR_CODE_INVALID_DATA:
        ss << "No valid data returned";
        break;
      case PTC_ERROR_CODE_OUT_OF_MEMORY:
        ss << "Server does not have enough memory";
        break;
      case PTC_ERROR_CODE_UNSUPPORTED_CMD:
        ss << "Server does not support this command yet";
        break;
      case PTC_ERROR_CODE_FPGA_COMM_FAILED:
        ss << "Server failed to communicate with FPGA";
        break;
      case PTC_ERROR_CODE_OTHER:
        ss << "Unspecified internal error";
        break;
      default:
        ss << "Unknown error";
        break;
    }

    if (!error_flags) {
      return ss.str();
    }

    if (ptc_error) {
      ss << ", ";
    }

    ss << "Communication error: ";
    std::vector<std::string> nebula_errors;

    if (error_flags & TCP_ERROR_INCOMPLETE_RESPONSE) {
      nebula_errors.push_back("Incomplete response payload");
    }
    if (error_flags & TCP_ERROR_TIMEOUT) {
      nebula_errors.push_back("Request timeout");
    }
    if (error_flags & TCP_ERROR_UNEXPECTED_PAYLOAD) {
      nebula_errors.push_back("Received payload but expected payload length 0");
    }
    if (error_flags & TCP_ERROR_UNRELATED_RESPONSE) {
      nebula_errors.push_back("Received unrelated response");
    }

    ss << boost::algorithm::join(nebula_errors, ", ");

    return ss.str();
  }

  /// @brief Checks if the data size matches that of the struct to be parsed, and parses the struct.
  /// If data is too small, a std::runtime_error is thrown. If data is too large, a warning is
  /// printed and the struct is parsed with the first sizeof(T) bytes.
  template <typename T>
  T checkSizeAndParse(const std::vector<uint8_t> & data)
  {
    if (data.size() < sizeof(T)) {
      throw std::runtime_error("Attempted to parse too-small payload");
    } else if (data.size() > sizeof(T)) {
      logger_->error("Sensor returned longer payload than expected. Will parse anyway.");
    }

    T parsed;
    memcpy(&parsed, data.data(), sizeof(T));
    return parsed;
  }

  const uint8_t PTC_COMMAND_DUMMY_BYTE = 0x00;
  const uint8_t PTC_COMMAND_HEADER_HIGH = 0x47;
  const uint8_t PTC_COMMAND_HEADER_LOW = 0x74;

  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;

  std::shared_ptr<boost::asio::io_context> ctx_;
  ::drivers::tcp_driver::TcpDriver tcp_driver_;

  std::mutex mtx_inflight_tcp_request_;
};

}  // namespace connections
}  // namespace drivers
}  // namespace nebula
