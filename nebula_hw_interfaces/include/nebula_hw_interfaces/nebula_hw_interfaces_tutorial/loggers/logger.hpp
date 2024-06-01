#pragma once

#include <string>
#include <memory>

namespace nebula
{
namespace drivers
{
namespace loggers
{

class Logger
{
public:
  virtual void debug(const std::string & message) = 0;
  virtual void info(const std::string & message) = 0;
  virtual void warn(const std::string & message) = 0;
  virtual void error(const std::string & message) = 0;

  virtual std::shared_ptr<Logger> child(const std::string & name) = 0;
};

}  // namespace loggers
}  // namespace drivers
}  // namespace nebula
