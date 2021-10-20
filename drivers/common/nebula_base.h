#ifndef NEBULA_DRIVER_NEBULA_BASE_H
#define NEBULA_DRIVER_NEBULA_BASE_H

namespace nebula
{
namespace drivers
{
class NebulaBaseDriver
{
  NebulaDriverBase(const NebulaDriverBase&) = delete;
  NebulaDriverBase & operator=(const NebulaDriverBase&) = delete;

  public:
  virtual ~NebulaDriverBase() {}

  virtual bool SetConfiguration() = 0;
  virtual bool GetConfiguration() const = 0;

  virtual void SetCalibration() = 0;
  virtual void GetCalibration() = 0;

  virtual void StartHwRxInterface() = 0;
  virtual void StopHwRxInterface() = 0;

  virtual int ParsePacket() = 0;
  virtual void* GenerateCloud() = 0;

  virtual bool Initialize();
  virtual LidarStatus GetLidarStatus();

  virtual void SetPublishLidarDataCallback(void*);
  virtual DiagnosticsStatus GetDiagnosticsStatus();

};

}  // namespace drivers
}  // namespace nebula
#endif  //NEBULA_DRIVER_NEBULA_BASE_H
