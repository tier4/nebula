#ifndef NEBULA_DRIVER_BASE_H
#define NEBULA_DRIVER_BASE_H

namespace nebula
{
namespace drivers
{
class NebulaBaseDriver
  {
  NebulaDriverBase(const NebulaDriverBase&) = delete;
  NebulaDriverBase & operator=(const NebulaDriverBase&) = delete;

  private:
    // Lidar specific conversion of LidarScan msg to Pointcloud2 msg
    virtual void GenerateCloud();
    virtual void LidarScanCallback();
    // Subscriber to lidarscan
    // On construction wait for lidarscan msg

  public:
    virtual bool SetCalibrationConfiguration() = 0;
    virtual bool SetCloudConfiguration() = 0;
    virtual void GetCloudConfiguration();
  };

}  // namespace drivers
}  // namespace nebula
#endif  //NEBULA_DRIVER_BASE_H
