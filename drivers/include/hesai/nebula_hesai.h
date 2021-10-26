#ifndef NEBULA_DRIVERS_NEBULA_HESAI_H
#define NEBULA_DRIVERS_NEBULA_HESAI_H

include "HwInterface/udp_socket.hpp"

namespace nebula
{
namespace drivers
{

class NebulaHesaiDriver : NebulaBaseDriver
{
private:
  HwInterface::UdpSocket cloud_socket_;
  HwInterface::UdpSocket gps_socket_;

public:

  bool SetConfiguration() override;
  bool GetConfiguration() override;

  void SetCalibration() override;
  void GetCalibration() override;

  void StartHwRxInterface() override;
  void StopHwRxInterface() override;

  int ParsePacket() override;
  void* GenerateCloud() override;

  bool Initialize() override;
  LidarStatus GetLidarStatus() override;

  void SetPublishLidarDataCallback(void*) override;
  DiagnosticsStatus GetDiagnosticsStatus() override;
};

}
}

#endif  //NEBULA_DRIVERS_NEBULA_HESAI_H
