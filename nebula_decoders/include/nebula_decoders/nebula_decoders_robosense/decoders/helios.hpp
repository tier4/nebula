#pragma once

#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_sensor.hpp"

#include "boost/endian/buffers.hpp"

#include <cstddef>
#include <cstdint>

namespace nebula
{
namespace drivers
{
namespace robosense_packet
{

#pragma pack(push, 1)

struct Timestamp
{
  boost::endian::big_uint48_buf_t seconds;
  boost::endian::big_uint32_buf_t nanoseconds;

  uint64_t get_time_in_ns() const
  {
    uint64_t total_nanoseconds = static_cast<uint64_t>(seconds.value()) * 1000000000ULL +
                                 static_cast<uint64_t>(nanoseconds.value());

    return total_nanoseconds;
  }
};

struct Header
{
  boost::endian::big_uint32_buf_t header_id;
  boost::endian::big_uint16_buf_t protocol_version;
  boost::endian::big_uint16_buf_t reserved_first;
  boost::endian::big_uint32_buf_t top_packet_count;
  boost::endian::big_uint32_buf_t bottom_packet_count;
  boost::endian::big_uint8_buf_t reserved_second;
  boost::endian::big_uint8_buf_t range_resolution;
  boost::endian::big_uint16_buf_t angle_interval_count;
  Timestamp timestamp;
  boost::endian::big_uint8_buf_t reserved_third;
  boost::endian::big_uint8_buf_t lidar_type;
  boost::endian::big_uint8_buf_t lidar_model;
  boost::endian::big_uint8_buf_t reserved_fourth[9];
};

struct PacketHelios : public PacketBase<12, 32, 2, 100>
{
  typedef Body<Block<Unit, PacketHelios::N_CHANNELS>, PacketHelios::N_BLOCKS> body_t;
  Header header;
  body_t body;
  boost::endian::big_uint48_buf_t tail;
};

struct InfoPacketHelios : public InfoPacketBase
{
  boost::endian::big_uint64_buf_t header;
  boost::endian::big_uint16_buf_t motor_speed;
  Ethernet ethernet;
  FovSetting fov_setting;
  boost::endian::big_uint16_buf_t reserved_first;
  boost::endian::big_uint16_buf_t phase_lock;
  boost::endian::big_uint40_buf_t top_firmware_version;
  boost::endian::big_uint40_buf_t bottom_firmware_version;
  boost::endian::big_uint40_buf_t bottom_software_version;
  boost::endian::big_uint40_buf_t motor_firmware_version;
  boost::endian::big_uint24_buf_t sensor_hw_version;
  boost::endian::big_uint32_buf_t web_page_version;
  boost::endian::big_uint32_buf_t top_backup_crc;
  boost::endian::big_uint32_buf_t bottom_backup_crc;
  boost::endian::big_uint32_buf_t software_backup_crc;
  boost::endian::big_uint32_buf_t webpage_backup_crc;
  boost::endian::big_uint32_buf_t ethernet_gateway;
  boost::endian::big_uint32_buf_t subnet_mask;
  uint8_t reserved_second[201];
  boost::endian::big_uint48_buf_t serial_number;
  boost::endian::big_uint16_buf_t zero_angle_offset;
  boost::endian::big_uint8_buf_t return_mode;
  boost::endian::big_uint8_buf_t time_sync_mode;
  boost::endian::big_uint8_buf_t sync_status;
  Timestamp time;
  OperatingStatus operating_status;
  uint8_t reserved_third[17];
  FaultDiagnosis fault_diagnosis;
  boost::endian::big_uint8_buf_t code_wheel_status;
  boost::endian::big_uint8_buf_t pps_trigger_mode;
  uint8_t reserved_fourth[20];
  boost::endian::big_uint8_buf_t gprmc[86];
  CorrectedVerticalAngle corrected_vertical;
  CorrectedHorizontalAngle corrected_horizontal;
  uint8_t reserved_fifth[586];
  boost::endian::big_uint16_buf_t tail;
};

#pragma pack(pop)

}  // namespace robosense_packet

class Helios
: public RobosenseSensor<robosense_packet::PacketHelios, robosense_packet::InfoPacketHelios>
{
private:
  static constexpr int firing_time_offset_ns_single_[12][32] = {
    {0,     1570,  3150,  4720,  6300,  7870,  9450,  11360, 13260, 15170, 17080,
     18990, 20560, 22140, 23710, 25290, 26530, 29010, 27770, 30250, 31490, 32730,
     33980, 35220, 36460, 37700, 38940, 40180, 41420, 42670, 43910, 45150},
    {55560, 57130, 58700, 60280, 61850, 63430, 65000, 66910, 68820, 70730, 72640,
     74540, 76120, 77690, 79270, 80840, 82080, 84570, 83320, 85810, 87050, 89530,
     88290, 90770, 92010, 93260, 94500, 95740, 96980, 98220, 99460, 100700},
    {111110, 112690, 114260, 115840, 117410, 118980, 120560, 122470, 124380, 126280, 128190,
     130100, 131670, 133250, 134820, 136400, 137640, 140120, 138880, 141360, 142600, 145090,
     143850, 146330, 147570, 148810, 150050, 151290, 152540, 153780, 155020, 156260},
    {166670, 168240, 169820, 171390, 172970, 174540, 176110, 178020, 179930, 181840, 183750,
     185650, 187230, 188800, 190380, 191950, 193190, 195680, 194440, 196920, 198160, 200640,
     199400, 201880, 203130, 204370, 205610, 206850, 208090, 209330, 210570, 211810},
    {222220, 223800, 225370, 226950, 228520, 230100, 231670, 233580, 235490, 237390, 239300,
     241210, 242780, 244360, 245930, 247510, 248750, 251230, 249990, 252470, 253720, 256200,
     254960, 257440, 258680, 259920, 261160, 262400, 263650, 264890, 266130, 267370},
    {277780, 279350, 280930, 282500, 284080, 285650, 287230, 289130, 291040, 292950, 294860,
     296770, 298340, 299920, 301490, 303060, 304310, 306790, 305550, 308030, 309270, 311750,
     310510, 313000, 314240, 315480, 316720, 317960, 319200, 320440, 321680, 322930},
    {333330, 334910, 336480, 338060, 339630, 341210, 342780, 344690, 346600, 348510, 350410,
     352320, 353900, 355470, 357050, 358620, 359860, 362340, 361100, 363590, 364830, 367310,
     366070, 368550, 369790, 371030, 372270, 373520, 374760, 376000, 377240, 378480},
    {388890, 390460, 392040, 393610, 395190, 396760, 398340, 400240, 402150, 404060, 405970,
     407880, 409450, 411030, 412600, 414180, 415420, 417900, 416660, 419140, 420380, 422860,
     421620, 424110, 425350, 426590, 427830, 429070, 430310, 431550, 432800, 434040},
    {444440, 446020, 447590, 449170, 450740, 452320, 453890, 455800, 457710, 459620, 461520,
     463430, 465010, 466580, 468160, 469730, 470970, 473460, 472210, 474700, 475940, 478420,
     477180, 479660, 480900, 482140, 483390, 484630, 485870, 487110, 488350, 489590},
    {500000, 501570, 503150, 504720, 506300, 507870, 509450, 511360, 513260, 515170, 517080,
     518990, 520560, 522140, 523710, 525290, 526530, 529010, 527770, 530250, 531490, 533980,
     532730, 535220, 536460, 537700, 538940, 540180, 541420, 542670, 543910, 545150},
    {555560, 557130, 558700, 560280, 561850, 563430, 565000, 566910, 568820, 570730, 572640,
     574540, 576120, 577690, 579270, 580840, 582080, 584570, 583320, 585810, 587050, 589530,
     588290, 590770, 592010, 593260, 594500, 595740, 596980, 598220, 599460, 600700},
    {611110, 612690, 614260, 615840, 617410, 618980, 620560, 622470, 624380, 626280, 628190,
     630100, 631670, 633250, 634820, 636400, 637640, 640120, 638880, 641360, 642600, 645090,
     643850, 646330, 647570, 648810, 650050, 651290, 652540, 653780, 655020, 656260}};

  static constexpr int firing_time_offset_ns_dual_[12][32]{
    {0,     1570,  3150,  4720,  6300,  7870,  9450,  11360, 13260, 15170, 17080,
     18990, 20560, 22140, 23710, 25290, 26530, 27770, 29010, 30250, 31490, 32730,
     33980, 35220, 36460, 37700, 38940, 40180, 41420, 42670, 43910, 45150},
    {0,     1570,  3150,  4720,  6300,  7870,  9450,  11360, 13260, 15170, 17080,
     18990, 20560, 22140, 23710, 25290, 26530, 27770, 29010, 30250, 31490, 32730,
     33980, 35220, 36460, 37700, 38940, 40180, 41420, 42670, 43910, 45150},
    {55560, 57130, 58700, 60280, 61850, 63430, 65000, 66910, 68820, 70730, 72640,
     74540, 76120, 77690, 79270, 80840, 82080, 83320, 84570, 85810, 87050, 88290,
     89530, 90770, 92010, 93260, 94500, 95740, 96980, 98220, 99460, 100700},
    {55560, 57130, 58700, 60280, 61850, 63430, 65000, 66910, 68820, 70730, 72640,
     74540, 76120, 77690, 79270, 80840, 82080, 83320, 84570, 85810, 87050, 88290,
     89530, 90770, 92010, 93260, 94500, 95740, 96980, 98220, 99460, 100700},
    {111110, 112690, 114260, 115840, 117410, 118980, 120560, 122470, 124380, 126280, 128190,
     130100, 131670, 133250, 134820, 136400, 137640, 138880, 140120, 141360, 142600, 143850,
     145090, 146330, 147570, 148810, 150050, 151290, 152540, 153780, 155020, 156260},
    {111110, 112690, 114260, 115840, 117410, 118980, 120560, 122470, 124380, 126280, 128190,
     130100, 131670, 133250, 134820, 136400, 137640, 138880, 140120, 141360, 142600, 143850,
     145090, 146330, 147570, 148810, 150050, 151290, 152540, 153780, 155020, 156260},
    {166670, 168240, 169820, 171390, 172970, 174540, 176110, 178020, 179930, 181840, 183750,
     185650, 187230, 188800, 190380, 191950, 193190, 194440, 195680, 196920, 198160, 199400,
     200640, 201880, 203130, 204370, 205610, 206850, 208090, 209330, 210570, 211810},
    {166670, 168240, 169820, 171390, 172970, 174540, 176110, 178020, 179930, 181840, 183750,
     185650, 187230, 188800, 190380, 191950, 193190, 194440, 195680, 196920, 198160, 199400,
     200640, 201880, 203130, 204370, 205610, 206850, 208090, 209330, 210570, 211810},
    {222220, 223800, 225370, 226950, 228520, 230100, 231670, 233580, 235490, 237390, 239300,
     241210, 242780, 244360, 245930, 247510, 248750, 249990, 251230, 252470, 253720, 254960,
     256200, 257440, 258680, 259920, 261160, 262400, 263650, 264890, 266130, 267370},
    {222220, 223800, 225370, 226950, 228520, 230100, 231670, 233580, 235490, 237390, 239300,
     241210, 242780, 244360, 245930, 247510, 248750, 249990, 251230, 252470, 253720, 254960,
     256200, 257440, 258680, 259920, 261160, 262400, 263650, 264890, 266130, 267370},
    {277780, 279350, 280930, 282500, 284080, 285650, 287230, 289130, 291040, 292950, 294860,
     296770, 298340, 299920, 301490, 303060, 304310, 305550, 306790, 308030, 309270, 310510,
     311750, 313000, 314240, 315480, 316720, 317960, 319200, 320440, 321680, 322930},
    {277780, 279350, 280930, 282500, 284080, 285650, 287230, 289130, 291040, 292950, 294860,
     296770, 298340, 299920, 301490, 303060, 304310, 305550, 306790, 308030, 309270, 310510,
     311750, 313000, 314240, 315480, 316720, 317960, 319200, 320440, 321680, 322930}};

public:
  static constexpr float MIN_RANGE = 0.2f;
  static constexpr float MAX_RANGE = 150.f;
  static constexpr size_t MAX_SCAN_BUFFER_POINTS = 230400;  ///// !! Calculate this

  int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id,
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration) override
  {
    if (sensor_configuration->return_mode == ReturnMode::DUAL)
      return firing_time_offset_ns_dual_[block_id][channel_id];
    else
      return firing_time_offset_ns_single_[block_id][channel_id];
  }
};

}  // namespace drivers
}  // namespace nebula