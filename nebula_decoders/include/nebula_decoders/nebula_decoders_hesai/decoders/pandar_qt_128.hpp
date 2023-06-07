#pragma once
/**
 * Pandar QT128
 */
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>

namespace nebula
{
namespace drivers
{
namespace pandar_qt_128
{
constexpr uint16_t MAX_AZIMUTH_STEPS = 900;  // High Res mode
// constexpr float DISTANCE_UNIT = 0.004f;       // 4mm
constexpr double MIN_RANGE = 0.05;
constexpr double MAX_RANGE = 50.0;

// Head
constexpr size_t HEAD_SIZE = 12;
constexpr size_t PRE_HEADER_SIZE = 6;
constexpr size_t HEADER_SIZE = 6;
// Body
constexpr size_t BLOCKS_PER_PACKET = 2;
constexpr size_t BLOCK_HEADER_AZIMUTH = 2;
constexpr size_t LASER_COUNT = 128;
constexpr size_t UNIT_SIZE = 4;
constexpr size_t CRC_SIZE = 4;
constexpr size_t BLOCK_SIZE = UNIT_SIZE * LASER_COUNT + BLOCK_HEADER_AZIMUTH;
constexpr size_t BODY_SIZE = BLOCK_SIZE * BLOCKS_PER_PACKET + CRC_SIZE;
// Functional Safety
constexpr size_t FS_VERSION_SIZE = 1;
constexpr size_t LIDAR_STATE_SIZE = 1;
constexpr size_t FAULT_SIZE = 1;
constexpr size_t OUT_FAULT_SIZE = 2;
constexpr size_t RESERVED_SIZE = 8;
constexpr size_t CRC2_SIZE = 4;
constexpr size_t PACKET_FS_SIZE = 17;
// Tail
constexpr size_t RESERVED2_SIZE = 5;
constexpr size_t MODE_FLAG_SIZE = 1;
constexpr size_t RESERVED3_SIZE = 6;
constexpr size_t RETURN_MODE_SIZE = 1;
constexpr size_t MOTOR_SPEED_SIZE = 2;
constexpr size_t UTC_SIZE = 6;
constexpr size_t TIMESTAMP_SIZE = 4;
constexpr size_t FACTORY_SIZE = 1;
constexpr size_t SEQUENCE_SIZE = 4;
constexpr size_t CRC3_SIZE = 4;
constexpr size_t PACKET_TAIL_SIZE = 34;
constexpr size_t PACKET_TAIL_WITHOUT_UDP_SEQ_CRC_SIZE = 26;

// Cyber Security
constexpr size_t SIGNATURE_SIZE = 32;

constexpr size_t SKIP_SIZE = CRC_SIZE + PACKET_FS_SIZE + RESERVED2_SIZE;

// All
constexpr size_t PACKET_SIZE =
  HEAD_SIZE + BODY_SIZE + PACKET_FS_SIZE + PACKET_TAIL_SIZE + SIGNATURE_SIZE;
constexpr size_t PACKET_WITHOUT_UDP_SEQ_CRC_SIZE =
  HEAD_SIZE + BODY_SIZE + PACKET_FS_SIZE + PACKET_TAIL_WITHOUT_UDP_SEQ_CRC_SIZE;

constexpr uint32_t SINGLE_FIRST_RETURN = 0x33;
constexpr uint32_t SINGLE_SECOND_RETURN = 0x34;
constexpr uint32_t SINGLE_STRONGEST_RETURN = 0x37;
constexpr uint32_t SINGLE_LAST_RETURN = 0x38;
constexpr uint32_t DUAL_LAST_STRONGEST_RETURN = 0x39;
constexpr uint32_t DUAL_FIRST_LAST_RETURN = 0x3B;
constexpr uint32_t DUAL_FIRST_STRONGEST_RETURN = 0x3C;
constexpr uint32_t DUAL_STRONGEST_2ndSTRONGEST_RETURN = 0x3E;
constexpr uint32_t DUAL_FIRST_SECOND_RETURN = 0x3A;

struct Header
{
  uint16_t sob;             // 0xFFEE 2bytes
  uint8_t chProtocolMajor;  // Protocol Version Major 1byte
  uint8_t chProtocolMinor;  // Protocol Version Minor 1byte
  uint8_t chLaserNumber;    // laser number 1byte
  uint8_t chBlockNumber;    // block number 1byte
  uint8_t chDisUnit;        // Distance unit, 4mm
  uint8_t chReturnType;     // return mode 1 byte  when dual return 0-Single Return
                            // 1-The first block is the 1 st return.
                            // 2-The first block is the 2 nd return
  uint8_t chFlags;          // [6] channel customization: 1-Selected channels, 0-All channels
                            // [3] digital signature: 1-YES, 0-NO
                            // [2] functional safety: 1-YES, 0-NO
                            // [1] IMU: 1-YES, 0-NO
                            // [0] UDP sequence: 1-YES, 0-NO
};

struct Unit
{
  float distance;
  uint16_t intensity;
  uint16_t confidence;
};

struct Block
{
  uint16_t azimuth;  // packet angle,Azimuth = RealAzimuth * 100
  Unit units[LASER_COUNT];
};

struct Packet
{
  Header header;
  Block blocks[BLOCKS_PER_PACKET];
  uint32_t usec;  // ms
  uint32_t mode_flag;
  uint32_t return_mode;
  tm t;
};
const std::string PandarQT128_TL1 = R"(33,27.656
34,53
35,2.312
36,78.344
37,81.512
38,5.48
39,56.168
40,30.824
41,33.992
42,59.336
43,8.648
44,84.68
45,87.848
46,11.816
47,62.504
48,37.16
49,40.328
50,65.672
51,14.984
52,91.016
53,94.184
54,18.152
55,68.84
56,43.496
57,46.664
58,72.008
59,21.32
60,97.352
61,100.52
62,24.488
63,75.176
64,49.832
65,1.456
66,77.488
67,26.8
68,52.144
69,55.312
70,29.968
71,80.656
72,4.624
73,7.792
74,83.824
75,33.136
76,58.48
77,61.648
78,36.304
79,86.992
80,10.96
81,14.128
82,90.16
83,39.472
84,64.816
85,67.984
86,42.64
87,93.328
88,17.296
89,20.464
90,96.496
91,45.808
92,71.152
93,74.32
94,48.976
95,99.664
96,23.632
97,25.944
98,51.288
99,0.6
100,76.632
101,79.8
102,3.768
103,54.456
104,29.112
105,32.28
106,57.624
107,6.936
108,82.968
109,86.136
110,10.104
111,60.792
112,35.448
113,38.616
114,63.96
115,13.272
116,89.304
117,92.472
118,16.44
119,67.128
120,41.784
121,44.952
122,70.296
123,19.608
124,95.64
125,98.808
126,22.776
127,73.464
128,48.12)";
const std::string PandarQT128_TL2 = R"(1,2.312
2,78.344
3,27.656
4,53
5,56.168
6,30.824
7,81.512
8,5.48
9,8.648
10,84.68
11,33.992
12,59.336
13,62.504
14,37.16
15,87.848
16,11.816
17,14.984
18,91.016
19,40.328
20,65.672
21,68.84
22,43.496
23,94.184
24,18.152
25,21.32
26,97.352
27,46.664
28,72.008
29,75.176
30,49.832
31,100.52
32,24.488
65,0.6
66,76.632
67,25.944
68,51.288
69,54.456
70,29.112
71,79.8
72,3.768
73,6.936
74,82.968
75,32.28
76,57.624
77,60.792
78,35.448
79,86.136
80,10.104
81,13.272
82,89.304
83,38.616
84,63.96
85,67.128
86,41.784
87,92.472
88,16.44
89,19.608
90,95.64
91,44.952
92,70.296
93,73.464
94,48.12
95,98.808
96,22.776
97,26.8
98,52.144
99,1.456
100,77.488
101,80.656
102,4.624
103,55.312
104,29.968
105,33.136
106,58.48
107,7.792
108,83.824
109,86.992
110,10.96
111,61.648
112,36.304
113,39.472
114,64.816
115,14.128
116,90.16
117,93.328
118,17.296
119,67.984
120,42.64
121,45.808
122,71.152
123,20.464
124,96.496
125,99.664
126,23.632
127,74.32
128,48.976)";
}  // namespace pandar_qt_128
}  // namespace drivers
}  // namespace nebula
