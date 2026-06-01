#include <Arduino.h>

uint8_t start_frame[2] = {0xBB, 0xAA};

struct __attribute__((packed)) GPS_Data
{
  uint32_t timestamp = 0;                             // 時間戳 (微秒)
  int32_t latitude = 221234567;                       // 緯度 (deg * 1e7)
  int32_t longitude = 1201234567;                     // 經度 (deg * 1e7)
  int32_t altitude = 20123;                           // 高度 (mm)
  uint8_t GPSFixType = 0;                             // GPS 定位類型
  uint8_t RTKStatus = 0;                              // RTK 狀態
};
GPS_Data gpsData;

struct __attribute__((packed)) SLAM_position
{
  uint64_t timestamp = 1;                             // 時間戳 (微秒), Unix epoch 時間，從 1970-01-01 00:00:00 UTC 起算的微秒
  float pos[3] = {2.0, 3.0, 4.0};                     // 位置 (公尺), [x, y, z]
  float orientation[4] = {0.0, 0.0, 0.0, 1.0};        // 姿態四元數 (無單位), [x, y, z, w]
};
SLAM_position slamPosition;

unsigned long T_last1, T_last2 = 0;

void SLAM_receive()
{
  static uint8_t rx_buf[sizeof(SLAM_position)];
  static size_t rx_count = 0;
  static bool rx_synced = false;

  while (Serial1.available())
  {
    uint8_t b = Serial1.read();
    if (!rx_synced)
    {
      if (rx_count == 0 && b == 0xBB)
      {
        rx_count = 1;
      }
      else if (rx_count == 1 && b == 0xAA)
      {
        rx_count = 0;
        rx_synced = true;
      }
      else
      {
        rx_count = 0;
      }
    }
    else
    {
      rx_buf[rx_count++] = b;
      if (rx_count == sizeof(SLAM_position))
      {
        memcpy(&slamPosition, rx_buf, sizeof(SLAM_position));
        rx_count = 0;
        rx_synced = false;
      }
    }
  }
}

void Serial_print()
{
  Serial.print("timestamp: ");
  Serial.print(slamPosition.timestamp);
  Serial.print(" | pos_x: ");
  Serial.print(slamPosition.pos[0], 3);
  Serial.print(" | pos_y: ");
  Serial.print(slamPosition.pos[1], 3);
  Serial.print(" | pos_z: ");
  Serial.print(slamPosition.pos[2], 3);
  Serial.print(" | ori[x,y,z,w]: ");
  Serial.print(slamPosition.orientation[0], 3);
  Serial.print(" ");
  Serial.print(slamPosition.orientation[1], 3);
  Serial.print(" ");
  Serial.print(slamPosition.orientation[2], 3);
  Serial.print(" ");
  Serial.print(slamPosition.orientation[3], 3);
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop()
{
  SLAM_receive();

  if (micros() - T_last1 >= 200000)
  { // 5Hz
    T_last1 = micros();
    gpsData.timestamp = T_last1;
    Serial1.write(start_frame, 2);
    Serial1.write((uint8_t *)&gpsData, sizeof(gpsData));
  }
  if (micros() - T_last2 >= 50000)
  { // 20Hz
    T_last2 = micros();
    Serial_print();
  }
}
