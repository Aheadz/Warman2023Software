#include <stdint.h>

struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};

class IMU 
{
public:
    IMU();
    ~IMU();
    void init_imu();
    void calibrate_imu();
    void getYawPitchRoll(double& y, double& p,double& r);

private:
    void updateBiasStoreSum(biasStore *store);
    bool isBiasStoreValid(biasStore *store);
    void printBiases(biasStore *store);
};