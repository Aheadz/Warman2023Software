#include "IMU.h"
#include "ICM_20948.h"
#include <Wire.h>
#include <EEPROM.h>


namespace ICMPU {
    ICM_20948_I2C myICM;
};

IMU::IMU(){}
IMU::~IMU(){}

void IMU::init_imu()
{
  Serial.println("Initializing IMU....");
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    ICMPU::myICM.begin(Wire, 1);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(ICMPU::myICM.statusString());
    if (ICMPU::myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  Serial.println(F("Device connected!"));
  bool success = true; // Use success to show if the DMP configuration was successful
  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (ICMPU::myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (ICMPU::myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (ICMPU::myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
  success &= (ICMPU::myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (ICMPU::myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (ICMPU::myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (ICMPU::myICM.resetFIFO() == ICM_20948_Stat_Ok);
  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }

  biasStore store;

  EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
  if (isBiasStoreValid(&store))
  {
    Serial.println(F("Bias data in EEPROM is valid. Restoring it..."));
    success &= (ICMPU::myICM.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (ICMPU::myICM.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      Serial.println(F("Biases restored."));
      printBiases(&store);
    }
    else
    {
      Serial.println(F("Bias restore failed!"));
    }
  }
}


void IMU::calibrate_imu(){
  static unsigned long startTime = millis(); // Save the biases when the code has been running for two minutes
  static bool biasesStored = false;
  while (!biasesStored){
    icm_20948_DMP_data_t data;
    ICMPU::myICM.readDMPdataFromFIFO(&data);

    //Output While Calibrating
    if ((ICMPU::myICM.status == ICM_20948_Stat_Ok) || (ICMPU::myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
      if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
      {
        // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
        // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
        // The quaternion data is scaled by 2^30.
        // Scale to +/- 1
        double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        double q2sqr = q2 * q2;

        // roll (x-axis rotation)
        double t0 = +2.0 * (q0 * q1 + q2 * q3);
        double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
        double roll = atan2(t0, t1) * 180.0 / PI;

        // pitch (y-axis rotation)
        double t2 = +2.0 * (q0 * q2 - q3 * q1);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        double pitch = asin(t2) * 180.0 / PI;

        // yaw (z-axis rotation)
        double t3 = +2.0 * (q0 * q3 + q1 * q2);
        double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
        double yaw = atan2(t3, t4) * 180.0 / PI;
        Serial.print(F("Roll: "));
        Serial.print(roll, 1);
        Serial.print(F("\tPitch: "));
        Serial.print(pitch, 1);
        Serial.print(F("\tYaw: "));
        Serial.println(yaw, 1);
      }
    }

    if (ICMPU::myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
      if (!biasesStored) // Should we store the biases?
      {
        if (millis() > (startTime + 120000)) // Is it time to store the biases?
        {
          Serial.println(F("\r\n\r\n\r\nSaving bias data..."));

          biasStore store;
    
          bool success = (ICMPU::myICM.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
          success &= (ICMPU::myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

          updateBiasStoreSum(&store);
        
          if (success)
          {
            biasesStored = true; // Only attempt this once
          
            EEPROM.put(0, store); // Write biases to EEPROM, starting at address 0
            EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
            if (isBiasStoreValid(&store))
            {
              Serial.println(F("Biases stored."));
              printBiases(&store);
              Serial.println(F("\r\n\r\n\r\n"));
            }
            else
              Serial.println(F("Bias store failed!\r\n\r\n\r\n"));
          }
          else
          {
            Serial.println(F("Bias read failed!\r\n\r\n\r\n"));
          }
        }
      }
      delay(10);
    }
  }
}

void IMU::updateBiasStoreSum(biasStore *store) // Update the bias store checksum
{
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

bool IMU::isBiasStoreValid(biasStore *store) // Returns true if the header and checksum are valid
{
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}

void IMU::printBiases(biasStore *store)
{
  Serial.print(F("Gyro X: "));
  Serial.print(store->biasGyroX);
  Serial.print(F(" Gyro Y: "));
  Serial.print(store->biasGyroY);
  Serial.print(F(" Gyro Z: "));
  Serial.println(store->biasGyroZ);
  Serial.print(F("Accel X: "));
  Serial.print(store->biasAccelX);
  Serial.print(F(" Accel Y: "));
  Serial.print(store->biasAccelY);
  Serial.print(F(" Accel Z: "));
  Serial.println(store->biasAccelZ);
  Serial.print(F("CPass X: "));
  Serial.print(store->biasCPassX);
  Serial.print(F(" CPass Y: "));
  Serial.print(store->biasCPassY);
  Serial.print(F(" CPass Z: "));
  Serial.println(store->biasCPassZ);
}

void IMU::getYawPitchRoll(double& y, double& p,double& r)
{
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  ICMPU::myICM.readDMPdataFromFIFO(&data);

  if ((ICMPU::myICM.status == ICM_20948_Stat_Ok) || (ICMPU::myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    // Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    // if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
    // if ( data.header < 0x100) Serial.print( "0" );
    // if ( data.header < 0x10) Serial.print( "0" );
    // Serial.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.
      // Serial.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);
      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      double q2sqr = q2 * q2;
      

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;
      
      y = yaw;
      p = pitch;
      r = roll;

      // Serial.print(y,3);
      // Serial.print(",");
      // Serial.print(p,3);
      // Serial.print(",");
      // Serial.print(r,3);
      // Serial.println("");

      // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
      // Serial.print(F("{\"quat_w\":"));
      // Serial.print(q0, 3);
      // Serial.print(F(", \"quat_x\":"));
      // Serial.print(q1, 3);
      // Serial.print(F(", \"quat_y\":"));
      // Serial.print(q2, 3);
      // Serial.print(F(", \"quat_z\":"));
      // Serial.print(q3, 3);
      // Serial.println(F("}"));

      // Serial.print("Yaw: ");
      // Serial.print(y,3);
      // Serial.print(" ");
      // Serial.print("Pitch: ");
      // Serial.print(p,3);
      // Serial.print(" ");
      // Serial.print("Roll: ");
      // Serial.print(r,3);
      // Serial.println("");

    }
  }
  if (ICMPU::myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}