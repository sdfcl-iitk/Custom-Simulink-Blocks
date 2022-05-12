#pragma once

#include <cmath>
#include <cstdint>

extern "C" {
    #include <unistd.h>
    #include <fcntl.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
    #include <sys/ioctl.h>
}

class MPU9250_driver {

    /* -------------------------------------
        MPU9250 Registers
       -------------------------------------
    */
    
    //Magnetometer Registers
    // static constexpr uint8_t AK8963_ADDRESS   = 0x0C;
    static constexpr uint8_t AK8963_WHO_AM_I  = 0x00; // should return = 0x48;
    static constexpr uint8_t AK8963_INFO      = 0x01;
    static constexpr uint8_t AK8963_ST1       = 0x02;  // data ready status bit 0
    static constexpr uint8_t AK8963_XOUT_L	 = 0x03;  // data
    static constexpr uint8_t AK8963_XOUT_H	 = 0x04;
    static constexpr uint8_t AK8963_YOUT_L	 = 0x05;
    static constexpr uint8_t AK8963_YOUT_H	 = 0x06;
    static constexpr uint8_t AK8963_ZOUT_L	 = 0x07;
    static constexpr uint8_t AK8963_ZOUT_H	 = 0x08;
    static constexpr uint8_t AK8963_ST2       = 0x09;  // Data overflow bit 3 and data read error status bit 2
    static constexpr uint8_t AK8963_CNTL      = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
    static constexpr uint8_t AK8963_ASTC      = 0x0C;  // Self test control
    static constexpr uint8_t AK8963_I2CDIS    = 0x0F;  // I2C disable
    static constexpr uint8_t AK8963_ASAX      = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
    static constexpr uint8_t AK8963_ASAY      = 0x11;  // Fuse ROM y-axis sensitivity adjustment value
    static constexpr uint8_t AK8963_ASAZ      = 0x12;  // Fuse ROM z-axis sensitivity adjustment value

    static constexpr uint8_t SELF_TEST_X_GYRO = 0x00;
    static constexpr uint8_t SELF_TEST_Y_GYRO = 0x01;
    static constexpr uint8_t SELF_TEST_Z_GYRO = 0x02;

    // static constexpr uint8_t X_FINE_GAIN      = 0x03; // [7:0] fine gain
    // static constexpr uint8_t Y_FINE_GAIN      = 0x04;
    // static constexpr uint8_t Z_FINE_GAIN      = 0x05;
    // static constexpr uint8_t XA_OFFSET_H      = 0x06; // User-defined trim values for accelerometer
    // static constexpr uint8_t XA_OFFSET_L_TC   = 0x07;
    // static constexpr uint8_t YA_OFFSET_H      = 0x08;
    // static constexpr uint8_t YA_OFFSET_L_TC   = 0x09;
    // static constexpr uint8_t ZA_OFFSET_H      = 0x0A;
    // static constexpr uint8_t ZA_OFFSET_L_TC   = 0x0B;

    static constexpr uint8_t SELF_TEST_X_ACCEL = 0x0D;
    static constexpr uint8_t SELF_TEST_Y_ACCEL = 0x0E;
    static constexpr uint8_t SELF_TEST_Z_ACCEL = 0x0F;

    static constexpr uint8_t SELF_TEST_A      = 0x10;

    static constexpr uint8_t XG_OFFSET_H      = 0x13;  // User-defined trim values for gyroscope
    static constexpr uint8_t XG_OFFSET_L      = 0x14;
    static constexpr uint8_t YG_OFFSET_H      = 0x15;
    static constexpr uint8_t YG_OFFSET_L      = 0x16;
    static constexpr uint8_t ZG_OFFSET_H      = 0x17;
    static constexpr uint8_t ZG_OFFSET_L      = 0x18;
    static constexpr uint8_t SMPLRT_DIV       = 0x19;
    static constexpr uint8_t MPU_CONFIG           = 0x1A;
    static constexpr uint8_t GYRO_CONFIG      = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG     = 0x1C;
    static constexpr uint8_t ACCEL_CONFIG2    = 0x1D;
    static constexpr uint8_t LP_ACCEL_ODR     = 0x1E;
    static constexpr uint8_t WOM_THR          = 0x1F;

    static constexpr uint8_t MOT_DUR          = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
    static constexpr uint8_t ZMOT_THR         = 0x21;  // Zero-motion detection threshold bits [7:0]
    static constexpr uint8_t ZRMOT_DUR        = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

    static constexpr uint8_t FIFO_EN          = 0x23;
    static constexpr uint8_t I2C_MST_CTRL     = 0x24;
    static constexpr uint8_t I2C_SLV0_ADDR    = 0x25;
    static constexpr uint8_t I2C_SLV0_REG     = 0x26;
    static constexpr uint8_t I2C_SLV0_CTRL    = 0x27;
    static constexpr uint8_t I2C_SLV1_ADDR    = 0x28;
    static constexpr uint8_t I2C_SLV1_REG     = 0x29;
    static constexpr uint8_t I2C_SLV1_CTRL    = 0x2A;
    static constexpr uint8_t I2C_SLV2_ADDR    = 0x2B;
    static constexpr uint8_t I2C_SLV2_REG     = 0x2C;
    static constexpr uint8_t I2C_SLV2_CTRL    = 0x2D;
    static constexpr uint8_t I2C_SLV3_ADDR    = 0x2E;
    static constexpr uint8_t I2C_SLV3_REG     = 0x2F;
    static constexpr uint8_t I2C_SLV3_CTRL    = 0x30;
    static constexpr uint8_t I2C_SLV4_ADDR    = 0x31;
    static constexpr uint8_t I2C_SLV4_REG     = 0x32;
    static constexpr uint8_t I2C_SLV4_DO      = 0x33;
    static constexpr uint8_t I2C_SLV4_CTRL    = 0x34;
    static constexpr uint8_t I2C_SLV4_DI      = 0x35;
    static constexpr uint8_t I2C_MST_STATUS   = 0x36;
    static constexpr uint8_t INT_PIN_CFG      = 0x37;
    static constexpr uint8_t INT_ENABLE       = 0x38;
    static constexpr uint8_t DMP_INT_STATUS   = 0x39;  // Check DMP interrupt
    static constexpr uint8_t INT_STATUS       = 0x3A;
    static constexpr uint8_t ACCEL_XOUT_H     = 0x3B;
    static constexpr uint8_t ACCEL_XOUT_L     = 0x3C;
    static constexpr uint8_t ACCEL_YOUT_H     = 0x3D;
    static constexpr uint8_t ACCEL_YOUT_L     = 0x3E;
    static constexpr uint8_t ACCEL_ZOUT_H     = 0x3F;
    static constexpr uint8_t ACCEL_ZOUT_L     = 0x40;
    static constexpr uint8_t TEMP_OUT_H       = 0x41;
    static constexpr uint8_t TEMP_OUT_L       = 0x42;
    static constexpr uint8_t GYRO_XOUT_H      = 0x43;
    static constexpr uint8_t GYRO_XOUT_L      = 0x44;
    static constexpr uint8_t GYRO_YOUT_H      = 0x45;
    static constexpr uint8_t GYRO_YOUT_L      = 0x46;
    static constexpr uint8_t GYRO_ZOUT_H      = 0x47;
    static constexpr uint8_t GYRO_ZOUT_L      = 0x48;
    static constexpr uint8_t EXT_SENS_DATA_00 = 0x49;
    static constexpr uint8_t EXT_SENS_DATA_01 = 0x4A;
    static constexpr uint8_t EXT_SENS_DATA_02 = 0x4B;
    static constexpr uint8_t EXT_SENS_DATA_03 = 0x4C;
    static constexpr uint8_t EXT_SENS_DATA_04 = 0x4D;
    static constexpr uint8_t EXT_SENS_DATA_05 = 0x4E;
    static constexpr uint8_t EXT_SENS_DATA_06 = 0x4F;
    static constexpr uint8_t EXT_SENS_DATA_07 = 0x50;
    static constexpr uint8_t EXT_SENS_DATA_08 = 0x51;
    static constexpr uint8_t EXT_SENS_DATA_09 = 0x52;
    static constexpr uint8_t EXT_SENS_DATA_10 = 0x53;
    static constexpr uint8_t EXT_SENS_DATA_11 = 0x54;
    static constexpr uint8_t EXT_SENS_DATA_12 = 0x55;
    static constexpr uint8_t EXT_SENS_DATA_13 = 0x56;
    static constexpr uint8_t EXT_SENS_DATA_14 = 0x57;
    static constexpr uint8_t EXT_SENS_DATA_15 = 0x58;
    static constexpr uint8_t EXT_SENS_DATA_16 = 0x59;
    static constexpr uint8_t EXT_SENS_DATA_17 = 0x5A;
    static constexpr uint8_t EXT_SENS_DATA_18 = 0x5B;
    static constexpr uint8_t EXT_SENS_DATA_19 = 0x5C;
    static constexpr uint8_t EXT_SENS_DATA_20 = 0x5D;
    static constexpr uint8_t EXT_SENS_DATA_21 = 0x5E;
    static constexpr uint8_t EXT_SENS_DATA_22 = 0x5F;
    static constexpr uint8_t EXT_SENS_DATA_23 = 0x60;
    static constexpr uint8_t MOT_DETECT_STATUS = 0x61;
    static constexpr uint8_t I2C_SLV0_DO      = 0x63;
    static constexpr uint8_t I2C_SLV1_DO      = 0x64;
    static constexpr uint8_t I2C_SLV2_DO      = 0x65;
    static constexpr uint8_t I2C_SLV3_DO      = 0x66;
    static constexpr uint8_t I2C_MST_DELAY_CTRL = 0x67;
    static constexpr uint8_t SIGNAL_PATH_RESET  = 0x68;
    static constexpr uint8_t MOT_DETECT_CTRL  = 0x69;
    static constexpr uint8_t USER_CTRL        = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
    static constexpr uint8_t PWR_MGMT_1       = 0x6B; // Device defaults to the SLEEP mode
    static constexpr uint8_t PWR_MGMT_2       = 0x6C;
    static constexpr uint8_t DMP_BANK         = 0x6D;  // Activates a specific bank in the DMP
    static constexpr uint8_t DMP_RW_PNT       = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
    static constexpr uint8_t DMP_REG          = 0x6F;  // Register in DMP from which to read or to which to write
    static constexpr uint8_t DMP_REG_1        = 0x70;
    static constexpr uint8_t DMP_REG_2        = 0x71;
    static constexpr uint8_t FIFO_COUNTH      = 0x72;
    static constexpr uint8_t FIFO_COUNTL      = 0x73;
    static constexpr uint8_t FIFO_R_W         = 0x74;
    static constexpr uint8_t WHO_AM_I_MPU9250 = 0x75; // Should return = 0x71;
    static constexpr uint8_t XA_OFFSET_H      = 0x77;
    static constexpr uint8_t XA_OFFSET_L      = 0x78;
    static constexpr uint8_t YA_OFFSET_H      = 0x7A;
    static constexpr uint8_t YA_OFFSET_L      = 0x7B;
    static constexpr uint8_t ZA_OFFSET_H      = 0x7D;
    static constexpr uint8_t ZA_OFFSET_L      = 0x7E;

    /* -------------------------------------
        MPU9250 Registers end
       -------------------------------------
    */

    // structs have been put inside the class to free the global namespace
    enum class ACCEL_FS_SEL {
        A2G,
        A4G,
        A8G,
        A16G
    };
    enum class GYRO_FS_SEL {
        G250DPS,
        G500DPS,
        G1000DPS,
        G2000DPS
    };
    enum class MAG_OUTPUT_BITS {
        M14BITS,
        M16BITS
    };

    enum class FIFO_SAMPLE_RATE : uint8_t {
        SMPL_1000HZ,
        SMPL_500HZ,
        SMPL_333HZ,
        SMPL_250HZ,
        SMPL_200HZ,
        SMPL_167HZ,
        SMPL_143HZ,
        SMPL_125HZ,
    };

    enum class GYRO_DLPF_CFG : uint8_t {
        DLPF_250HZ,
        DLPF_184HZ,
        DLPF_92HZ,
        DLPF_41HZ,
        DLPF_20HZ,
        DLPF_10HZ,
        DLPF_5HZ,
        DLPF_3600HZ,
    };

    enum class ACCEL_DLPF_CFG : uint8_t {
        DLPF_218HZ_0,
        DLPF_218HZ_1,
        DLPF_99HZ,
        DLPF_45HZ,
        DLPF_21HZ,
        DLPF_10HZ,
        DLPF_5HZ,
        DLPF_420HZ,
    };

    static constexpr uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71};
    static constexpr uint8_t MPU9255_WHOAMI_DEFAULT_VALUE {0x73};
    static constexpr uint8_t MPU6500_WHOAMI_DEFAULT_VALUE {0x70};

    struct MPU9250Setting {
	MPU9250Setting() {}
        ACCEL_FS_SEL accel_fs_sel {ACCEL_FS_SEL::A16G};
        GYRO_FS_SEL gyro_fs_sel {GYRO_FS_SEL::G2000DPS};
        MAG_OUTPUT_BITS mag_output_bits {MAG_OUTPUT_BITS::M16BITS};
        FIFO_SAMPLE_RATE fifo_sample_rate {FIFO_SAMPLE_RATE::SMPL_200HZ};
        uint8_t gyro_fchoice {0x03};
        GYRO_DLPF_CFG gyro_dlpf_cfg {GYRO_DLPF_CFG::DLPF_41HZ};
        uint8_t accel_fchoice {0x01};
        ACCEL_DLPF_CFG accel_dlpf_cfg {ACCEL_DLPF_CFG::DLPF_45HZ};
    };

    // from Arduino.h
    typedef uint8_t byte;
    static constexpr double PI = 3.1415926535897932384626433832795;
    static constexpr double DEG_TO_RAD = 0.017453292519943295769236907684886;

    static constexpr uint8_t MPU9250_DEFAULT_ADDRESS {0x68};  // Device address when ADO = 0
    static constexpr uint8_t AK8963_ADDRESS {0x0C};           //  Address of magnetometer
    static constexpr uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};
    uint8_t mpu_i2c_addr {MPU9250_DEFAULT_ADDRESS};

    // settings
    MPU9250Setting setting;
    // TODO: this should be configured!!
    static constexpr uint8_t MAG_MODE {0x06};  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
    float acc_resolution {0.f};                // scale resolutions per LSB for the sensors
    float gyro_resolution {0.f};               // scale resolutions per LSB for the sensors
    float mag_resolution {0.f};                // scale resolutions per LSB for the sensors

    // Calibration Parameters
    float acc_bias[3] {0., 0., 0.};   // acc calibration value in ACCEL_FS_SEL: 2g
    float gyro_bias[3] {0., 0., 0.};  // gyro calibration value in GYRO_FS_SEL: 250dps
    float mag_bias_factory[3] {0., 0., 0.};
    float mag_bias[3] {0., 0., 0.};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
    float mag_scale[3] {1., 1., 1.};
    float magnetic_declination = -7.51;  // Japan, 24th June

    // Temperature
    int16_t temperature_count {0};  // temperature raw count output
    float temperature {0.f};        // Stores the real internal chip temperature in degrees Celsius

    // Self Test
    float self_test_result[6] {0.f};  // holds results of gyro and accelerometer self test

    // IMU Data
    float a[3] {0.f, 0.f, 0.f};
    float g[3] {0.f, 0.f, 0.f};
    float m[3] {0.f, 0.f, 0.f};

    // Other settings
    bool has_connected {false};

    // I2C
    int mpu_i2c_bus, mag_i2c_bus;
    // uint8_t i2c_err_;

public:
    static constexpr uint16_t CALIB_GYRO_SENSITIVITY {131};     // LSB/degrees/sec
    static constexpr uint16_t CALIB_ACCEL_SENSITIVITY {16384};  // LSB/g

    ~MPU9250_driver() {
        if (mpu_i2c_bus > 0) close(mpu_i2c_bus);
        if (mag_i2c_bus > 0) close(mag_i2c_bus);
    }

    bool begin(const uint8_t addr = 0x68, const MPU9250Setting& mpu_setting = MPU9250Setting()) {
        // addr should be valid for MPU
        if ((addr < MPU9250_DEFAULT_ADDRESS) || (addr > MPU9250_DEFAULT_ADDRESS + 7)) {
            return false;
        }
        mpu_i2c_addr = addr;
        setting = mpu_setting;

        mpu_i2c_bus = open("/dev/i2c-1", O_RDWR);
		ioctl(mpu_i2c_bus, I2C_SLAVE, mpu_i2c_addr);

        if (isConnectedMPU9250()) {
            initMPU9250();

            mag_i2c_bus = open("/dev/i2c-1", O_RDWR);
		    ioctl(mag_i2c_bus, I2C_SLAVE, AK8963_ADDRESS);

            if (isConnectedAK8963())
                initAK8963();
            else {
                has_connected = false;
                return false;
            }
        } else {
            has_connected = false;
            return false;
        }
        has_connected = true;
        return true;
    }

    void sleep(bool b) {
        byte c = i2c_smbus_read_byte_data(mpu_i2c_bus, PWR_MGMT_1);;  // read the value, change sleep bit to match b, write byte back to register
        if (b) {
            c = c | 0x40;  // sets the sleep bit
        } else {
            c = c & 0xBF;  // mask 1011111 keeps all the previous bits
        }
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_1, c);
    }

    void calibrateAccelGyro() {
        calibrate_acc_gyro_impl();
    }

    void calibrateMag() {
        calibrate_mag_impl();
    }

    bool isConnected() {
        has_connected = isConnectedMPU9250() && isConnectedAK8963();
        return has_connected;
    }

    bool isConnectedMPU9250() {
        byte c = i2c_smbus_read_byte_data(mpu_i2c_bus, WHO_AM_I_MPU9250);;
        bool b = (c == MPU9250_WHOAMI_DEFAULT_VALUE);
        b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
        b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
        return b;
    }

    bool isConnectedAK8963() {
        byte c = i2c_smbus_read_byte_data(mag_i2c_bus, AK8963_WHO_AM_I);
        return (c == AK8963_WHOAMI_DEFAULT_VALUE);
    }

    bool isSleeping() {
        byte c = i2c_smbus_read_byte_data(mpu_i2c_bus, PWR_MGMT_1);;
        return (c & 0x40) == 0x40;
    }

    bool available() {
        return has_connected && (i2c_smbus_read_byte_data(mpu_i2c_bus, INT_STATUS) & 0x01);
    }

    bool update() {
        // if (!available()) return false;

        update_accel_gyro();
        update_mag();

        return true;
    }

    float getAcc(const uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
    float getGyro(const uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
    float getMag(const uint8_t i) const { return (i < 3) ? m[i] : 0.f; }

    float getAccX() const { return a[0]; }
    float getAccY() const { return a[1]; }
    float getAccZ() const { return a[2]; }
    float getGyroX() const { return g[0]; }
    float getGyroY() const { return g[1]; }
    float getGyroZ() const { return g[2]; }
    float getMagX() const { return m[0]; }
    float getMagY() const { return m[1]; }
    float getMagZ() const { return m[2]; }

    float getAccBias(const uint8_t i) const { return (i < 3) ? acc_bias[i] : 0.f; }
    float getGyroBias(const uint8_t i) const { return (i < 3) ? gyro_bias[i] : 0.f; }
    float getMagBias(const uint8_t i) const { return (i < 3) ? mag_bias[i] : 0.f; }
    float getMagScale(const uint8_t i) const { return (i < 3) ? mag_scale[i] : 0.f; }

    float getAccBiasX() const { return acc_bias[0]; }
    float getAccBiasY() const { return acc_bias[1]; }
    float getAccBiasZ() const { return acc_bias[2]; }
    float getGyroBiasX() const { return gyro_bias[0]; }
    float getGyroBiasY() const { return gyro_bias[1]; }
    float getGyroBiasZ() const { return gyro_bias[2]; }
    float getMagBiasX() const { return mag_bias[0]; }
    float getMagBiasY() const { return mag_bias[1]; }
    float getMagBiasZ() const { return mag_bias[2]; }
    float getMagScaleX() const { return mag_scale[0]; }
    float getMagScaleY() const { return mag_scale[1]; }
    float getMagScaleZ() const { return mag_scale[2]; }

    float getTemperature() const { return temperature; }

    void setAccBias(const float x, const float y, const float z) {
        acc_bias[0] = x;
        acc_bias[1] = y;
        acc_bias[2] = z;
        write_accel_offset();
    }
    void setGyroBias(const float x, const float y, const float z) {
        gyro_bias[0] = x;
        gyro_bias[1] = y;
        gyro_bias[2] = z;
        write_gyro_offset();
    }
    void setMagBias(const float x, const float y, const float z) {
        mag_bias[0] = x;
        mag_bias[1] = y;
        mag_bias[2] = z;
    }
    void setMagScale(const float x, const float y, const float z) {
        mag_scale[0] = x;
        mag_scale[1] = y;
        mag_scale[2] = z;
    }
    void setMagneticDeclination(const float d) { magnetic_declination = d; }

    bool selftest() {
        return self_test_impl();
    }

private:
    void initMPU9250() {
        acc_resolution = get_acc_resolution(setting.accel_fs_sel);
        gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);
        mag_resolution = get_mag_resolution(setting.mag_output_bits);

        // reset device
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device

        // wake up device
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors

        // get stable time source
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        uint8_t mpu_config = (uint8_t)setting.gyro_dlpf_cfg;
        i2c_smbus_write_byte_data(mpu_i2c_bus, MPU_CONFIG, mpu_config);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        uint8_t sample_rate = (uint8_t)setting.fifo_sample_rate;
        i2c_smbus_write_byte_data(mpu_i2c_bus, SMPLRT_DIV, sample_rate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                            // determined inset in CONFIG above

        // Set gyroscope full scale range
        // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t c = i2c_smbus_read_byte_data(mpu_i2c_bus, GYRO_CONFIG);  // get current GYRO_CONFIG register value
        c = c & ~0xE0;                                     // Clear self-test bits [7:5]
        c = c & ~0x03;                                     // Clear Fchoice bits [1:0]
        c = c & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
        c = c | (uint8_t(setting.gyro_fs_sel) << 3);       // Set full scale range for the gyro
        c = c | (uint8_t(~setting.gyro_fchoice) & 0x03);   // Set Fchoice for the gyro
        i2c_smbus_write_byte_data(mpu_i2c_bus, GYRO_CONFIG, c);          // Write new GYRO_CONFIG value to register

        // Set accelerometer full-scale range configuration
        c = i2c_smbus_read_byte_data(mpu_i2c_bus, ACCEL_CONFIG);     // get current ACCEL_CONFIG register value
        c = c & ~0xE0;                                 // Clear self-test bits [7:5]
        c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
        c = c | (uint8_t(setting.accel_fs_sel) << 3);  // Set full scale range for the accelerometer
        i2c_smbus_write_byte_data(mpu_i2c_bus, ACCEL_CONFIG, c);     // Write new ACCEL_CONFIG register value

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        c = i2c_smbus_read_byte_data(mpu_i2c_bus, ACCEL_CONFIG2);        // get current ACCEL_CONFIG2 register value
        c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | (~(setting.accel_fchoice << 3) & 0x08);    // Set accel_fchoice_b to 1
        c = c | (uint8_t(setting.accel_dlpf_cfg) & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        i2c_smbus_write_byte_data(mpu_i2c_bus, ACCEL_CONFIG2, c);        // Write new ACCEL_CONFIG2 register value

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        i2c_smbus_write_byte_data(mpu_i2c_bus, INT_PIN_CFG, 0x22);
        i2c_smbus_write_byte_data(mpu_i2c_bus, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    }

    void initAK8963() {
        // First extract the factory calibration for each magnetometer axis
        uint8_t raw_data[3];                            // x/y/z gyro calibration data stored here
        i2c_smbus_write_byte_data(mag_i2c_bus, AK8963_CNTL, 0x00);  // Power down magnetometer
        i2c_smbus_write_byte_data(mag_i2c_bus, AK8963_CNTL, 0x0F);  // Enter Fuse ROM access mode
        i2c_smbus_read_i2c_block_data(mag_i2c_bus, AK8963_ASAX, 3, &raw_data[0]);      // Read the x-, y-, and z-axis calibration values
        mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.;  // Return x-axis sensitivity adjustment values, etc.
        mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
        mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
        i2c_smbus_write_byte_data(mag_i2c_bus, AK8963_CNTL, 0x00);  // Power down magnetometer
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        i2c_smbus_write_byte_data(mag_i2c_bus, AK8963_CNTL, (uint8_t)setting.mag_output_bits << 4 | MAG_MODE);  // Set magnetometer data resolution and sample ODR
    }

public:
    void update_accel_gyro() {
        int16_t raw_acc_gyro_data[7];        // used to read all 14 bytes at once from the MPU9250 accel/gyro
        read_accel_gyro(raw_acc_gyro_data);  // INT cleared on any read

        // Now we'll calculate the accleration value into actual g's
        a[0] = (float)raw_acc_gyro_data[0] * acc_resolution;  // get actual g value, this depends on scale being set
        a[1] = (float)raw_acc_gyro_data[1] * acc_resolution;
        a[2] = (float)raw_acc_gyro_data[2] * acc_resolution;

        temperature_count = raw_acc_gyro_data[3];                  // Read the adc values
        temperature = ((float)temperature_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade

        // Calculate the gyro value into actual degrees per second
        g[0] = (float)raw_acc_gyro_data[4] * gyro_resolution;  // get actual gyro value, this depends on scale being set
        g[1] = (float)raw_acc_gyro_data[5] * gyro_resolution;
        g[2] = (float)raw_acc_gyro_data[6] * gyro_resolution;
    }

private:
    void read_accel_gyro(int16_t* destination) {
        uint8_t raw_data[14];                                                 // x/y/z accel register data stored here
        i2c_smbus_read_i2c_block_data(mpu_i2c_bus, ACCEL_XOUT_H, 14, &raw_data[0]);             // Read the 14 raw data registers into data array
        destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1];  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
        destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
        destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
        destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
        destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
        destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
    }

public:
    void update_mag() {
        int16_t mag_count[3] = {0, 0, 0};  // Stores the 16-bit signed magnetometer sensor output

        // Read the x/y/z adc values
        if (read_mag(mag_count)) {
            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            // mag_bias is calcurated in 16BITS
            float bias_to_current_bits = mag_resolution / get_mag_resolution(MAG_OUTPUT_BITS::M16BITS);
            m[0] = (float)(mag_count[0] * mag_resolution * mag_bias_factory[0] - mag_bias[0] * bias_to_current_bits) * mag_scale[0];  // get actual magnetometer value, this depends on scale being set
            m[1] = (float)(mag_count[1] * mag_resolution * mag_bias_factory[1] - mag_bias[1] * bias_to_current_bits) * mag_scale[1];
            m[2] = (float)(mag_count[2] * mag_resolution * mag_bias_factory[2] - mag_bias[2] * bias_to_current_bits) * mag_scale[2];
        }
    }

private:
    bool read_mag(int16_t* destination) {
        uint8_t raw_data[7];
        i2c_smbus_read_i2c_block_data(mag_i2c_bus, AK8963_XOUT_L, 7, &raw_data[0]);      // Read the six raw data and ST2 registers sequentially into data array
        destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
        destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];

        return true;

        /* -------- Original Function ---------*/
        // const uint8_t st1 = i2c_smbus_read_byte_data(mag_i2c_bus, AK8963_ST1);
        // if (st1 & 0x01) {                                                    // wait for magnetometer data ready bit to be set
        //     uint8_t raw_data[7];                                             // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        //     i2c_smbus_read_i2c_block_data(mag_i2c_bus, AK8963_XOUT_L, 7, &raw_data[0]);      // Read the six raw data and ST2 registers sequentially into data array
        //     if (MAG_MODE == 0x02 || MAG_MODE == 0x04 || MAG_MODE == 0x06) {  // continuous or external trigger read mode
        //         if ((st1 & 0x02) != 0)                                       // check if data is not skipped
        //             return false;                                            // this should be after data reading to clear DRDY register
        //     }
            
        //     uint8_t c = raw_data[6];                                         // End data read by reading ST2 register
        //     if (!(c & 0x08)) {                                               // Check if magnetic sensor overflow set, if not then report data
        //         destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
        //         destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
        //         destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
        //         return true;
        //     }
        // }
        
        return false;
    }

    int16_t read_temperature_data() {
        uint8_t raw_data[2];                                    // x/y/z gyro register data stored here
        i2c_smbus_read_i2c_block_data(mpu_i2c_bus, TEMP_OUT_H, 2, &raw_data[0]);  // Read the two raw data registers sequentially into data array
        return ((int16_t)raw_data[0] << 8) | raw_data[1];       // Turn the MSB and LSB into a 16-bit value
    }

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    // ACCEL_FS_SEL: 2g (maximum sensitivity)
    // GYRO_FS_SEL: 250dps (maximum sensitivity)
    void calibrate_acc_gyro_impl() {
        set_acc_gyro_to_calibration();
        collect_acc_gyro_data_to(acc_bias, gyro_bias);
        write_accel_offset();
        write_gyro_offset();
        initMPU9250();
    }

    void set_acc_gyro_to_calibration() {
        // reset device
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
        // else use the internal oscillator, bits 2:0 = 001
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_1, 0x01);
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_2, 0x00);

        // Configure device for bias calculation
        i2c_smbus_write_byte_data(mpu_i2c_bus, INT_ENABLE, 0x00);    // Disable all interrupts
        i2c_smbus_write_byte_data(mpu_i2c_bus, FIFO_EN, 0x00);       // Disable FIFO
        i2c_smbus_write_byte_data(mpu_i2c_bus, PWR_MGMT_1, 0x00);    // Turn on internal clock source
        i2c_smbus_write_byte_data(mpu_i2c_bus, I2C_MST_CTRL, 0x00);  // Disable I2C master
        i2c_smbus_write_byte_data(mpu_i2c_bus, USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
        i2c_smbus_write_byte_data(mpu_i2c_bus, USER_CTRL, 0x0C);     // Reset FIFO and DMP

        // Configure MPU6050 gyro and accelerometer for bias calculation
        i2c_smbus_write_byte_data(mpu_i2c_bus, MPU_CONFIG, 0x01);    // Set low-pass filter to 188 Hz
        i2c_smbus_write_byte_data(mpu_i2c_bus, SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
        i2c_smbus_write_byte_data(mpu_i2c_bus, GYRO_CONFIG, 0x00);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        i2c_smbus_write_byte_data(mpu_i2c_bus, ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        i2c_smbus_write_byte_data(mpu_i2c_bus, USER_CTRL, 0x40);  // Enable FIFO
        i2c_smbus_write_byte_data(mpu_i2c_bus, FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    }

    void collect_acc_gyro_data_to(float* a_bias, float* g_bias) {
        // At end of sample accumulation, turn off FIFO sensor read
        uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
        i2c_smbus_write_byte_data(mpu_i2c_bus, FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
        i2c_smbus_read_i2c_block_data(mpu_i2c_bus, FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
        uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
        uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

        for (uint16_t ii = 0; ii < packet_count; ii++) {
            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
            i2c_smbus_read_i2c_block_data(mpu_i2c_bus, FIFO_R_W, 12, &data[0]);              // read data for averaging
            accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
            accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
            gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
            gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
            gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

            a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            a_bias[1] += (float)accel_temp[1];
            a_bias[2] += (float)accel_temp[2];
            g_bias[0] += (float)gyro_temp[0];
            g_bias[1] += (float)gyro_temp[1];
            g_bias[2] += (float)gyro_temp[2];
        }
        a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
        a_bias[1] /= (float)packet_count;
        a_bias[2] /= (float)packet_count;
        g_bias[0] /= (float)packet_count;
        g_bias[1] /= (float)packet_count;
        g_bias[2] /= (float)packet_count;

        if (a_bias[2] > 0L) {
            a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
        }  // Remove gravity from the z-axis accelerometer bias calculation
        else {
            a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
        }
    }

    void write_accel_offset() {
        // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
        // the accelerometer biases calculated above must be divided by 8.

        uint8_t read_data[2] = {0};
        int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
        i2c_smbus_read_i2c_block_data(mpu_i2c_bus, XA_OFFSET_H, 2, &read_data[0]);  // Read factory accelerometer trim values
        acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
        i2c_smbus_read_i2c_block_data(mpu_i2c_bus, YA_OFFSET_H, 2, &read_data[0]);
        acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
        i2c_smbus_read_i2c_block_data(mpu_i2c_bus, ZA_OFFSET_H, 2, &read_data[0]);
        acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

        int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
        for (int i = 0; i < 3; i++) {
            if (acc_bias_reg[i] % 2) {
                mask_bit[i] = 0;
            }
            acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
            if (mask_bit[i]) {
                acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
            } else {
                acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
            }
        }

        uint8_t write_data[6] = {0};
        write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
        write_data[1] = (acc_bias_reg[0]) & 0xFF;
        write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
        write_data[3] = (acc_bias_reg[1]) & 0xFF;
        write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
        write_data[5] = (acc_bias_reg[2]) & 0xFF;

        // Push accelerometer biases to hardware registers
        i2c_smbus_write_byte_data(mpu_i2c_bus, XA_OFFSET_H, write_data[0]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, XA_OFFSET_L, write_data[1]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, YA_OFFSET_H, write_data[2]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, YA_OFFSET_L, write_data[3]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, ZA_OFFSET_H, write_data[4]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, ZA_OFFSET_L, write_data[5]);
    }

    void write_gyro_offset() {
        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        uint8_t gyro_offset_data[6] {0};
        gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
        gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
        gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
        gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
        gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

        // Push gyro biases to hardware registers
        i2c_smbus_write_byte_data(mpu_i2c_bus, XG_OFFSET_H, gyro_offset_data[0]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, XG_OFFSET_L, gyro_offset_data[1]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, YG_OFFSET_H, gyro_offset_data[2]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, YG_OFFSET_L, gyro_offset_data[3]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, ZG_OFFSET_H, gyro_offset_data[4]);
        i2c_smbus_write_byte_data(mpu_i2c_bus, ZG_OFFSET_L, gyro_offset_data[5]);
    }

    // mag calibration is executed in MAG_OUTPUT_BITS: 16BITS
    void calibrate_mag_impl() {
        // set MAG_OUTPUT_BITS to maximum to calibrate
        MAG_OUTPUT_BITS mag_output_bits_cache = setting.mag_output_bits;
        setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
        initAK8963();
        collect_mag_data_to(mag_bias, mag_scale);

        // restore MAG_OUTPUT_BITS
        setting.mag_output_bits = mag_output_bits_cache;
        initAK8963();
    }

    void collect_mag_data_to(float* m_bias, float* m_scale) {

        // shoot for ~fifteen seconds of mag data
        uint16_t sample_count = 0;
        if (MAG_MODE == 0x02)
            sample_count = 128;     // at 8 Hz ODR, new mag data is available every 125 ms
        else if (MAG_MODE == 0x06)  // in this library, fixed to 100Hz
            sample_count = 1500;    // at 100 Hz ODR, new mag data is available every 10 ms

        int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
        int16_t mag_max[3] = {-32767, -32767, -32767};
        int16_t mag_min[3] = {32767, 32767, 32767};
        int16_t mag_temp[3] = {0, 0, 0};
        for (uint16_t ii = 0; ii < sample_count; ii++) {
            read_mag(mag_temp);  // Read the mag data
            for (int jj = 0; jj < 3; jj++) {
                if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
                if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
            }
        }

        // Get hard iron correction
        bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
        bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
        bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

        float bias_resolution = get_mag_resolution(MAG_OUTPUT_BITS::M16BITS);
        m_bias[0] = (float)bias[0] * bias_resolution * mag_bias_factory[0];  // save mag biases in G for main program
        m_bias[1] = (float)bias[1] * bias_resolution * mag_bias_factory[1];
        m_bias[2] = (float)bias[2] * bias_resolution * mag_bias_factory[2];

        // Get soft iron correction estimate
        //*** multiplication by mag_bias_factory added in accordance with the following comment
        //*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
        scale[0] = (float)(mag_max[0] - mag_min[0]) * mag_bias_factory[0] / 2;  // get average x axis max chord length in counts
        scale[1] = (float)(mag_max[1] - mag_min[1]) * mag_bias_factory[1] / 2;  // get average y axis max chord length in counts
        scale[2] = (float)(mag_max[2] - mag_min[2]) * mag_bias_factory[2] / 2;  // get average z axis max chord length in counts

        float avg_rad = scale[0] + scale[1] + scale[2];
        avg_rad /= 3.0;

        m_scale[0] = avg_rad / ((float)scale[0]);
        m_scale[1] = avg_rad / ((float)scale[1]);
        m_scale[2] = avg_rad / ((float)scale[2]);
    }

    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    bool self_test_impl()  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    {
        uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
        int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
        float factoryTrim[6];
        uint8_t FS = 0;

        i2c_smbus_write_byte_data(mpu_i2c_bus, SMPLRT_DIV, 0x00);       // Set gyro sample rate to 1 kHz
        i2c_smbus_write_byte_data(mpu_i2c_bus, MPU_CONFIG, 0x02);       // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
        i2c_smbus_write_byte_data(mpu_i2c_bus, GYRO_CONFIG, FS << 3);   // Set full scale range for the gyro to 250 dps
        i2c_smbus_write_byte_data(mpu_i2c_bus, ACCEL_CONFIG2, 0x02);    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
        i2c_smbus_write_byte_data(mpu_i2c_bus, ACCEL_CONFIG, FS << 3);  // Set full scale range for the accelerometer to 2 g

        for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

            i2c_smbus_read_i2c_block_data(mpu_i2c_bus, ACCEL_XOUT_H, 6, &raw_data[0]);          // Read the six raw data registers into data array
            aAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            aAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            aAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

            i2c_smbus_read_i2c_block_data(mpu_i2c_bus, GYRO_XOUT_H, 6, &raw_data[0]);           // Read the six raw data registers sequentially into data array
            gAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            gAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            gAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
        }

        for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
            aAvg[ii] /= 200;
            gAvg[ii] /= 200;
        }

        // Configure the accelerometer for self-test
        i2c_smbus_write_byte_data(mpu_i2c_bus, ACCEL_CONFIG, 0xE0);  // Enable self test on all three axes and set accelerometer range to +/- 2 g
        i2c_smbus_write_byte_data(mpu_i2c_bus, GYRO_CONFIG, 0xE0);   // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

        for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

            i2c_smbus_read_i2c_block_data(mpu_i2c_bus, ACCEL_XOUT_H, 6, &raw_data[0]);            // Read the six raw data registers into data array
            aSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            aSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            aSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

            i2c_smbus_read_i2c_block_data(mpu_i2c_bus, GYRO_XOUT_H, 6, &raw_data[0]);             // Read the six raw data registers sequentially into data array
            gSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
            gSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            gSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
        }

        for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
            aSTAvg[ii] /= 200;
            gSTAvg[ii] /= 200;
        }

        // Configure the gyro and accelerometer for normal operation
        i2c_smbus_write_byte_data(mpu_i2c_bus, ACCEL_CONFIG, 0x00);
        i2c_smbus_write_byte_data(mpu_i2c_bus, GYRO_CONFIG, 0x00);

        // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
        uint8_t self_test_data[6];
        self_test_data[0] = i2c_smbus_read_byte_data(mpu_i2c_bus, SELF_TEST_X_ACCEL);  // X-axis accel self-test results
        self_test_data[1] = i2c_smbus_read_byte_data(mpu_i2c_bus, SELF_TEST_Y_ACCEL);  // Y-axis accel self-test results
        self_test_data[2] = i2c_smbus_read_byte_data(mpu_i2c_bus, SELF_TEST_Z_ACCEL);  // Z-axis accel self-test results
        self_test_data[3] = i2c_smbus_read_byte_data(mpu_i2c_bus, SELF_TEST_X_GYRO);   // X-axis gyro self-test results
        self_test_data[4] = i2c_smbus_read_byte_data(mpu_i2c_bus, SELF_TEST_Y_GYRO);   // Y-axis gyro self-test results
        self_test_data[5] = i2c_smbus_read_byte_data(mpu_i2c_bus, SELF_TEST_Z_GYRO);   // Z-axis gyro self-test results

        // Retrieve factory self-test value from self-test code reads
        factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[0] - 1.0)));  // FT[Xa] factory trim calculation
        factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[1] - 1.0)));  // FT[Ya] factory trim calculation
        factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[2] - 1.0)));  // FT[Za] factory trim calculation
        factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[3] - 1.0)));  // FT[Xg] factory trim calculation
        factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[4] - 1.0)));  // FT[Yg] factory trim calculation
        factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[5] - 1.0)));  // FT[Zg] factory trim calculation

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get percent, must multiply by 100
        for (int i = 0; i < 3; i++) {
            self_test_result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;          // Report percent differences
            self_test_result[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.;  // Report percent differences
        }

        bool b = true;
        for (uint8_t i = 0; i < 6; ++i) {
            b &= fabs(self_test_result[i]) <= 14.f;
        }
        return b;
    }

    float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const {
        switch (accel_af_sel) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case ACCEL_FS_SEL::A2G:
                return 2.0 / 32768.0;
            case ACCEL_FS_SEL::A4G:
                return 4.0 / 32768.0;
            case ACCEL_FS_SEL::A8G:
                return 8.0 / 32768.0;
            case ACCEL_FS_SEL::A16G:
                return 16.0 / 32768.0;
            default:
                return 0.;
        }
    }

    float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const {
        switch (gyro_fs_sel) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case GYRO_FS_SEL::G250DPS:
                return 250.0 / 32768.0;
            case GYRO_FS_SEL::G500DPS:
                return 500.0 / 32768.0;
            case GYRO_FS_SEL::G1000DPS:
                return 1000.0 / 32768.0;
            case GYRO_FS_SEL::G2000DPS:
                return 2000.0 / 32768.0;
            default:
                return 0.;
        }
    }

    float get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits) const {
        switch (mag_output_bits) {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
            // Proper scale to return milliGauss
            case MAG_OUTPUT_BITS::M14BITS:
                return 10. * 4912. / 8190.0;
            case MAG_OUTPUT_BITS::M16BITS:
                return 10. * 4912. / 32760.0;
            default:
                return 0.;
        }
    }
};
