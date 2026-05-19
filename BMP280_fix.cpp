/*
 * แก้ไขปัญหาการอ่านค่า BMP280
 * ==============================
 *
 * ปัญหาที่พบ:
 * 1. ไม่มีการตรวจสอบ NULL pointer ก่อนใช้ bmp280
 * 2. Logic ผิดใน getBMP_I2C - อ่าน temperature แล้ว return true ทันที
 *    ทำให้ไม่อ่าน pressure เมื่อมีทั้ง temp และ pressure sensors
 */

// ============= แก้ไขที่ 1: เพิ่มการตรวจสอบ NULL =============

// ที่บรรทัด 1132 แก้จาก:
case PORT_BMP280_I2C0:
    if (config.i2c_enable)
    {
        if (getBMP_I2C(*bmp280, port))
            return true;
    }
    break;

// เป็น:
case PORT_BMP280_I2C0:
    if (config.i2c_enable)
    {
        if (bmp280 != NULL) {  // ✅ เพิ่มการตรวจสอบ NULL
            if (getBMP_I2C(*bmp280, port))
                return true;
        } else {
            log_e("BMP280 I2C0: bmp280 pointer is NULL!");
        }
    }
    break;

// ที่บรรทัด 1139 แก้จาก:
case PORT_BMP280_I2C1:
    if (config.i2c1_enable)
    {
        if (getBMP_I2C(*bmp280, port))
            return true;
    }
    break;

// เป็น:
case PORT_BMP280_I2C1:
    if (config.i2c1_enable)
    {
        if (bmp280 != NULL) {  // ✅ เพิ่มการตรวจสอบ NULL
            if (getBMP_I2C(*bmp280, port))
                return true;
        } else {
            log_e("BMP280 I2C1: bmp280 pointer is NULL!");
        }
    }
    break;


// ============= แก้ไขที่ 2: แก้ Logic ใน getBMP_I2C =============

// ที่บรรทัด 485-523 แก้จาก:
bool getBMP_I2C(Adafruit_BMP280 &node, uint8_t port)
{
    bool result;
    /* Default settings from datasheet. */
    node.setSampling(Adafruit_BMP280::MODE_FORCED,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);
    result = node.takeForcedMeasurement();
    if (result)
    {
        float pressure = node.readPressure();
        float temperature = node.readTemperature();
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == port)
            {
                sensorUpdate(i, temperature);
                return true;  // ❌ PROBLEM: Returns immediately!
            }
            else if (config.sensor[i].type == SENSOR_PRESSURE && config.sensor[i].port == port)
            {
                if(pressure>0){
                    sensorUpdate(i, pressure/100.0F);
                }
            }
        }
        return true;
    }
    return false;
}

// เป็น:
bool getBMP_I2C(Adafruit_BMP280 &node, uint8_t port)
{
    bool result;
    bool dataUpdated = false;  // ✅ Track if any data was updated

    /* Default settings from datasheet. */
    node.setSampling(Adafruit_BMP280::MODE_FORCED,
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    result = node.takeForcedMeasurement();
    if (result)
    {
        float pressure = node.readPressure();
        float temperature = node.readTemperature();

        // ✅ Read BOTH temperature AND pressure
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;

            if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == port)
            {
                sensorUpdate(i, temperature);
                dataUpdated = true;  // ✅ Mark as updated, but don't return
                log_d("BMP280: Temperature = %.2f°C", temperature);
            }
            else if (config.sensor[i].type == SENSOR_PRESSURE && config.sensor[i].port == port)
            {
                if(pressure > 0) {
                    sensorUpdate(i, pressure / 100.0F);  // Convert Pa to hPa
                    dataUpdated = true;
                    log_d("BMP280: Pressure = %.2f hPa", pressure / 100.0F);
                } else {
                    log_e("BMP280: Invalid pressure reading: %.2f", pressure);
                }
            }
        }
        return dataUpdated;  // ✅ Return true only if data was actually updated
    }
    else
    {
        log_e("BMP280: takeForcedMeasurement() failed");
    }
    return false;
}


// ============= แก้ไขที่ 3 (ถ้าต้องการให้ดีขึ้น): เพิ่ม Error Handling =============

bool getBMP_I2C_Improved(Adafruit_BMP280 &node, uint8_t port)
{
    bool dataUpdated = false;

    // ✅ Check sensor ID to verify BMP280 is connected
    uint8_t sensorID = node.sensorID();
    if (sensorID == 0 || sensorID == 0xFF) {
        log_e("BMP280: Invalid sensor ID 0x%02X - sensor not connected!", sensorID);
        return false;
    }

    /* Default settings from datasheet. */
    node.setSampling(Adafruit_BMP280::MODE_FORCED,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);

    if (!node.takeForcedMeasurement()) {
        log_e("BMP280: Failed to take forced measurement");
        return false;
    }

    float pressure = node.readPressure();
    float temperature = node.readTemperature();

    // ✅ Validate readings
    if (isnan(pressure) || isnan(temperature)) {
        log_e("BMP280: Invalid readings - Pressure: %.2f, Temperature: %.2f", pressure, temperature);
        return false;
    }

    // ✅ Check for reasonable value ranges
    if (temperature < -40 || temperature > 85) {
        log_e("BMP280: Temperature out of range: %.2f°C (valid: -40 to +85°C)", temperature);
        return false;
    }

    if (pressure < 30000 || pressure > 110000) {
        log_e("BMP280: Pressure out of range: %.2f Pa (valid: 30000 to 110000 Pa)", pressure);
        return false;
    }

    // ✅ Update all enabled sensors for this port
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if(!config.sensor[i].enable) continue;

        if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == port)
        {
            sensorUpdate(i, temperature);
            dataUpdated = true;
            log_d("BMP280: Temperature = %.2f°C", temperature);
        }
        else if (config.sensor[i].type == SENSOR_PRESSURE && config.sensor[i].port == port)
        {
            sensorUpdate(i, pressure / 100.0F);
            dataUpdated = true;
            log_d("BMP280: Pressure = %.2f hPa", pressure / 100.0F);
        }
    }

    return dataUpdated;
}


// ============= การทดสอบ =============

/*
 * 1. คอมไพล์และอัปโหลดโค้ดที่แก้ไขแล้ว
 * 2. เปิด Serial Monitor ดู log:
 *    - ถ้า bmp280 เป็น NULL: "BMP280 I2C0: bmp280 pointer is NULL!"
 *    - ถ้าอ่านสำเร็จ: "BMP280: Temperature = 25.30°C"
 *    - ถ้าอ่านสำเร็จ: "BMP280: Pressure = 1013.25 hPa"
 *
 * 3. ทดสอบกรณีมีทั้ง Temp และ Pressure sensors:
 *    - Sensor #1: Temperature
 *    - Sensor #2: Pressure
 *    - ต้องเห็นทั้งสองค่าถูกอ่านและอัปเดต
 *
 * 4. ทดสอบกรณีมีเฉพาะ Pressure sensor:
 *    - Sensor #1: Pressure
 *    - ต้องเห็น pressure ถูกอ่านถูกต้อง
 */
