/*
 * วิธีการตรวจสอบปัญหา Heap Memory Restart
 * ================================================
 *
 * สาเหตุที่อาจทำให้ไม่เกิดการรีสตาร์ทเมื่อ FreeHeap < 80000:
 *
 * 1. heapCount ต้องมากกว่า 3 (ต้องเท่ากับ 4 ขึ้นไป)
 *    - ต้องเช็คแล้วพบว่า heap < 80000 ถึง 4 ครั้งติดต่อกันถึงจะรีสตาร์ท
 *    - ถ้าระหว่างนั้น heap >= 80000 สักครั้ง → heapCount จะลดลง
 *
 * 2. heapCount ถูกลดเมื่อ heap พอ
 *    - ทำให้ heapCount สลับไปมาไม่เคยถึง 4
 *
 * 3. เช็คทุกๆ 10 วินาที
 *    - ต้องใช้เวลา 40 วินาทีถึงจะรีสตาร์ท
 *
 * 4. PSRAM อาจมีผลต่อค่าที่ได้
 */

// ============= วิธีการ Debug =============

// 1. เพิ่ม logging เพื่อดูการทำงานของ heapCount
// แก้ไขโค้ดบรรทัด 6804-6826:

#if defined(HEAP_DEBUG)  // เพิ่มนี้ใน main.h หรือ config
    static uint32_t lastHeapDebug = 0;
    if (millis() > lastHeapDebug + 1000) {  // ทุกๆ 1 วินาที
        lastHeapDebug = millis();
        log_d("Heap: %u bytes, heapCount: %u", ESP.getFreeHeap(), heapCount);

        // แสดงข้อมูล heap ทั้งหมด
        log_d("  Total Heap: %u", ESP.getHeapSize());
        log_d("  Free Heap: %u", ESP.getFreeHeap());
        log_d("  Max Free Heap: %u", ESP.getMaxAllocHeap());
        log_d("  PSRAM: %u", ESP.getPsRamSize());
        log_d("  Free PSRAM: %u", ESP.getFreePsRam());
    }
#endif

// 2. แก้ไขโค้ด heap check ให้ดีขึ้น:

// วิธี A: เปลี่ยนจาก > 3 เป็น >= 3 (ให้รีสตาร์ทเร็วขึ้น)
#ifdef __XTENSA__
if (ESP.getFreeHeap() < 60000)
#else
if (ESP.getFreeHeap() < 80000)
#endif
{
    if (++heapCount >= 3)  // เปลี่ยนจาก > 3 เป็น >= 3
    {
        heapCount = 0;
        log_d("LOW MEMORY RESTART: FreeHeap=%u", ESP.getFreeHeap());
        vTaskSuspendAll();
        WiFi.disconnect(true);
        WiFi.persistent(false);
        WiFi.mode(WIFI_OFF);
        PowerOff();
        esp_restart();
    }
}
else
{
    // ลด heapCount ช้าลง (ไม่ลดทุกครั้ง)
    static uint8_t heapDecayCounter = 0;
    if (++heapDecayCounter >= 5) {  // ลดทุกๆ 5 ครั้ง
        heapDecayCounter = 0;
        if (heapCount > 0)
            heapCount--;
    }
}

// วิธี B: ใช้เวลาสะสมแทนการนับครั้ง
static uint32_t lowHeapStartTime = 0;
#ifdef __XTENSA__
if (ESP.getFreeHeap() < 60000)
#else
if (ESP.getFreeHeap() < 80000)
#endif
{
    if (lowHeapStartTime == 0) {
        lowHeapStartTime = millis();
        log_d("LOW HEAP DETECTED: %u bytes", ESP.getFreeHeap());
    }

    // ถ้า heap น้อยเกิน 30 วินาที → รีสตาร์ท
    if (millis() - lowHeapStartTime > 30000) {
        log_d("LOW MEMORY TIMEOUT RESTART: FreeHeap=%u", ESP.getFreeHeap());
        lowHeapStartTime = 0;
        vTaskSuspendAll();
        WiFi.disconnect(true);
        WiFi.persistent(false);
        WiFi.mode(WIFI_OFF);
        PowerOff();
        esp_restart();
    }
}
else
{
    if (lowHeapStartTime != 0) {
        log_d("Heap recovered: %u bytes (was low for %lu ms)",
              ESP.getFreeHeap(), millis() - lowHeapStartTime);
        lowHeapStartTime = 0;
    }
}

// วิธี C: ปรับ threshold ให้เหมาะสมกับ PSRAM
uint32_t getTotalFreeHeap() {
    uint32_t total = ESP.getFreeHeap();
    if (ESP.getPsRamSize() > 0) {
        total += ESP.getFreePsRam();
    }
    return total;
}

// แล้วใช้ในการเช็ค:
uint32_t totalFreeHeap = getTotalFreeHeap();
uint32_t heapThreshold = (ESP.getPsRamSize() > 0) ? 200000 : 80000;

if (totalFreeHeap < heapThreshold) {
    // ... restart logic
}

// ============= การทดสอบ =============

/*
 * 1. คอมไพล์และอัปโหลดโค้ดที่มี HEAP_DEBUG
 * 2. เปิด Serial Monitor ดูค่า heap:
 *    - Heap: 75000 bytes, heapCount: 1
 *    - Heap: 79000 bytes, heapCount: 0  (heap พอ → ลด heapCount)
 *    - Heap: 76000 bytes, heapCount: 1
 *    - Heap: 76000 bytes, heapCount: 2
 *    - Heap: 76000 bytes, heapCount: 3
 *    - Heap: 76000 bytes, heapCount: 4  (ถึง 4 → รีสตาร์ท)
 *
 * 3. สังเกตว่า heapCount ถึง 4 หรือไม่
 * 4. ถ้าไม่ถึง → แสดงว่า heap กลับมาพอสักครั้ง
 * 5. ใช้วิธี B (เวลาสะสม) จะแก้ปัญหานี้ได้
 */
