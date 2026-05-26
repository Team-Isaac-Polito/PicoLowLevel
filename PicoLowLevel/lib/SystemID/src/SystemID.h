#ifndef SYSTEM_ID_H
#define SYSTEM_ID_H

#include <Arduino.h>
#include <LittleFS.h>
#include "CanWrapper.h"

class SystemID {
public:
    SystemID();

    /// LittleFS'dan oldingi session ID ni o'qiydi.
    /// Agar fayl yo'q bo'lsa, 0 qaytaradi (yangi MCU degani).
    uint32_t loadStoredID();

    /// Yangi tasodifiy session ID yaratadi.
    uint32_t generateNewID();

    /// Session ID ni LittleFS'ga saqlaydi.
    bool saveID(uint32_t id);

    /// CAN bus orqali o'z saqlangan ID'sini jo'natadi (broadcast).
    void broadcastID(CanWrapper &can, uint32_t storedID);

    /// Boshqa MCU'lardan kelgan ID'larni qabul qiladi va consensusni tekshiradi.
    /// timeout_ms: kutish vaqti (user xohishiga ko'ra 10000ms gacha ruxsat).
    /// Qaytaradi: konsensusga erishildimi yoki yo'qmi.
    bool waitForConsensus(CanWrapper &can, uint32_t myStoredID, uint32_t timeout_ms = 10000);

    /// Oxirgi tekshiruv natijasi
    bool isVerified() const;

private:
    static constexpr const char* ID_FILE = "/sys_id.bin";
    static constexpr float CONSENSUS_THRESHOLD = 0.6f;  // 60%
    static constexpr uint8_t MAX_MODULES = 3;

    bool _verified = false;
    uint32_t _receivedIDs[MAX_MODULES];
    uint8_t _receivedSenders[MAX_MODULES];
    uint8_t _receivedCount = 0;
};

#endif // SYSTEM_ID_H
