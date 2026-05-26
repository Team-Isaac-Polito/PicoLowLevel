#include "SystemID.h"
#include "../../../include/communication.h"

SystemID::SystemID() : _verified(false), _receivedCount(0) {
    memset(_receivedIDs, 0, sizeof(_receivedIDs));
    memset(_receivedSenders, 0, sizeof(_receivedSenders));
}

uint32_t SystemID::loadStoredID() {
    if (!LittleFS.exists(ID_FILE)) {
        Serial.println("[SystemID] No stored ID file found.");
        return 0; // Means a new MCU or uninitialized
    }

    File file = LittleFS.open(ID_FILE, "r");
    if (!file) {
        Serial.println("[SystemID] Failed to open ID file for reading.");
        return 0;
    }

    uint32_t id = 0;
    if (file.read((uint8_t*)&id, sizeof(id)) != sizeof(id)) {
        Serial.println("[SystemID] Failed to read complete ID from file.");
        id = 0;
    }
    file.close();
    Serial.print("[SystemID] Loaded Stored ID: 0x");
    Serial.println(id, HEX);
    return id;
}

uint32_t SystemID::generateNewID() {
    // Generate pseudorandom ID based on chip id, micros, and analog noise
    uint32_t id = micros();
    
    // Mix in some analog noise
    pinMode(A0, INPUT);
    id ^= (analogRead(A0) << 16);
    
    // Mix in RP2040 unique board/chip ID if available
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    for (int i = 0; i < 8; i++) {
        id = (id << 4) ^ board_id.id[i];
    }
    
    // Ensure we don't accidentally generate 0 as a valid ID (0 represents uninitialized)
    if (id == 0) {
        id = 0xFEE1DEAD;
    }

    Serial.print("[SystemID] Generated New ID: 0x");
    Serial.println(id, HEX);
    return id;
}

bool SystemID::saveID(uint32_t id) {
    File file = LittleFS.open(ID_FILE, "w");
    if (!file) {
        Serial.println("[SystemID] Failed to open ID file for writing.");
        return false;
    }

    size_t written = file.write((const uint8_t*)&id, sizeof(id));
    file.close();

    if (written == sizeof(id)) {
        Serial.print("[SystemID] Saved ID: 0x");
        Serial.println(id, HEX);
        return true;
    } else {
        Serial.println("[SystemID] Failed to write complete ID to file.");
        return false;
    }
}

void SystemID::broadcastID(CanWrapper &can, uint32_t storedID) {
    uint8_t payload[5];
    // Byte 0-3: Stored ID (little endian)
    memcpy(payload, &storedID, 4);
    // Byte 4: Sender's own module CAN_ID
    payload[4] = CAN_ID;

    Serial.print("[SystemID] Broadcasting Stored ID (0x");
    Serial.print(storedID, HEX);
    Serial.print(") from module ");
    Serial.println(CAN_ID);

    can.sendBroadcast(SYSTEM_ID_EXCHANGE, payload, sizeof(payload));
}

bool SystemID::waitForConsensus(CanWrapper &can, uint32_t myStoredID, uint32_t timeout_ms) {
    uint32_t startTime = millis();
    _receivedCount = 0;
    
    Serial.print("[SystemID] Waiting for consensus (Timeout: ");
    Serial.print(timeout_ms);
    Serial.println("ms)...");

    // Loop until timeout or we have collected responses from other 2 modules (Max modules = 3)
    // Note: We always allow the timeout to process so other modules have enough time to boot and send
    while (millis() - startTime < timeout_ms) {
        uint8_t bc_id = 0;
        byte bc_data[8] = {0};
        
        if (can.readBroadcastMessage(&bc_id, bc_data)) {
            if (bc_id == SYSTEM_ID_EXCHANGE) {
                uint32_t peerID = 0;
                memcpy(&peerID, bc_data, 4);
                uint8_t peerSender = bc_data[4];

                // Ignore if it is our own broadcast echoed back
                if (peerSender == CAN_ID) {
                    continue;
                }

                // Check if we already received from this sender
                bool exists = false;
                for (uint8_t i = 0; i < _receivedCount; i++) {
                    if (_receivedSenders[i] == peerSender) {
                        // Update with latest ID in case it retried
                        _receivedIDs[i] = peerID;
                        exists = true;
                        break;
                    }
                }

                if (!exists && _receivedCount < MAX_MODULES) {
                    _receivedIDs[_receivedCount] = peerID;
                    _receivedSenders[_receivedCount] = peerSender;
                    _receivedCount++;
                    Serial.print("[SystemID] Received ID 0x");
                    Serial.print(peerID, HEX);
                    Serial.print(" from Peer Module ");
                    Serial.println(peerSender);
                }
            }
        }
        delay(5); // Don't hog CPU entirely
    }

    // Now assess consensus
    Serial.print("[SystemID] Consensus phase complete. Total peers responded: ");
    Serial.println(_receivedCount);

    // If no other peers responded at all, we are isolated
    if (_receivedCount == 0) {
        Serial.println("[SystemID] No peers found! We will proceed but consensus flag is false.");
        _verified = false;
        return false;
    }

    // Calculate matching ratio
    uint8_t matchingCount = 1; // Start with 1 (ourselves)
    for (uint8_t i = 0; i < _receivedCount; i++) {
        // If myStoredID is 0 (new/empty MCU), but the peers have a valid non-zero matching ID,
        // we will fail consensus here which is correct since we need manual override to trust the peer group.
        if (myStoredID != 0 && _receivedIDs[i] == myStoredID) {
            matchingCount++;
        }
    }

    uint8_t totalCount = _receivedCount + 1; // peers + ourselves
    float ratio = (float)matchingCount / totalCount;

    Serial.print("[SystemID] Consensus matching: ");
    Serial.print(matchingCount);
    Serial.print("/");
    Serial.print(totalCount);
    Serial.print(" (");
    Serial.print(ratio * 100.0f, 1);
    Serial.println("%)");

    if (ratio >= CONSENSUS_THRESHOLD) {
        _verified = true;
        Serial.println("[SystemID] Consensus VERIFIED!");
        return true;
    } else {
        _verified = false;
        Serial.println("[SystemID] Consensus FAILED!");
        return false;
    }
}

bool SystemID::isVerified() const {
    return _verified;
}
