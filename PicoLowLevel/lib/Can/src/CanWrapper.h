#ifndef CAN_WRAPPER_H
#define CAN_WRAPPER_H

#include "mcp2515.h"
#include "../../../include/mod_config.h"

/**
 * @class CanWrapper
 * @brief A wrapper class for the MCP2515 CAN controller.
 */
class CanWrapper {
public:
    /**
     * @brief Constructor that initializes the MCP2515 object with the provided parameters.
     * @param csPin Chip select pin for the MCP2515.
     * @param speed SPI speed for the MCP2515.
     * @param spi SPI interface to use.
     */
    CanWrapper(uint8_t csPin, uint32_t speed, SPIClass* spi) : mcp2515(csPin, speed, spi) {}

    /**
     * @brief Initializes the CAN module.
     */
    void begin();

    /**
     * @brief Sends a CAN message.
     * @param id The ID of the CAN message.
     * @param data Pointer to the data to be sent.
     * @param length Length of the data to be sent.
     * @return True if the message was sent successfully, false otherwise.
     */
    bool sendMessage(uint8_t id, const void* data, uint8_t length);

    /**
     * @brief Reads a CAN message.
     * @param id Pointer to store the ID of the received CAN message.
     * @param data Pointer to store the data of the received CAN message.
     * @return True if a message was read successfully, false otherwise.
     */
    bool readMessage(uint8_t* id, byte* data);

    /**
     * @brief Sends a broadcast CAN message (to all modules using prefix 0xFF).
     */
    bool sendBroadcast(uint8_t id, const void* data, uint8_t length);

    /**
     * @brief Reads a message from either own CAN_ID or broadcast.
     * @param id Pointer to store the message ID.
     * @param data Pointer to store message data.
     * @param senderModule Pointer to store sender's CAN_ID (optional).
     * @return True if a message was read.
     */
    bool readBroadcastMessage(uint8_t* id, byte* data, uint8_t* senderModule = nullptr);

private:
    MCP2515 mcp2515; ///< MCP2515 object for CAN communication.
};

#endif // CAN_WRAPPER_H