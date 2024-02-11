#include "communicator.h"
#include "payloads.h"

void Communicator::setSerialTransfer(SerialTransfer* serialTransfer) {
    this->serialTransfer = serialTransfer;
}

Communicator& Communicator::getInstance() {
    static Communicator instance;
    return instance;
}

Communicator::Communicator() = default;
Communicator::~Communicator() = default;

// template<typename T>
// void Communicator::sendPacket(const T& packet) {
//     if (this->serialTransfer) {
//         const uint16_t packetSize = this->serialTransfer->txObj(packet);
//         this->serialTransfer->sendData(packetSize);
//     }
// }
