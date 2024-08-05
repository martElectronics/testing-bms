#include "MART_CAN.h"

/**
 * Reads a message from the CAN bus if available.
 * It checks for the availability of a message using the interrupt pin.
 * If a message is available, it reads the message ID, length, and data bytes
 * into the DataIN structure. It also determines whether the message uses an
 * extended ID and if it is a Remote Request Frame (RRF).
 * return true if a message was successfully read, false otherwise.
 */
bool CAN_BUS::readBytes()
{
    bool ok = false;
    if (!digitalRead(_CAN.pinINT) && !config.simulating)
    {
        byte rxBuf[8];
        _CAN.readMsgBuf(&DataIN.dataRaw.id, &DataIN.dataRaw.size, DataIN.dataRaw.bytes); // Read data: len = data length, buf = data byte(s)
        if ((DataIN.dataRaw.id>0x7FF))
            DataIN.dataRaw.typeExtendedId = 1;
        else
            DataIN.dataRaw.typeExtendedId = 0;

        if ((DataIN.dataRaw.id & 0x40000000) == 0x40000000)
        {
            unsigned long mask = ~(1UL << 30);
            // Clear the bit at bitPosition
            DataIN.dataRaw.id &= mask;
            DataIN.dataRaw.rrf = true;
            DEBUG_PRINTLN("Received RRF");
        }
        else
            DataIN.dataRaw.rrf = false;

        ok = true;
    }
    else
    {
        ok = false;
    }
    return (ok || config.simulating);
}

/**
 * Writes a message to the CAN bus using the data in DataOUT structure.
 * It sends a message with the specified message ID, flag for extended ID,
 * message length, and data bytes. The status of the message send operation
 * is checked to ensure successful transmission.
 * @return true if the message was successfully sent, false otherwise.
 */
bool CAN_BUS::writeBytes()
{
    bool ok;
    byte sndStat = _CAN.sendMsgBuf(DataOUT.dataRaw.id, DataOUT.dataRaw.typeExtendedId, 8, DataOUT.dataRaw.bytes);
    if (sndStat == CAN_OK)
    {
        ok = true;
    }
    else
    {
        ok = false;
    }
    return (ok || config.simulating);
}

/**
 * Sends all packets in the DataOUT structure to the CAN bus.
 * Iterates over each packet in DataOUT and sends them individually.
 * If a message fails to send, it logs an error but continues sending
 * the remaining messages. The success status is updated accordingly.
 * return true if all messages were sent successfully, false if any failed.
 */
bool CAN_BUS::send()
{
    bool success = true;

    unsigned long currentTime = millis();

    //Send status data if the timer reaches PT and the ESP is configured accordingly
    if (((millis() - previousStatusIntervalTime) >= (intervalTime / 3)) && (config.sendStatusData))
    {
        numCurrentSamples++;
        setCANStatusData();
        previousStatusIntervalTime = millis();

        if (numCurrentSamples > 3)
        {
            // numRXPaqOK = 0;
            // numTXPaqOK = 0;
            // numTxPaqError = 0;
            // numCurrentSamples = 1;
        }
    }

    DataOUT.forEachPacket([this, &success, currentTime](CanPacketRawData &packet)
                          {
        // Initially assume the packet does not have a timer and is ready to send
        bool readyToSend = true;

        // Check for a timer associated with this packet
        for (const auto& timer : packetTimers) {
            if (timer.packetID == packet.id) {
                // Check if the current time is past the next scheduled send time
                if (currentTime < packet.nextSendTime) {
                    readyToSend = false; // Not yet time to send this packet
                }
                break; // Timer found, no need to check further
            }
        }

        if (readyToSend && !packet.WaitForRRF) {

            //Store the packet ID before the possible ID change if the packet is a RRF
            unsigned long idAux=packet.id;
            byte buf[8];
            // Copy data to buffer
            std::copy(std::begin(packet.bytes), std::end(packet.bytes), std::begin(buf));
           // this->printByteArray(buf, 8);
            if(packet.rrf)
            {
            unsigned long mask = 1UL << 30;

              // Set the bit at bitPosition
              packet.id |= mask;
            }
            // Attempt to send the packet
            if (_CAN.sendMsgBuf(packet.id, packet.size, buf) != CAN_OK) {
              //  this->printByteArray(buf,8);
                ERROR_PRINTLN("Error sending message");
                success = false; // Mark failure but continue sending the rest
                numTxPaqError++;
            } else {
              DEBUG_PRINTLN((String)"Packet sent ID = " + packet.id);
                packet.id=idAux;
                // Update the next send time for this packet if it has a timer
                for (auto& timer : packetTimers) {
                    if (timer.packetID == packet.id) {
                        
                        packet.nextSendTime = currentTime + timer.interval;
                        break;
                    }
                }
                numTXPaqOK++;
            }
        }
        else if (packet.WaitForRRF) {
            DEBUG_PRINTLN((String)"Packet not sent because it is waiting for a RRF ID = " + packet.id);
        } });

    runtimeTime = millis() - previousStatusRuntimeTime;

    return success;
}

/**
 * Looks for an specific packet in DataOUT and sends it if exists
 * If a message fails to send, it logs an error but continues sending
 * the remaining messages. The success status is updated accordingly.
 * @return true if all messages were sent successfully, false if any failed.
 */
bool CAN_BUS::send(unsigned long id)
{
    bool success = true;

    const CanPacketRawData *packet = DataOUT.getPacketById(id);
    DEBUG_PRINT((String) "Sending packet with id " + id);
    if (packet != nullptr)
    {
        byte len = packet->size;
        byte buf[8];

        // Copy data to buffer
        std::copy(std::begin(packet->bytes), std::end(packet->bytes), std::begin(buf));
        if (_CAN.sendMsgBuf(packet->id, packet->size, buf))
        {
            ERROR_PRINTLN("Error sending message");
            success = false; // Mark failure but continue sending the rest
        }
        else
        {
            DEBUG_PRINTLN(" sent OK");
        }
    }
    else
    {
        DEBUG_PRINTLN("NULLPTR");
        success = false;
    }

    return success;
}

bool CAN_BUS::sendRequestedRRF(unsigned long id)
{
    bool ok = true;
    auto outIds = getOutIdsByInId(id);
    if (outIds)
    {
        DEBUG_PRINTLN("Sending messages for OUTRRFids associated with INRRFid 0x");
        DEBUG_PRINTLN(id);
        for (unsigned long id : outIds.value())
        {
            if (!send(id))
                ok = false; // Call the send method for each OUTRRFid
        }
    }
    else
    {
        ok = false;
        DEBUG_PRINT("No OUTRRFids found for INRRFid 0x");
        DEBUG_PRINTLN(id);
    }
    // returns ok if all the ids that ere config using setRRFId are found and sent correctly
    return (ok || config.simulating);
}

/**
 * Receives messages from the CAN bus and stores them in DataIN.
 * This method repeatedly calls readBytes() to read any available CAN messages.
 * Each read message is added to the DataIN structure for later processing.
 */
void CAN_BUS::receive()
{
    previousStatusRuntimeTime = millis();
    if (readBytes() || config.simulating)
    {
        //Store packet in memory if is not in the IDs set by the filter or if are no ids stored
        if ((filterIDs.empty())||(std::binary_search(filterIDs.begin(), filterIDs.end(), DataIN.dataRaw.id)))
        {
            //Serial.println("ADDED");
            DataIN.addPacket(DataIN.dataRaw);
        }

         Serial.println((String) "Rx ID: " + DataIN.dataRaw.id);
        //  Respond to RRF if the option is enabled
        if (DataIN.dataRaw.rrf && config.respondToRRF)
        {      

            // Serial.println("Sending requested paquets of rrf");
            sendRequestedRRF(DataIN.dataRaw.id);
            if (config.autoRemoveRRFPacket)
            {
                DataIN.removePacket(DataIN.dataRaw.id);
            }
        }
        numRXPaqOK++;
    }
}

// Configures the RRF pairs
void CAN_BUS::setRRFId(unsigned long inId, unsigned long outId)
{
    // Search for an existing inId
    auto it = std::find_if(rrfIdsList.begin(), rrfIdsList.end(),
                           [inId](const RRFIds &rrfIds)
                           {
                               return !rrfIds.INRRFid.empty() && rrfIds.INRRFid[0] == inId;
                           });

    if (it != rrfIdsList.end())
    {
        // inId found, append outId to the existing OUTRRFid vector
        it->OUTRRFid.push_back(outId);
    }
    else
    {
        // inId not found, create a new RRFIds instance and insert it
        RRFIds newIds;
        newIds.INRRFid.push_back(inId);
        newIds.OUTRRFid.push_back(outId);
        // Find the correct position to insert to keep the list ordered
        auto insertPos = std::lower_bound(rrfIdsList.begin(), rrfIdsList.end(), inId,
                                          [](const RRFIds &rrfIds, unsigned long id)
                                          {
                                              return !rrfIds.INRRFid.empty() && rrfIds.INRRFid[0] < id;
                                          });
        rrfIdsList.insert(insertPos, newIds);
    }
}

std::optional<std::vector<unsigned long>> CAN_BUS::getOutIdsByInId(unsigned long inId)
{
    for (const auto &rrfIds : rrfIdsList)
    {
        // Check if the inId is in the INRRFid vector
        if (std::find(rrfIds.INRRFid.begin(), rrfIds.INRRFid.end(), inId) != rrfIds.INRRFid.end())
        {
            // If inId is found, return the associated OUTRRFid vector
            return rrfIds.OUTRRFid;
        }
    }
    return std::nullopt; // inId not found
}

bool CAN_BUS::searchOutId(unsigned long outId)
{
    bool ok;
    for (const auto &rrfIds : rrfIdsList)
    {
        // Check if the outId is in the OUTRRFid vector
        if (std::find(rrfIds.OUTRRFid.begin(), rrfIds.OUTRRFid.end(), outId) != rrfIds.OUTRRFid.end())
        {
            ok = true;
        }
        else
        {
            ok = false; // outId not found in any vector
        }
    }
    return (ok);
}

// Calculates and writes the masks and filters to the MCP2515 registers given a set of IDs
bool CAN_BUS::setFilters(const unsigned long ids[], unsigned size)
{
    for (int i = 0; i < size; i++)
    {
        filterIDs.push_back(ids[i]);
    }
    sort(filterIDs.begin(), filterIDs.end());
    return true;
}

void CAN_BUS::printFilters()
{
    configurator.printCalculatedValues();
}

void CAN_BUS::testFilters(const std::vector<uint16_t> &testIds)
{
    configurator.testFilters(testIds);
}

void CAN_BUS::setPacketTimer(unsigned long packetID, unsigned long time)
{
    // Check if the timer already exists and update it
    for (auto &timer : packetTimers)
    {
        if (timer.packetID == packetID)
        {
            timer.interval = time;
            return;
        }
    }

    // If not found, add a new timer
    packetTimers.push_back({packetID, time});
}

//** CAN STATUS **//
void CAN_BUS::setCANStatusData()
{
    int d0[2]; // runtimeTime,numTxPaqError
    int d1[2]; // numRXPaqOK,numTXPaqOK
    int d2[2] = {0, 0};
    d0[0] = runtimeTime;
    d0[1] = numTxPaqError / numCurrentSamples;
    d1[0] = numRXPaqOK / numCurrentSamples;
    d1[1] = numTXPaqOK / numCurrentSamples;

    DEBUG_PRINTLN("Offset: ");
    DEBUG_PRINTLN(statusPacketOffset);

    this->setPacket(statusPacketOffset, d0);
    this->setPacket(statusPacketOffset + 1, d1);
    this->setPacket(statusPacketOffset + 2, d2);
}

void CAN_BUS::getCANStatusData(unsigned _nodeid, int _d0[], int _d1[], int _d2[], bool &ok)
{

    int d0[2];
    int d1[2];
    int d2[2];
    if (_nodeid > 0)
    {
        unsigned long _statusPacketOffset = STATUS_START_MASTER_ID + (_nodeid - 1) * STATUS_NUM_PAQUETS;
        if (_nodeid != nodeID)
        {

            DEBUG_PRINTLN((String) "ID " + _nodeid + "d0 data");
            ok = getPacket(_statusPacketOffset, d0);
            if (ok)
            {
                getPacket(_statusPacketOffset + 1, d1);
                getPacket(_statusPacketOffset + 2, d2);
            }
        }
        else
        {
            d0[0] = runtimeTime;
            d0[1] = numTxPaqError;
            d1[0] = numRXPaqOK;
            d1[1] = numTXPaqOK;
            printArray(d0);
            ok = true;
        }
    }
    else
    {
        ok = false;
    }

    _d0[0] = d0[0];
    _d0[1] = d0[1];
    _d1[0] = d1[0];
    _d1[1] = d1[1];
    _d2[0] = d2[0];
    _d2[1] = d2[0];
}
bool CAN_BUS::getCANStatusData(unsigned _nodeid, int d[])
{
    bool ok;
    int d0[2]; // runtimeTime,numTxPaqError
    int d1[2]; // numRXPaqOK,numTXPaqOK
    int d2[2];
    getCANStatusData(_nodeid, d0, d1, d2, ok);
    d[0] = d0[0];
    d[1] = d0[1];
    d[2] = d1[0];
    d[3] = d1[1];
    d[4] = d2[0];
    d[5] = d2[1];
    return ok;
}

void CAN_BUS::printStatusData(unsigned _nodeID)
{
    bool ok;
    int d0[2]; // runtimeTime,numTxPaqError
    int d1[2]; // numRXPaqOK,numTXPaqOK
    int d2[2];
    getCANStatusData(_nodeID, d0, d1, d2, ok);
    if (!ok)
    {
        // Serial.println((String) "Status data for nodeId: " + _nodeID + "not found");
    }
    else
    {
        Serial.println((String) "NodeId: " + _nodeID + " data:");
        Serial.println((String) "runtimeTime: " + d0[0]);
        Serial.println((String) "numTxPaqError: " + d0[1]);
        Serial.println((String) "numRXPaqOK: " + d1[0]);
        Serial.println((String) "numTXPaqOK: " + d1[1]);
    }
}
void CAN_BUS::printReceivedIds()
{
    DataIN.printAllPacketsIDs();
}