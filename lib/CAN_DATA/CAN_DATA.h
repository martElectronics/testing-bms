//********MART CAN DATA STORAGE LIBRARY
#ifndef CANDATA_H
#define CANDATA_H
#include <Arduino.h>
#include "common.h"

class CAN_DATA
{
private:
    std::vector<CanPacketRawData> packets;   // Vector to store CAN packets
    CanPacketRawData *lastAddedPacket;       // Pointer to the last added packet
    std::vector<unsigned long> removableIds; // List of IDs whose packets should be auto-removed after being read
    bool allIdsRemovable;                    // Flag to indicate if all IDs are removable
public:
    CAN_DATA()
    {
        allIdsRemovable = false;
    }

    CanPacketRawData dataRaw; // Raw data of CAN packet

    // Adds a CAN packet to the storage, keeping packets sorted by their ID
    void addPacket(const CanPacketRawData &packet)
    {
        // Use std::find_if to check if a packet with the same id already exists
        auto it = std::find_if(packets.begin(), packets.end(), [&packet](const CanPacketRawData &a)
                               { return a.id == packet.id; });

        if (it != packets.end())
        {
            // If a packet with the same id is found, update its information
            it->size = packet.size;
            std::copy(std::begin(packet.bytes), std::end(packet.bytes), std::begin(it->bytes));
            lastAddedPacket = &(*it); // Update the pointer to the last updated packet
        }
        else
        {
            // If no packet with the same id exists, add the new packet in sorted order
            auto insertIt = std::lower_bound(packets.begin(), packets.end(), packet,
                                             [](const CanPacketRawData &a, const CanPacketRawData &b)
                                             {
                                                 return a.id < b.id;
                                             });
            auto insertedIt = packets.insert(insertIt, packet); // Insert and get iterator to the new element
            lastAddedPacket = &(*insertedIt);                   // Update the pointer to the last added packet
        }
    }

    
    // Removes a CAN packet from the storage by its ID
    void removePacket(unsigned long id)
    {
        packets.erase(std::remove_if(packets.begin(), packets.end(),
                                     [id](const CanPacketRawData &packet)
                                     {
                                         return packet.id == id;
                                         Serial.print("Removed id=");
                                         Serial.println(packet.id);
                                     }),
                      packets.end());
    }

    // Retrieves a packet by its ID and removes it if its ID is in the removable list or if all IDs are marked as removable
    const CanPacketRawData *getPacketById(unsigned long id)
    {
        auto it = std::find_if(packets.begin(), packets.end(),
                               [id](const CanPacketRawData &packet)
                               { return packet.id == id; });
        if (it != packets.end())
        {
            CanPacketRawData *foundPacket = &(*it);
            // Check if this ID should be auto-removed or if all IDs are removable
            if (allIdsRemovable || std::find(removableIds.begin(), removableIds.end(), id) != removableIds.end())
            {
                packets.erase(it); // Remove the packet
            }
            return foundPacket;
        }
        else
        {
            return nullptr; // Packet not found
        }
    }

    // Executes a provided function on each packet
    template <typename Func>
    void forEachPacket(Func func)
    {
        for (auto &packet : packets)
        {
            func(packet);
        }
    }
    // Method to add an array of IDs to the removable IDs list
    void setRemovableIds(const unsigned long *ids, size_t size)
    {
        for (size_t i = 0; i < size; ++i)
        {
            unsigned long id = ids[i];
            if (std::find(removableIds.begin(), removableIds.end(), id) == removableIds.end())
            {
                removableIds.push_back(id);
            }
        }
        allIdsRemovable = false; // Reset the allIdsRemovable flag
    }

    // Overloaded method to handle the ALL condition
    void setRemovableIds()
    {
        allIdsRemovable = true;
        removableIds.clear(); // Clear specific IDs as all are now removable
    }

    // Prints the latest packet's details to the serial monitor
    void printLastPacket() const
    {
        Serial.println();
        Serial.print((String) "ID = " + dataRaw.id + " SIZE = " + dataRaw.size + " DATA = ");
        for (unsigned i = 0; i < dataRaw.size; i++)
        {
            Serial.print((String)dataRaw.bytes[i] + " ");
            Serial.println();
        }
    }

    // Method to print all stored CAN packets
    void printAllPackets() const
    {
        unsigned cont = 1;
        if (packets.empty())
        {
            Serial.println("No packets stored.");
            return;
        }

        for (const auto &packet : packets)
        {
            Serial.println((String) "Packet " + cont);
            Serial.println("-------------");
            Serial.print("ID = ");
            Serial.println(packet.id); // Assuming ID is hexadecimal
            Serial.print("Extended ID: ");
            Serial.println(packet.typeExtendedId ? "Yes" : "No");
            Serial.print("RRF: ");
            Serial.println(packet.rrf ? "Yes" : "No");
            Serial.print("Size = ");
            Serial.println(packet.size);
            Serial.print("Data = ");

            for (unsigned i = 0; i < packet.size; i++)
            {
                Serial.print(packet.bytes[i], HEX);
                Serial.print(" ");
            }

            Serial.println(); // New line after printing each packet
        }
    }

    void printAllPacketsIDs() const
    {
        unsigned cont = 1;
        if (packets.empty())
        {
            Serial.println("No packets stored.");
            return;
        }
        else
        {
            for (const auto &packet : packets)
            {
                Serial.println((String) "Packet " + cont);
                Serial.print("ID = ");
                Serial.println(packet.id); // Assuming ID is hexadecimal
                // Serial.print("Extended ID: ");
                // Serial.println(packet.typeExtendedId ? "Yes" : "No");
                Serial.println(); // New line after printing each packet
                cont++;
            }
        }
    }
};

#endif
