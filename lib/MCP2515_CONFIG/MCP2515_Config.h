
#ifndef __MCP2515Configurator
#define __MCP2515Configurator
#include <Arduino.h>
#include <vector>

class MCP2515Configurator
{
private:
    // Calculates a common mask for a given set of CAN IDs
    uint32_t calculateMask(const std::vector<uint16_t> &ids)
    {
        uint16_t commonBits = 0xFFFF;
        uint16_t differingBits = 0;

        for (size_t i = 0; i < ids.size(); i++)
        {
            for (size_t j = i + 1; j < ids.size(); j++)
            {
                differingBits |= ids[i] ^ ids[j];
            }
        }

        commonBits &= ~differingBits;
        return static_cast<uint32_t>(commonBits) << 16;
    }

    // Assigns filters for a specific mask and group of CAN IDs
    void assignFilters(uint32_t mask, const std::vector<uint16_t> &ids, size_t maskIndex)
    {
        std::vector<uint32_t> assignedFilters;
        for (auto id : ids)
        {
            uint32_t filter = static_cast<uint32_t>(id) << 16;
            // Correct the assignment here to preserve the unique ID in the filter
            assignedFilters.push_back(filter);
        }
        filters[maskIndex] = assignedFilters;
    }

    // Formats a standard 11-bit CAN mask for use with MCP2515
    uint32_t formatStandardCANMask(uint32_t mask)
    {
        return (mask & 0x7FF) << 21; // Aligning the 11-bit mask in the 16-bit segment
    }

public:
    // Public member variables to store masks and filters
    std::vector<uint32_t> masks, masks_shifted;
    std::vector<std::vector<uint32_t>> filters, filters_shifted;

    bool calculateFiltersAndMasks(const std::vector<uint16_t> &ids)
    {
        std::vector<uint16_t> group1, group2;
        divideIntoGroups(ids, group1, group2);

        // Calculate masks for each group
        uint32_t mask1 = calculateMask(group1);
        uint32_t mask2 = calculateMask(group2);
        masks = {mask1, mask2};

        // Assign filters for each mask
        filters.resize(2);
        assignFilters(mask1, group1, 0);
        assignFilters(mask2, group2, 1);
        return ((filters[0].size() + filters[1].size()) < 7);
    }

    // Tests a set of CAN IDs against the configured filters to check if they are accepted
    void testFilters(const std::vector<uint16_t> &testIds)
    {

        for (auto id : testIds)
        {
            bool isAccepted = false;
            uint32_t extendedId = static_cast<uint32_t>(id) << 16; // Adjust for MCP2515 format

            // Check against the first mask and its filters
            for (auto filter : filters[0])
            {
                if ((extendedId & masks[0]) == (filter & masks[0]))
                {
                    isAccepted = true;
                    break;
                }
            }

            // Check against the second mask and its filters if not already accepted
            if (!isAccepted)
            {
                for (auto filter : filters[1])
                {
                    if ((extendedId & masks[1]) == (filter & masks[1]))
                    {
                        isAccepted = true;
                        break;
                    }
                }
            }

            Serial.print("ID 0x");
            Serial.print(id, HEX);
            Serial.print(isAccepted ? " is accepted by the filter." : " is blocked by the filter.");
            Serial.println();
        }
    }

    void printCalculatedValues() const
    {
        int filterIndex = 0;
        for (size_t i = 0; i < masks.size(); ++i)
        {
            Serial.print("Mask ");
            Serial.print(i);
            Serial.print(": 0x");
            Serial.println(masks_shifted[i], HEX);

            // Initialize filters associated with this mask
            for (size_t j = 0; j < filters[i].size(); ++j)
            {
                Serial.print("Filter ");
                Serial.print(j);
                Serial.print(": 0x");
                Serial.println(filters_shifted[i][j], HEX);
                filterIndex++;
            }
            Serial.println();
        }
    }

    // Divides CAN IDs into two groups based on some criteria
    bool divideIntoGroups(const std::vector<uint16_t> &ids,
                          std::vector<uint16_t> &group1,
                          std::vector<uint16_t> &group2)
    {
        std::vector<uint16_t> sortedIds = ids;
        std::sort(sortedIds.begin(), sortedIds.end());

        group1.assign(sortedIds.begin(), sortedIds.begin() + 2);
        group2.assign(sortedIds.begin() + 2, sortedIds.end());

        return true;
    }

    // Function to shift the contents of a vector<uint32_t> by n positions
    std::vector<uint32_t> shiftVector(const std::vector<uint32_t> &vec, int n)
    {
        std::vector<uint32_t> shiftedVec;
        for (auto val : vec)
        {
            shiftedVec.push_back(val >> n); // Right shift each element by n positions
        }
        return shiftedVec;
    }

    // Function to shift the contents of a vector<vector<uint32_t>> by n positions
    std::vector<std::vector<uint32_t>> shiftNestedVector(const std::vector<std::vector<uint32_t>> &nestedVec, int n)
    {
        std::vector<std::vector<uint32_t>> shiftedNestedVec;
        for (const auto &innerVec : nestedVec)
        {
            shiftedNestedVec.push_back(shiftVector(innerVec, n)); // Apply shiftVector to each inner vector
        }
        return shiftedNestedVec;
    }

    // Shifts all masks and filters by a given number of bit positions
    void shiftValues(int n)
    {
        masks_shifted = shiftVector(masks, n);
        filters_shifted = shiftNestedVector(filters, n);
    }
};
#endif