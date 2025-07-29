#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// --- Corrected and Final CRC Implementation ---

// CRC16 lookup table from sluc682/source/bq79606.c
// This is the definitive reference for the BQ79606A-Q1 CRC calculation.
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/**
 * @brief Calculates the CRC for BQ79606A-Q1 communication frames.
 *
 * @note This is a direct adaptation of the `CRC16` function from the official
 * Texas Instruments sample code (sluc682/source/bq79606.c). It correctly
 * generates the CRC values for all examples in the software reference guide.
 *
 * @param pBuf Pointer to the data array.
 * @param nLen The number of bytes in the data array.
 * @return The calculated 16-bit CRC value.
 */
uint16_t bq79606_calculate_crc(const uint8_t *pBuf, int nLen) {
    uint16_t wCRC = 0xFFFF;
    int i;

    for (i = 0; i < nLen; i++) {
        wCRC = crc16_table[(wCRC ^ pBuf[i]) & 0xFF] ^ (wCRC >> 8);
    }

    // The TI sample code has a byte-swapped CRC compared to the documentation.
    // The documentation examples are correct, so we must swap the bytes of the result.
    return (wCRC >> 8) | (wCRC << 8);
}

/**
 * @brief Verifies the CRC of a received frame from the BQ79606A-Q1.
 *
 * @param received_frame Pointer to the complete received frame (data + CRC).
 * @param frame_length The total length of the received frame in bytes.
 * @return True if the CRC is correct, false otherwise.
 */
bool bq79606_verify_crc(const uint8_t *received_frame, int frame_length) {
    if (frame_length < 2) {
        return false; // Frame is too short
    }
    
    // To verify, we calculate the CRC on the data part only...
    uint16_t calculated_crc = bq79606_calculate_crc(received_frame, frame_length - 2);
    
    // ...and compare it to the received CRC bytes.
    uint16_t received_crc = ((uint16_t)received_frame[frame_length - 2] << 8) | received_frame[frame_length - 1];

    return (calculated_crc == received_crc);
}


// --- Testbench ---

// Structure to hold test cases from the datasheet
typedef struct {
    const char* description;
    uint8_t data[32];
    int data_len;
    uint16_t expected_crc;
} CrcTestCase;

int main() {
    printf("--- BQ79606A-Q1 CRC Function Testbench ---\n\n");

    // Test cases extracted from BQ79606A-Q1 Software Design Reference (SLVA970E)
    CrcTestCase test_cases[] = {
        {"Table 1: Single Device Read", {0x80, 0x00, 0x02, 0x15, 0x0B}, 5, 0xCB49},
        {"Table 2: Single Device Write", {0x93, 0x00, 0x01, 0x00, 0x02, 0xB7, 0x78, 0xBC}, 8, 0x9A8C},
        {"Table 3: Stack Read", {0xA0, 0x02, 0x15, 0x0B}, 4, 0xCCB3},
        {"Table 4: Stack Write", {0xB3, 0x01, 0x00, 0x02, 0xB7, 0x78, 0xBC}, 7, 0x0A35},
        {"Table 5: Broadcast Read", {0xC0, 0x02, 0x15, 0x0B}, 4, 0xD2B3},
        {"Table 6: Broadcast Write", {0xD3, 0x01, 0x00, 0x02, 0xB7, 0x78, 0xBC}, 7, 0x336A},
        {"Shutdown Command", {0xD0, 0x01, 0x05, 0x08}, 4, 0x6BB2},
        {"Auto-Address Dummy Write", {0xD0, 0x01, 0x1D, 0x00}, 4, 0x6074},
    };

    int num_tests = sizeof(test_cases) / sizeof(CrcTestCase);
    int passed_count = 0;

    for (int i = 0; i < num_tests; i++) {
        printf("--- Test Case %d: %s ---\n", i + 1, test_cases[i].description);

        // --- Part 1: Test the calculation function ---
        uint16_t calculated_crc = bq79606_calculate_crc(test_cases[i].data, test_cases[i].data_len);
        
        printf("Data: ");
        for(int j=0; j < test_cases[i].data_len; j++) {
            printf("0x%02X ", test_cases[i].data[j]);
        }
        printf("\n");
        
        printf("Expected CRC:   0x%04X\n", test_cases[i].expected_crc);
        printf("Calculated CRC: 0x%04X\n", calculated_crc);

        bool calc_passed = (calculated_crc == test_cases[i].expected_crc);
        if (calc_passed) {
            printf("Calculation Test: PASSED\n");
        } else {
            printf("Calculation Test: FAILED\n");
        }

        // --- Part 2: Test the verification function ---
        uint8_t full_frame[34];
        memcpy(full_frame, test_cases[i].data, test_cases[i].data_len);
        full_frame[test_cases[i].data_len] = (test_cases[i].expected_crc >> 8) & 0xFF;   // CRC MSB
        full_frame[test_cases[i].data_len + 1] = test_cases[i].expected_crc & 0xFF; // CRC LSB
        
        bool verify_passed = bq79606_verify_crc(full_frame, test_cases[i].data_len + 2);
        
        if (verify_passed) {
            printf("Verification Test (Good Frame): PASSED\n");
        } else {
            printf("Verification Test (Good Frame): FAILED\n");
        }
        
        // --- Part 3: Test verification with a corrupted frame ---
        full_frame[0] ^= 0x01; // Introduce a single-bit error in the first byte
        bool verify_corrupt_passed = bq79606_verify_crc(full_frame, test_cases[i].data_len + 2);
        
        if (!verify_corrupt_passed) {
            printf("Verification Test (Corrupt Frame): PASSED (Correctly detected error)\n");
        } else {
            printf("Verification Test (Corrupt Frame): FAILED (Did not detect error)\n");
        }

        if (calc_passed && verify_passed && !verify_corrupt_passed) {
            passed_count++;
        }
        printf("\n");
    }

    printf("--- Test Summary ---\n");
    printf("Passed %d out of %d test cases.\n", passed_count, num_tests);
    
    if (passed_count == num_tests) {
        printf("All CRC tests passed successfully!\n");
    } else {
        printf("Some CRC tests failed. Please review the output.\n");
    }

    return 0;
}
