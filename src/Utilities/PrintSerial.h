#pragma once

#ifndef PRINTSERIAL_H
#define PRINTSERIAL_H

#include <PolarRobotics.h>
#include <vector>

/**
 * @author Corbin Hibler
 * @date 2024-02-12
 * @brief Prints information to serial in various formats
 */
class PrintSerial {
    private:
        PrintSerial();
        std::vector<float> serialValues;
        std::vector<String> serialHeaders;
    public:
        static PrintSerial& getInstance() {
            static PrintSerial instance;
            return instance;
        }
        PrintSerial(const PrintSerial& obj) = delete; // delete copy constructor
        void operator=(PrintSerial const&)  = delete; // delete set operator
        void printDebugInfo();
        void printCsvInfo(const std::vector<float>& values, const std::vector<String>& headers);
};

#endif // PRINTSERIAL_H