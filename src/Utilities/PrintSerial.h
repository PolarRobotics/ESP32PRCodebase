#pragma once

#ifndef PRINTSERIAL_H
#define PRINTSERIAL_H

#define GET_VARIABLE_NAME(Variable) (#Variable)

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
        float value1;
        float value2;
        float value3;
        float value4;
        float value5;
        const char* HEADER1;
        const char* HEADER2;
        const char* HEADER3;
        const char* HEADER4;
        const char* HEADER5; 
    public:
        static PrintSerial& getInstance() {
            static PrintSerial instance;
            return instance;
        }
        PrintSerial(const PrintSerial& obj) = delete; // delete copy constructor
        void operator=(PrintSerial const&)  = delete; // delete set operator
        void printCsvInfo(const std::vector<float>& values);
        void printDebugInfo();
};

#endif // PRINTSERIAL_H