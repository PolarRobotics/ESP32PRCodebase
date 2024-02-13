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
        ~PrintSerial();
        float value1;
        float value2;
        float value3;
        float value4;
        float value5;
        char* header1;
        char* header2;
        char* header3;
        char* header4;
        char* header5; 
    protected:
        static PrintSerial* instance;
    public:
        static PrintSerial& getInstance();
        void printCsvInfo(const std::vector<float>& values);
        void printDebugInfo();
};

#endif // PRINTSERIAL_H