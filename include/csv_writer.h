//
// Created by yan on 5/16/18.
//

#ifndef PROJECT_CSV_WRITER_H
#define PROJECT_CSV_WRITER_H
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <iterator>
#include <string>
#include <algorithm>

/*
 * A class to create and write data in a csv file.
 */
class CSVWriter
{
    int linesCount;
    std::string delimeter = ",";
public:
    std::string fileName;

    CSVWriter():delimeter(","), linesCount(0){};
    CSVWriter(std::string filename, std::string delm = ",") :
            fileName(filename), delimeter(delm), linesCount(0) {};
    /*
     * Member function to store a range as comma seperated value
     */
    template<typename T>
    void addDatainRow(T first, T last)
    {
        std::fstream file;
        // Open the file in truncate mode if first line else in Append Mode
        file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));

        // Iterate over the range and add each lement to file seperated by delimeter.
        for (; first != last; )
        {
            file << *first;
            if (++first != last)
                file << delimeter;
        }
        file << "\n";
        linesCount++;

        // Close the file
        file.close();
    }

    template<typename T>
    void addDatainCol(T first, T last)
    {
        std::fstream file;
        // Open the file in truncate mode if first line else in Append Mode
        file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));

        // Iterate over the range and add each lement to file seperated by delimeter.
        for (; first != last; )
        {
            file << *first;
            if (++first != last)
                file << delimeter;
            file << "\n";
            linesCount++;
        }

        // Close the file
        file.close();
    }
};


#endif //PROJECT_CSV_WRITER_H
