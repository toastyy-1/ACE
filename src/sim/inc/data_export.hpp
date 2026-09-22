#pragma once
#include "types.hpp"
#include <cstdio>
#include <string>

// opens a CSV on construction, writes a row per write_row call, and closes the file on destruction
class DataExport {
    public:

    // opens the file and writes the header row
    // interval is the sim seconds between written rows (0 = write every call)
    DataExport(const std::string& filename, double interval);

    // on death, flushes and closes the file
    ~DataExport();

    // can't be copied (two objects would close the same file)
    DataExport(const DataExport&) = delete;
    DataExport& operator=(const DataExport&) = delete;

    // writes a new row to the CSV file containing tracked data
    // call it every time step; rows that land before the next interval are skipped
    void write_row(double t, const Vec3& r, const Vec3& v, const Vec3& a, const Quat& q, const Vec3& w,
                   double m, double m_fuel, double thrust);

    private:

    // the file the data is writing to (rows are buffered in memory and written in large chunks)
    FILE* file = nullptr;

    double interval = 0.0; // sim seconds between rows
    double next_t = 0.0;   // sim time the next row is due
};
