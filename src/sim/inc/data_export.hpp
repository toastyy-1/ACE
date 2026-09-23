#pragma once
#include "types.hpp"
#include <cstdio>
#include <string>

// one CSV row of tracked flight data (vectors are ECI unless noted)
struct ExportRow {
    double t = 0;
    Vec3 r, v, a;
    Quat q;
    Vec3 w;
    double m = 0, m_fuel = 0, thrust = 0;
    Vec3 g;           // gravitational acceleration
    Vec3 drag;        // drag acceleration
    Vec3 thrust_a;    // thrust acceleration
    Vec3 a_spec;      // specific force, body frame
    double altitude = 0;
    double mach = 0;
    double dyn_pressure = 0;
    double aoa = 0;   // angle of attack (rad)
    double z_cm = 0;  // CoM from the active stage's aft edge (m)
    double z_cp = 0;  // CoP from the active stage's aft edge (m)
    int stage = 0;    // active stage index
};

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
    void write_row(const ExportRow& row);

    private:

    // the file the data is writing to (rows are buffered in memory and written in large chunks)
    FILE* file = nullptr;

    double interval = 0.0; // sim seconds between rows
    double next_t = 0.0;   // sim time the next row is due
};
