#include "sim/inc/data_export.hpp"
#include <charconv>

static constexpr size_t FILE_BUFFER_SIZE = 1 << 20;

/**
 * @brief
 * @param filename name of the file to export
 * @param interval how often (in seconds of sim time) a new line is expected to export
 */
DataExport::DataExport(const std::string& filename, double interval) : interval(interval) {
    file = std::fopen(filename.c_str(), "wb");
    if (!file) {
        std::perror(filename.c_str());
        return;
    }
    std::setvbuf(file, nullptr, _IOFBF, FILE_BUFFER_SIZE);

    std::fputs("t,rx,ry,rz,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wx,wy,wz,m,m_fuel,thrust,"
               "gx,gy,gz,dragx,dragy,dragz,thrust_ax,thrust_ay,thrust_az,a_spec_x,a_spec_y,a_spec_z,"
               "altitude,mach,dyn_pressure,aoa,z_cm,z_cp,stage\n", file);
}

/**
 * @brief destructor for data export
 */
DataExport::~DataExport() {
    if (file) std::fclose(file);
}

/**
 * @brief
 * @param row the row of which a new line should be written
 */
void DataExport::write_row(const ExportRow& row) {
    if (!file) return;

    // skip rows until the next interval (small tolerance since t accumulates float error each step)
    double t = row.t;
    if (t < next_t - 1e-9) return;
    next_t += interval;
    if (next_t < t) next_t = t + interval; // jumped past several intervals (or first row), don't try to catch up

    const double vals[] = {
        t,
        row.r.x, row.r.y, row.r.z,
        row.v.x, row.v.y, row.v.z,
        row.a.x, row.a.y, row.a.z,
        row.q.w, row.q.x, row.q.y, row.q.z,
        row.w.x, row.w.y, row.w.z,
        row.m, row.m_fuel, row.thrust,
        row.g.x, row.g.y, row.g.z,
        row.drag.x, row.drag.y, row.drag.z,
        row.thrust_a.x, row.thrust_a.y, row.thrust_a.z,
        row.a_spec.x, row.a_spec.y, row.a_spec.z,
        row.altitude, row.mach, row.dyn_pressure, row.aoa,
        row.z_cm, row.z_cp,
        static_cast<double>(row.stage),
    };

    char buf[sizeof(vals) / sizeof(vals[0]) * 32];
    char* p = buf;
    for (double x : vals) {
        p = std::to_chars(p, buf + sizeof(buf), x).ptr;
        *p++ = ',';
    }
    p[-1] = '\n';

    std::fwrite(buf, 1, p - buf, file);
}
