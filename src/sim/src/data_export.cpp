#include "sim/inc/data_export.hpp"
#include <charconv>

static constexpr size_t FILE_BUFFER_SIZE = 1 << 20;

DataExport::DataExport(const std::string& filename, double interval) : interval(interval) {
    file = std::fopen(filename.c_str(), "wb");
    if (!file) {
        std::perror(filename.c_str());
        return;
    }
    std::setvbuf(file, nullptr, _IOFBF, FILE_BUFFER_SIZE);

    std::fputs("t,rx,ry,rz,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wx,wy,wz,m,m_fuel,thrust\n", file);
}

DataExport::~DataExport() {
    if (file) std::fclose(file);
}

void DataExport::write_row(double t, const Vec3& r, const Vec3& v, const Vec3& a, const Quat& q, const Vec3& w,
                           double m, double m_fuel, double thrust) {
    if (!file) return;

    // skip rows until the next interval (small tolerance since t accumulates float error each step)
    if (t < next_t - 1e-9) return;
    next_t += interval;
    if (next_t < t) next_t = t + interval; // jumped past several intervals (or first row), don't try to catch up

    const double vals[] = {
        t,
        r.x, r.y, r.z,
        v.x, v.y, v.z,
        a.x, a.y, a.z,
        q.w, q.x, q.y, q.z,
        w.x, w.y, w.z,
        m, m_fuel, thrust,
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
