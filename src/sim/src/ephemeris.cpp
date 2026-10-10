// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#include "sim/inc/ephemeris.hpp"
#include "sim_constants.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>

/**
 * @brief reads the sun, moon, and earth segments covering the epoch from a DE440 kernel
 * @param path SPK kernel to read
 * @param epoch_utc UTC time at sim t = 0, "YYYY-MM-DDTHH:MM:SS"
 * @return false if the kernel or epoch can't be used
 */
bool Ephemeris::load(const std::string& path, const std::string& epoch_utc) {
    int y, mo, d, h, mi;
    double s;
    if (std::sscanf(epoch_utc.c_str(), "%d-%d-%dT%d:%d:%lf", &y, &mo, &d, &h, &mi, &s) != 6) {
        std::cerr << "config error: epoch '" << epoch_utc << "' must be YYYY-MM-DDTHH:MM:SS\n";
        return false;
    }

    // days since J2000 (2000-01-01 12:00), gregorian calendar
    if (mo <= 2) { y--; mo += 12; }
    int a = y / 100;
    double days = std::floor(365.25 * (y + 4716)) + std::floor(30.6001 * (mo + 1)) + d + 2 - a + a / 4 - 1524.5
                + (h + mi / 60.0 + s / 3600.0) / 24.0 - 2451545.0;

    // TDB ~= UTC + 37 leap seconds (current since 2017) + 32.184 s, and UT1 ~= UTC
    et0 = days * 86400.0 + 69.184;

    // earth rotation angle at the epoch, the sim's ECI +x is greenwich at t = 0
    double era = consts::TAU * (0.7790572732640 + 1.00273781191135448 * days);
    cos_era = std::cos(era);
    sin_era = std::sin(era);

    std::ifstream f(path, std::ios::binary);
    char file_rec[1024];
    if (!f.read(file_rec, sizeof(file_rec))) {
        std::cerr << "config error: could not read ephemeris '" << path << "', rebuild to download it\n";
        return false;
    }

    // DAF file record, SPK summaries are 2 doubles + 6 ints
    int nd, ni, fward;
    std::memcpy(&nd, file_rec + 8, 4);
    std::memcpy(&ni, file_rec + 12, 4);
    std::memcpy(&fward, file_rec + 76, 4);
    if (std::memcmp(file_rec, "DAF/SPK ", 8) != 0 || std::memcmp(file_rec + 88, "LTL-IEEE", 8) != 0 || nd != 2 || ni != 6) {
        std::cerr << "config error: '" << path << "' is not a little endian SPK kernel\n";
        return false;
    }

    // summary records are a linked list, each holds up to 25 summaries after next/prev/count
    Segment* wanted[] = {&emb, &sun_ssb, &moon_emb, &earth_emb};
    double rec[128];
    for (int r = fward; r > 0; r = (int)rec[0]) {
        f.seekg((std::streamoff)(r - 1) * 1024);
        if (!f.read((char*)rec, sizeof(rec))) break;

        int nsum = std::min((int)rec[2], 25);
        for (int i = 0; i < nsum; i++) {
            const double* sum = rec + 3 + i * 5;
            int ids[6]; // target, center, frame, type, first word, last word
            std::memcpy(ids, sum + 2, sizeof(ids));
            if (et0 < sum[0] || et0 > sum[1] || ids[2] != 1 || ids[3] != 2) continue; // need ICRF type 2 covering the epoch

            for (Segment* seg : wanted) {
                if (seg->target != ids[0] || seg->center != ids[1] || !seg->data.empty()) continue;

                // segment words are the records followed by init, intlen, rsize, n
                int count = ids[5] - ids[4] + 1;
                if (count < 4) break;
                std::vector<double> words(count);
                f.seekg((std::streamoff)(ids[4] - 1) * 8);
                if (!f.read((char*)words.data(), words.size() * 8)) break;

                const double* tail = words.data() + words.size() - 4;
                int rsize = (int)tail[2], n = (int)tail[3];
                if (rsize < 5 || (rsize - 2) % 3 != 0 || (size_t)rsize * n + 4 != words.size()) break;

                words.resize((size_t)rsize * n);
                seg->init = tail[0];
                seg->intlen = tail[1];
                seg->rsize = rsize;
                seg->n = n;
                seg->data = std::move(words);
            }
        }
    }

    for (Segment* seg : wanted) {
        if (seg->data.empty()) {
            std::cerr << "config error: '" << path << "' has no usable segment for body " << seg->target
                      << " at epoch " << epoch_utc << "\n";
            return false;
        }
    }
    return true;
}

/**
 * @brief moon position relative to the earth
 * @param t sim time (s)
 * @return position in sim ECI (m)
 */
Vec3 Ephemeris::moon(double t) const {
    double et = et0 + t;
    return to_eci(moon_emb.position(et) - earth_emb.position(et));
}

/**
 * @brief sun position relative to the earth
 * @param t sim time (s)
 * @return position in sim ECI (m)
 */
Vec3 Ephemeris::sun(double t) const {
    double et = et0 + t;
    return to_eci(sun_ssb.position(et) - emb.position(et) - earth_emb.position(et));
}

/**
 * @brief evaluates the chebyshev record covering et
 * @param et TDB seconds past J2000
 * @return position (km, ICRF)
 */
Vec3 Ephemeris::Segment::position(double et) const {
    int i = std::clamp((int)std::floor((et - init) / intlen), 0, n - 1);
    const double* rec = data.data() + (size_t)i * rsize;
    double x = (et - rec[0]) / rec[1]; // mid and half length of the record, x is in [-1, 1]
    int ncoef = (rsize - 2) / 3;

    double p[3];
    for (int k = 0; k < 3; k++) {
        const double* c = rec + 2 + k * ncoef;
        double t0 = 1, t1 = x, sum = c[0];
        for (int j = 1; j < ncoef; j++) {
            sum += c[j] * t1;
            double t2 = 2 * x * t1 - t0;
            t0 = t1;
            t1 = t2;
        }
        p[k] = sum;
    }
    return {p[0], p[1], p[2]};
}

/**
 * @brief rotates an ICRF position into the sim's ECI frame (precession and nutation ignored)
 * @param p_km ICRF position (km)
 * @return sim ECI position (m)
 */
Vec3 Ephemeris::to_eci(const Vec3& p_km) const {
    return Vec3{cos_era * p_km.x + sin_era * p_km.y, -sin_era * p_km.x + cos_era * p_km.y, p_km.z} * consts::KM_TO_M;
}
