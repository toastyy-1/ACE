// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#include "sim/inc/rocket.hpp"
#include <limits>

static constexpr double RTOL = 1e-9;
static constexpr double ATOL = 1e-6;

static constexpr double SAFE = 0.9;
static constexpr double FAC1 = 0.333;
static constexpr double FAC2 = 6.0;

// SOURCE: coefficient credit given to https://github.com/scipy/scipy/blob/main/scipy/integrate/_ivp/dop853_coefficients.py
// accessed: 10/21/2026

static constexpr double C[12] = {
    0.0,
    0.526001519587677318785587544488e-01,
    0.789002279381515978178381316732e-01,
    0.118350341907227396726757197510,
    0.281649658092772603273242802490,
    0.333333333333333333333333333333,
    0.25,
    0.307692307692307692307692307692,
    0.651282051282051282051282051282,
    0.6,
    0.857142857142857142857142857142,
    1.0
};

static constexpr double A[12][12] = {
    {},
    {5.26001519587677318785587544488e-2},
    {1.97250569845378994544595329183e-2, 5.91751709536136983633785987549e-2},
    {2.95875854768068491816892993775e-2, 0, 8.87627564304205475450678981324e-2},
    {2.41365134159266685502369798665e-1, 0, -8.84549479328286085344864962717e-1, 9.24834003261792003115737966543e-1},
    {3.7037037037037037037037037037e-2, 0, 0, 1.70828608729473871279604482173e-1, 1.25467687566822425016691814123e-1},
    {3.7109375e-2, 0, 0, 1.70252211019544039314978060272e-1, 6.02165389804559606850219397283e-2, -1.7578125e-2},
    {3.70920001185047927108779319836e-2, 0, 0, 1.70383925712239993810214054705e-1, 1.07262030446373284651809199168e-1, -1.53194377486244017527936158236e-2, 8.27378916381402288758473766002e-3},
    {6.24110958716075717114429577812e-1, 0, 0, -3.36089262944694129406857109825, -8.68219346841726006818189891453e-1, 2.75920996994467083049415600797e1, 2.01540675504778934086186788979e1, -4.34898841810699588477366255144e1},
    {4.77662536438264365890433908527e-1, 0, 0, -2.48811461997166764192642586468, -5.90290826836842996371446475743e-1, 2.12300514481811942347288949897e1, 1.52792336328824235832596922938e1, -3.32882109689848629194453265587e1, -2.03312017085086261358222928593e-2},
    {-9.3714243008598732571704021658e-1, 0, 0, 5.18637242884406370830023853209, 1.09143734899672957818500254654, -8.14978701074692612513997267357, -1.85200656599969598641566180701e1, 2.27394870993505042818970056734e1, 2.49360555267965238987089396762, -3.0467644718982195003823669022},
    {2.27331014751653820792359768449, 0, 0, -1.05344954667372501984066689879e1, -2.00087205822486249909675718444, -1.79589318631187989172765950534e1, 2.79488845294199600508499808837e1, -2.85899827713502369474065508674, -8.87285693353062954433549289258, 1.23605671757943030647266201528e1, 6.43392746015763530355970484046e-1}
};

static constexpr double B[12] = {
    5.42937341165687622380535766363e-2, 0, 0, 0, 0, 4.45031289275240888144113950566, 1.89151789931450038304281599044,
    -5.8012039600105847814672114227, 3.1116436695781989440891606237e-1, -1.52160949662516078556178806805e-1,
    2.01365400804030348374776537501e-1, 4.47106157277725905176885569043e-2
};

static constexpr double E3[12] = {
    5.42937341165687622380535766363e-2 - 0.244094488188976377952755905512, 0, 0, 0, 0,
    4.45031289275240888144113950566, 1.89151789931450038304281599044, -5.8012039600105847814672114227,
    3.1116436695781989440891606237e-1 - 0.733846688281611857341361741547, -1.52160949662516078556178806805e-1,
    2.01365400804030348374776537501e-1, 4.47106157277725905176885569043e-2 - 0.220588235294117647058823529412e-1
};

static constexpr double E5[12] = {
    0.1312004499419488073250102996e-1, 0, 0, 0, 0,
    -0.1225156446376204440720569753e+1, -0.4957589496572501915214079952, 0.1664377182454986536961530415e+1,
    -0.3503288487499736816886487290, 0.3341791187130174790297318841, 0.8192320648511571246570742613e-1,
    -0.2235530786388629525884427845e-1
};

// position and velocity
struct State {
    Vec3 r;
    Vec3 v;
    State operator+(const State& o) const { return {r + o.r, v + o.v}; }
    State operator*(double s)       const { return {r * s,   v * s};   }
};

// state
static void components(const State& s, double out[6]) {
    out[0] = s.r.x; out[1] = s.r.y; out[2] = s.r.z;
    out[3] = s.v.x; out[4] = s.v.y; out[5] = s.v.z;
}

// acceleration
Vec3 Rocket::dop853_accel(const Vec3& r_i, const Vec3& v_i, double t) {
    return kinematic_state(m_current, r_i, v_i, q_rocket, w, props, burn_time, t).accel;
}

/**
 * integrates position and velocity across one sim time step with DOP853
 * @param current_time simulation time
 */
void Rocket::update_dynamics_3DoF_DOP853(double current_time) {
    double t = current_time;
    const double t_end = current_time + consts::TIME_STEP;

    // try whole step first
    double h = t_end - t;

    State y = {r, v};
    State k[12];
    k[0] = {y.v, dop853_accel(y.r, y.v, t)};
    bool reject = false;

    for (;;) {
        // clamp step
        bool last = false;
        if (t + 1.01 * h > t_end) {
            h = t_end - t;
            last = true;
        }

        for (int i = 1; i < 12; i++) {
            State sum = {};
            for (int j = 0; j < i; j++) sum = sum + k[j] * A[i][j];
            State y_i = y + sum * h;
            k[i] = {
                y_i.v, 
                dop853_accel(y_i.r, y_i.v, t + C[i] * h)
            };

        }

        // 8th order solution and 5th and 3rd order error
        State dy, e5, e3;
        for (int i = 0; i < 12; i++) {
            dy = dy + k[i] * B[i];
            e5 = e5 + k[i] * E5[i];
            e3 = e3 + k[i] * E3[i];
        }
        State y_new = y + dy * h;

        // error
        double y0[6], y1[6], e5c[6], e3c[6];
        components(y, y0);
        components(y_new, y1);
        components(e5, e5c);
        components(e3, e3c);
        double err5 = 0, err3 = 0;
        for (int i = 0; i < 6; i++) {
            double sk = ATOL + RTOL * std::max(std::abs(y0[i]), std::abs(y1[i]));
            err5 += (e5c[i] / sk) * (e5c[i] / sk);
            err3 += (e3c[i] / sk) * (e3c[i] / sk);
        }
        double den = err5 + 0.01 * err3;
        if (den <= 0) den = 1;
        double err = h * err5 / std::sqrt(6 * den);

        // next step size
        double fac = std::pow(err, 1.0 / 8.0) / SAFE;
        double h_new = h / std::max(1.0 / FAC2, std::min(1.0 / FAC1, fac));

        // accept the step if the error is small
        if (err <= 1.0 || 0.1 * h <= std::abs(t) * std::numeric_limits<double>::epsilon()) {
            y = y_new;
            t += h;
            if (last) break;

            // first stage of next step
            k[0] = {y.v, dop853_accel(y.r, y.v, t)};
            if (reject) h_new = std::min(h_new, h);
            reject = false;
        }
        else {
            reject = true;
        }
        h = h_new;
    }

    r = y.r;
    v = y.v;
}
