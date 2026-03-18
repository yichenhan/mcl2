#include "mcl/mcl_engine.hpp"
#include "distance_localization.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

using namespace mcl;
using namespace distance_loc;

static void perfect_readings(double x, double y, double heading_deg,
                             const MCLConfig& cfg, double out[4]) {
    Vec2 pos = { x, y };
    for (int i = 0; i < 4; i++) {
        double d = ray_distance_with_offset(pos, heading_deg,
                                            cfg.sensors[i].offset,
                                            cfg.sensors[i].angle_deg);
        out[i] = std::isfinite(d) ? d : -1.0;
    }
}

static void check_particle_scoring(const Particle& p, double heading_deg,
                                   const MCLConfig& cfg, const double readings[4]) {
    Vec2 pos = { (double)p.x, (double)p.y };
    double max_err = cfg.max_sensor_error;
    const char* names[] = {"L", "R", "F", "B"};
    int inliers = 0;
    printf("    particle (%.1f,%.1f) w=%.6f: ", p.x, p.y, p.weight);
    for (int i = 0; i < 4; i++) {
        if (readings[i] < 0) continue;
        double predicted = ray_distance_with_offset(pos, heading_deg,
                                                    cfg.sensors[i].offset,
                                                    cfg.sensors[i].angle_deg);
        double err = std::isfinite(predicted) ? std::fabs(predicted - readings[i]) : 999.0;
        bool ok = err <= max_err;
        if (ok) inliers++;
        printf("%s:pred=%.1f read=%.1f err=%.1f%s  ", names[i],
               std::isfinite(predicted) ? predicted : -1.0,
               readings[i], err, ok ? "*" : "");
    }
    printf("inliers=%d\n", inliers);
}

int main() {
    const double kTrueX = 20.0;
    const double kTrueY = 30.0;
    const double kTrueHeading = 45.0;

    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    printf("Truth: (%.1f, %.1f) heading=%.1f\n", kTrueX, kTrueY, kTrueHeading);
    printf("Readings: L=%.2f R=%.2f F=%.2f B=%.2f\n",
           readings[0], readings[1], readings[2], readings[3]);
    printf("Config: particles=%d sigma=%.1f max_err=%.1f random_inj=%.2f resample_thr=%.2f roughen=%.1f\n\n",
           cfg.num_particles, cfg.sigma_sensor, cfg.max_sensor_error,
           cfg.random_injection, cfg.resample_threshold, cfg.roughening_sigma);

    // Check scoring for a particle at truth
    printf("=== Scoring check for particle AT truth ===\n");
    Particle truth_p = { (float)kTrueX, (float)kTrueY, 0.0f };
    check_particle_scoring(truth_p, kTrueHeading, cfg, readings);
    printf("\n");

    for (int tick = 0; tick < 20; tick++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);

        const auto& ps = mcl.particles();
        float max_w = 0; int max_i = 0;
        int near_truth = 0;
        double closest_dist = 1e9;
        int closest_idx = 0;
        for (int i = 0; i < (int)ps.size(); i++) {
            if (ps[i].weight > max_w) { max_w = ps[i].weight; max_i = i; }
            double dx = ps[i].x - kTrueX;
            double dy = ps[i].y - kTrueY;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < 6.0) near_truth++;
            if (d < closest_dist) { closest_dist = d; closest_idx = i; }
        }
        double neff = mcl.n_eff();
        auto est = mcl.estimate();
        double err = std::sqrt((est.x-kTrueX)*(est.x-kTrueX) + (est.y-kTrueY)*(est.y-kTrueY));

        printf("Tick %2d | est=(%.1f,%.1f) err=%.1f | neff=%.1f | near_truth=%d closest=%.1f\n",
               tick, est.x, est.y, err, neff, near_truth, closest_dist);
        printf("  best-weight particle:\n");
        check_particle_scoring(ps[max_i], kTrueHeading, cfg, readings);
        printf("  closest-to-truth particle:\n");
        check_particle_scoring(ps[closest_idx], kTrueHeading, cfg, readings);

        mcl.resample();
    }
    return 0;
}
