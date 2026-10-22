#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

struct GrindRecommendation {
    float actualResistance{0.0f}; // bar*s^2/ml^2 (or bar / (ml/s)^2)
    float targetResistance{1.8f};
    float deltaGrindSteps{0.0f}; // negative = finer, positive = coarser
    float timeToCompress{0.0f};  // seconds to peak resistance
    bool isValid{false};
    std::string adviceText{};
};

class GrindAdvisor {
  public:
    static constexpr float kDefaultKGrind = 3.0f;           // Steps per log resistance unit
    static constexpr float kDefaultTargetResistance = 1.8f; // Nominal 9 bar @ 2.2 ml/s (1.85 bar*s^2/ml^2)

    GrindAdvisor() = default;

    void reset() {
        sumRpuck_ = 0.0f;
        sampleCount_ = 0;
        peakResistance_ = 0.0f;
        timeToPeak_ = 0.0f;
        shotStartTimeMs_ = 0;
        lastRecommendation_ = GrindRecommendation{};
    }

    void onShotStart(uint32_t nowMs) {
        reset();
        shotStartTimeMs_ = nowMs;
    }

    void addSample(float pressure, float flow, float rpuck, uint32_t nowMs) {
        if (rpuck > peakResistance_) {
            peakResistance_ = rpuck;
            if (shotStartTimeMs_ > 0 && nowMs >= shotStartTimeMs_) {
                timeToPeak_ = static_cast<float>(nowMs - shotStartTimeMs_) / 1000.0f;
            }
        }
        // Steady extraction window: pressure >= 4.0 bar, flow >= 0.8 ml/s, valid positive resistance
        if (pressure >= 4.0f && flow >= 0.8f && rpuck > 0.05f && std::isfinite(rpuck)) {
            sumRpuck_ += rpuck;
            sampleCount_++;
        }
    }

    GrindRecommendation onShotEnd(float targetR = kDefaultTargetResistance, float kGrind = kDefaultKGrind) {
        lastRecommendation_ = evaluate(targetR, kGrind);
        return lastRecommendation_;
    }

    [[nodiscard]] GrindRecommendation evaluate(float targetR = kDefaultTargetResistance, float kGrind = kDefaultKGrind) const {
        GrindRecommendation rec{};
        rec.targetResistance = targetR;
        rec.timeToCompress = timeToPeak_;

        if (sampleCount_ < 10) { // Require minimum 10 steady-state samples (~1-2 seconds extraction)
            rec.isValid = false;
            rec.actualResistance = (sampleCount_ > 0) ? (sumRpuck_ / static_cast<float>(sampleCount_)) : 0.0f;
            rec.deltaGrindSteps = 0.0f;
            rec.adviceText = "Insufficient extraction data for grind advice";
            return rec;
        }

        rec.actualResistance = sumRpuck_ / static_cast<float>(sampleCount_);
        rec.deltaGrindSteps = kGrind * std::log(rec.actualResistance / rec.targetResistance);
        rec.isValid = true;

        char buf[96];
        if (std::abs(rec.deltaGrindSteps) < 0.25f) {
            rec.adviceText = "Grind dialed in! No adjustment needed.";
        } else if (rec.deltaGrindSteps < 0.0f) {
            snprintf(buf, sizeof(buf), "Shot ran fast: Adjust ~%.1f steps FINER", std::abs(rec.deltaGrindSteps));
            rec.adviceText = buf;
        } else {
            snprintf(buf, sizeof(buf), "Shot ran slow: Adjust ~%.1f steps COARSER", rec.deltaGrindSteps);
            rec.adviceText = buf;
        }
        return rec;
    }

    [[nodiscard]] const GrindRecommendation &getLastRecommendation() const { return lastRecommendation_; }

    [[nodiscard]] uint32_t getSampleCount() const { return sampleCount_; }

    [[nodiscard]] float getAverageResistance() const {
        return sampleCount_ > 0 ? (sumRpuck_ / static_cast<float>(sampleCount_)) : 0.0f;
    }

    [[nodiscard]] float getPeakResistance() const { return peakResistance_; }
    [[nodiscard]] float getTimeToPeak() const { return timeToPeak_; }

  private:
    float sumRpuck_{0.0f};
    uint32_t sampleCount_{0};
    float peakResistance_{0.0f};
    float timeToPeak_{0.0f};
    uint32_t shotStartTimeMs_{0};
    GrindRecommendation lastRecommendation_{};
};
