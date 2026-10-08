#pragma once

#include "DeluxeLine.h"   // raw input ring + B-spline front end (CubicResult)
#include "SparseLine.h"   // shared sparse keyframe buffer (Keyframe/Window)
#include <cmath>
#include <cstdint>

namespace capicola {

// The keyframe writer. Owns the raw input ring and the write head (rawCount, in
// raw samples), consumes one sample per Analyze(), and sparsifies the stream
// into a borrowed SparseLine — the medium it shares with the Granule reader.
//
// Timestamps in SAMPLES (rawCount, the reader's native unit); carries fs so a
// seconds view is one divide away. Transport policy (stop-when-full, closing
// keyframe) is surfaced as plain queries/commands for the owner to drive.
template <int bufsz = 1024>
class Analyzer {
private:
    SparseLine<bufsz>*  sparse;      // borrowed; owner keeps the storage alive
    DeluxeLine<float,16> raw;        // raw input ring for the B-spline front end ([Deluge] 16 for the value read)
    CubicResult         mostRecent;  // last front-end read (value + d1)

    // 64-bit: both are monotonic for the life of the session and nothing
    // re-Init()s the analyzer, so 32 bits wrapped after 12 h 25 min of uptime.
    int64_t rawCount;                // write head, in raw samples
    int64_t sparseCount;             // keyframes written
    float  threshold;                // value-diff gate for storing a peak
    float  fs;                       // sample rate
    bool   firstAnalysis;            // seed the very first keyframe unconditionally
    bool   armFinal;                 // force one closing keyframe on next Analyze

    // [Deluge] Analysis runs this many samples further behind the input than
    // upstream's, so that each extremum's value can be read from the raw signal
    // with a symmetric 8-point window (see ReadValue()), which needs 4 samples
    // on the newer side.
    static constexpr int kLag = 3;

    // [Deluge] 8-point Lanczos (a = 4) weights at kPhases + 1 fractional
    // positions, interpolated between. Tabulated once, on first use.
    static constexpr int kPhases = 128;
    struct LanczosTable {
        float w[kPhases + 1][8];
        LanczosTable() {
            for (int p = 0; p <= kPhases; p++) {
                const float t = (float)p / (float)kPhases;
                float sum = 0.0f;
                for (int k = -3; k <= 4; k++) {
                    const float x = (float)k - t;
                    float v = 1.0f;
                    if (std::fabs(x) > 1e-6f) {
                        const float px = 3.14159265358979f * x;
                        v = 4.0f * std::sin(px) * std::sin(px * 0.25f) / (px * px);
                    }
                    w[p][k + 3] = v;
                    sum += v;
                }
                for (float& v : w[p]) v /= sum;   // unity gain at DC
            }
        }
    };
    static const LanczosTable& Lanczos() { static const LanczosTable table; return table; }

    // [Deluge] Value of the raw signal at a fractional delay (ReadHermite's
    // convention: the fraction moves towards older samples). The B-spline is
    // still what finds the extremum, but its value used to come from the
    // B-spline too, whose lowpass cost ~3 dB at 10 kHz and ~13 dB at 18 kHz.
    float ReadValue(float delay) const {
        const int   d = (int)delay;
        const float position = (delay - (float)d) * (float)kPhases;
        const int   p = (int)position;
        const float f = position - (float)p;
        const float* w0 = Lanczos().w[p];
        const float* w1 = Lanczos().w[(p < kPhases) ? p + 1 : p];
        float acc = 0.0f;
        for (int k = 0; k < 8; k++) {
            const float w = w0[k] + f * (w1[k] - w0[k]);
            acc += w * raw.Read((float)(d - 3 + k));
        }
        return acc;
    }

    inline void Push(const Keyframe& kf) {
        sparse->Write(kf);
        sparseCount++;
        rawCount++;
    }

public:
    // Implicit trivial constructor on purpose (SDRAM-resident; see Init()).

    // Bind the shared buffer and reset the write head. Buffer must outlive this.
    void Init(SparseLine<bufsz>& s, float sampleRate = 48000.0f) {
        sparse        = &s;
        fs            = sampleRate;
        raw.Init();
        rawCount      = 0;
        sparseCount   = 0;
        threshold     = 0.001f;
        firstAnalysis = true;
        armFinal      = false;
        mostRecent    = {};
    }

    void SetThreshold(float t) { threshold = t; }
    void ArmFinalFrame()       { armFinal = true; }   // force a closing keyframe

    int64_t WriteHead()     const { return rawCount; }
    int64_t KeyframeCount() const { return sparseCount; }
    bool   IsEmpty()       const { return sparseCount == 0; }
    bool   IsFull()        const { return sparseCount >= bufsz - 1; }
    float  SampleRate()    const { return fs; }
    double Seconds()       const { return (double)rawCount / fs; }
    Keyframe* GetLatest()        { return sparse->GetLatest(); }

    // Furthest raw-sample time a reader may safely reach: last written sample
    // minus one block. The only wire from writer to reader.
    double LiveEdge(int blockSize) const { return (double)(rawCount - 1 - blockSize); }

    // Local record-edge bandwidth: 1 / raw-sample interval between the two
    // latest keyframes (x fs for Hz). 0 when there is no interval yet.
    float LatestInvDuration() const {
        float d = sparse->GetWindowDuration(sparse->GetLatestIndex());
        return (d > 1e-5f) ? (1.0f / d) : 0.0f;
    }

    // Consume one sample; store a keyframe at each significant peak. Returns true
    // iff a keyframe was written. Advances the write head by 1.
    bool Analyze(float input) {
        raw.Write(input);

        // The lag-3 tap after this Write equals the lag-2 tap before it (the ring
        // advanced by one), so last sample's mostRecent is this sample's older
        // read — one spline eval per sample, not two.
        const CubicResult older = mostRecent;
        mostRecent = raw.ReadBSplineD1Integer(2 + kLag);

        bool crossedD1 = ((older.d1 > 0.0f && mostRecent.d1 <= 0.0f) ||
                          (older.d1 < 0.0f && mostRecent.d1 >= 0.0f));

        Keyframe* lastFrame = sparse->GetLatest();
        Keyframe kf;
        kf.value = mostRecent.value;
        kf.time  = (rawCount > 0) ? (double)(rawCount - 1) : 0.0;

        float valueDiff = std::abs(mostRecent.value - lastFrame->value);

        if (firstAnalysis) {
            Push(kf);
            firstAnalysis = false;
            return true;
        }
        if (armFinal) {
            Push(kf);
            armFinal = false;
            return true;
        }
        if (crossedD1 && valueDiff > threshold) {
            // Fractional peak location between the two front-end taps.
            float  alpha = std::abs(older.d1) / (std::abs(older.d1) + std::abs(mostRecent.d1));
            double index = (double)(rawCount - 2 - kLag) + alpha;
            if (std::abs(index - lastFrame->time) > 1.0) {
                kf.time  = index;
                kf.value = ReadValue((float)(3 + kLag) - alpha);   // [Deluge]
                Push(kf);
                return true;
            }
        }

        rawCount++;
        return false;
    }

    // Gap-gated boundary keyframe (call once per block; invariant math at the
    // KeyframeRecorder call site). Writes a keyframe at the current write
    // position only if the newest stored keyframe has fallen more than `maxGap`
    // raw samples behind — i.e. the signal has been extremum-quiet long enough
    // that the reader's coverage needs topping up. Replaces the old
    // unconditional per-block frame, which landed mid-slope and injected
    // block-rate reconstruction error. Does NOT advance the write head. Returns
    // true iff a frame was written.
    bool GuardKeyframe(double maxGap) {
        if ((double)(rawCount - kLag) - sparse->GetLatest()->time <= maxGap)   // [Deluge] kLag
            return false;
        Keyframe kf;
        // [Deluge] At the analysis point (kLag behind the input), with the raw
        // sample that's actually at that time rather than the smoothed value
        // from two samples earlier.
        kf.value = raw.Read((float)(1 + kLag));
        kf.time  = (rawCount > kLag) ? (double)(rawCount - 1 - kLag) : 0.0;
        sparse->Write(kf);
        sparseCount++;
        return true;
    }
};

} // namespace capicola
