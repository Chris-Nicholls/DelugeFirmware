#pragma once

// Derived from capicola's lib/Detector.h for the Deluge port. Same front end
// (TKEO -> SVF lowpass -> B-spline d1 peak picker, adaptive ratio threshold
// over a slow envelope average, hold-off between kept events), but:
//   - no event history ring (the Deluge only needs "a kept peak just happened"),
//   - time is an integer sample counter, not seconds,
//   - it reports where the envelope started rising towards each kept peak,
//     which is where the transient's energy arrived: a far better onset
//     estimate than the peak, which the smoothing delays well past the attack.

#include "DeluxeLine.h"
#include "filter.h"
#include <cmath>
#include <cstdint>

namespace capicola {

class OnsetDetector {
public:
	// Trivially constructible on purpose; call Init().
	void Init(float sampleRate, float cutoff) {
		fs = sampleRate;
		raw.Init();
		env.Init();
		svf.Init();
		svf.SetControls(cutoff, 0.0f);
		avgLp.Init();
		// kAvgSec time constant -> normalised fc: 1/(2*pi*tau) Hz over fs/2.
		avgLp.SetCutoff(1.0f / ((float)M_PI * kAvgSec * fs));
		threshold = 2.0f;
		clock = 0.0;
		gapSamps = 0;
		minGapSamps = (int32_t)(kMinGapSec * fs);
		prevNewer = {};
		troughTime = 0.0;
	}

	void SetThreshold(float t) { threshold = t; }

	// Consume one sample. Returns true iff a KEPT peak was found; *onset then
	// holds the estimated onset time (in samples, same clock as the input).
	inline bool Analyze(float x, double* onset) {
		raw.Write(x);
		svf.Tick(TkeoAtLag2(raw));
		const float e = svf.GetLowpass();
		env.Write(e);
		avgLp.Tick(e);

		// The lag-3 tap after this Write equals the lag-2 tap before it, so last
		// sample's `newer` is this sample's `older` — one spline eval per sample.
		// (Detector.h uses ReadBSplineFull(2.0f) for both reads; at an integer
		// delay that's the same value and d1 as this, for a fraction of the work.)
		const CubicResult newer = env.ReadBSplineD1Integer(2);
		const CubicResult older = prevNewer;
		prevNewer = newer;

		// Envelope samples sit 2 behind the input (raw B-spline lag) and a
		// further 2..3 behind on the env ring.
		const double now = clock;
		bool fired = false;

		// d1 is d/d(delay); the delay axis points backward in time, so rising in
		// time is d1 < 0. Peak: rising at the older tap, not at the newer.
		if (older.d1 < 0.0f && newer.d1 >= 0.0f) {
			const float dd = older.d1 - newer.d1; // < 0 by the test above
			const float frac = (dd < -1e-20f) ? (older.d1 / dd) : 0.5f;
			const float value = env.ReadBSplineFull(3.0f - frac, 1).value;
			if (value > kAbsFloor && value > threshold * avgLp.GetLowpass() && gapSamps <= 0) {
				gapSamps = minGapSamps;
				*onset = troughTime;
				fired = true;
			}
		}
		// The rise towards the next peak starts at the last point the envelope
		// was NOT rising. (Tracked as "not rising" rather than as a d1 sign
		// change, because in digital silence d1 is exactly zero throughout.)
		if (newer.d1 >= 0.0f) {
			troughTime = now - 2.0 - 2.5;
		}
		if (gapSamps > 0) {
			gapSamps--;
		}
		clock += 1.0; // A double counter: converting an integer one is a library call on ARM
		return fired;
	}

	// |TKEO| of the B-spline through the raw input, at integer lag 2: what
	// ReadBSplineFull(2.0f, 1).tkeo gives, from its three non-zero taps.
	static inline float TkeoAtLag2(const DeluxeLine<float, 8>& line) {
		const float x0 = line.Read(1.0f);
		const float x1 = line.Read(2.0f);
		const float x2 = line.Read(3.0f);
		const float value = (x0 + 4.0f * x1 + x2) * (1.0f / 6.0f);
		const float d1 = (x2 - x0) * 0.5f;
		const float d2 = x0 - 2.0f * x1 + x2;
		return std::abs(d1 * d1 - value * d2);
	}

	static constexpr float kAvgSec = 0.1f;
	static constexpr float kAbsFloor = 1e-5f;
	static constexpr float kMinGapSec = 0.1f;

private:
	DeluxeLine<float, 8> raw;
	DeluxeLine<float, 8> env;
	CubicResult prevNewer;
	StateVariable svf;
	OnePole avgLp;
	float fs;
	float threshold;
	double troughTime;
	double clock;
	int32_t gapSamps;
	int32_t minGapSamps;
};

} // namespace capicola
