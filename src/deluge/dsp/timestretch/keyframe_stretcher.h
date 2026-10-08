/*
 * Copyright © 2026 Synthstrom Audible Limited
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 *
 * The Synthstrom Audible Deluge Firmware is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "dsp/timestretch/capicola/Analyzer.h"
#include "dsp/timestretch/capicola/Granule.h"
#include "dsp/timestretch/capicola/OnsetDetector.h"
#include "dsp/timestretch/capicola/SparseLine.h"
#include <cstdint>

namespace deluge::dsp::timestretch {

/// Keyframe time stretching (capicola, see capicola/README.md) driven by Sample playback.
///
/// The source is fed in at its native rate, in play direction, and sparsified into a ring of keyframes (one per
/// waveform extremum). Playback reads that ring with two cursors per grain: a "grid" that advances at the
/// time-stretch rate and a "head" that advances at the pitch rate, crossfading the head back onto the grid whenever it
/// drifts a grain away. Pitch and time are therefore fully independent.
///
/// Time inside the engine is "keyframe time": a monotonic count of frames fed. A *segment* is a run of contiguous
/// source starting at a known sample. When playback jumps (loop, resync, restart) the caller begins a new segment and
/// the engine crossfades to a fresh grain there; keyframe time itself never jumps.
///
/// On top of that, an onset detector watches the fed audio, and when the grid reaches a detected transient a fresh
/// grain is punched in just ahead of it, so attacks are played intact rather than smeared across a splice.
class KeyframeStretcher {
public:
	static constexpr int32_t kRingKeyframes = 4096;

	/// Allocates (in external RAM) and initialises. Returns nullptr if out of memory.
	static KeyframeStretcher* create(int32_t numChannels);
	static void destroy(KeyframeStretcher* stretcher);

	[[nodiscard]] int32_t numChannels() const { return numChannels_; }

	/// The next frame fed will be source sample `sourceSample`. Crossfades to it at the next render (or starts there
	/// outright if nothing has played yet).
	void beginSegment(int32_t sourceSample);

	/// Keyframe time of a source position (24-bit fractional samples, as TimeStretcher::samplePosBig) in the current
	/// segment.
	[[nodiscard]] double gridFor(int64_t samplePosBig, int32_t playDirection) const;

	/// Whether the grid may simply move to `grid` (still within the current segment and the ring), as opposed to
	/// needing a new segment.
	[[nodiscard]] bool canMoveGridTo(double grid) const;

	/// Whether a grid at `grid` is far enough from where the last render left it to deserve a crossfade.
	[[nodiscard]] bool isDiscontinuity(double grid) const;
	void requestPunch(double grid);

	/// How many source frames to feed before render()ing `numSamples` at these rates.
	[[nodiscard]] int32_t framesToFeed(int32_t numSamples, double gridStart, float gridSpeed, float pitch) const;

	/// Interleaved frames (numChannels() wide), as accumulated by SampleLowLevelReader::readSamplesNative() into a
	/// zeroed buffer at amplitude 0x7FFFFFFF — i.e. half of full-scale q31.
	void feed(const int32_t* frames, int32_t numFrames);
	void feedSilence(int32_t numFrames);

	/// The source has run out (end of waveform): everything fed from here on is silence.
	void markSourceEnded();
	[[nodiscard]] bool sourceEnded() const { return sourceEndKf_ >= 0; }

	/// Source has ended and every grain has played past the end of it.
	[[nodiscard]] bool drained() const;

	/// Accumulates `numSamples` frames into `out` (outChannels wide), scaled by a ramping q31 amplitude the same way
	/// SampleLowLevelReader::readSamplesNative() applies it.
	void render(int32_t* out, int32_t numSamples, int32_t outChannels, double gridStart, float gridSpeed, float pitch,
	            int32_t amplitude, int32_t amplitudeIncrement);

private:
	using Sparse = capicola::SparseLine<kRingKeyframes>;
	using Analyzer = capicola::Analyzer<kRingKeyframes>;
	using Granule = capicola::Granule<kRingKeyframes>;

	struct Channel {
		Sparse sparse;
		Analyzer analyzer;
		Granule grain[2];
		float peakSinceKeyframe; ///< Loudest input since the last keyframe was written
	};

	void init(int32_t numChannels);
	void punch(double gridPos, double headPos, float fadeSamples);
	void popOnset();
	void guardKeyframes();
	void analyze(Channel& ch, float x);
	bool handleOnset(int32_t i, double gridStart, float gridSpeed, float pitch, int32_t* end);
	void pushOnset(double onset);
	void renderSpan(int32_t* out, int32_t from, int32_t to, int32_t outChannels, int32_t* amplitude,
	                int32_t amplitudeIncrement);
	[[nodiscard]] double front() const;
	[[nodiscard]] double oldestTime() const;

	Channel channels_[2];
	capicola::OnsetDetector onsets_;

	int32_t numChannels_;
	int64_t fed_;           ///< Frames fed so far == analyzer write head == keyframe time of the next frame
	double segmentKf_;      ///< Keyframe time at which the current segment starts
	int64_t segmentSrcBig_; ///< Source position (<<24) the current segment starts at
	int64_t sourceEndKf_;   ///< Keyframe time the source ran out at, or -1

	bool started_; ///< Grains have been seated
	bool punchPending_;
	double punchPos_;
	double lastGridEnd_;

	int32_t liveGrain_; ///< Index of the grain fading in / steady; the other fades out
	float xGain_;       ///< Gain of the live grain during a takeover
	float xStep_;

	static constexpr int32_t kMaxPendingOnsets = 8;
	double pendingOnsets_[kMaxPendingOnsets];
	int32_t onsetHead_;
	int32_t onsetCount_;
};

} // namespace deluge::dsp::timestretch
