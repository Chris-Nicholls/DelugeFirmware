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

#include "dsp/timestretch/keyframe_stretcher.h"
#include "definitions_cxx.hpp"
#include "memory/memory_allocator_interface.h"
#include "util/fixedpoint.h"
#include <algorithm>
#include <cmath>
#include <new>

namespace deluge::dsp::timestretch {

namespace {

// Input frames arrive at half of full-scale q31 (see feed()); the engine works in +/-1 floats.
constexpr float kInputScale = 1.0f / 1073741824.0f;
constexpr float kOutputScale = 2147483648.0f;

// Fixed crossfade used by the grains' boundary guards. Short, because it is also how far ahead of the furthest read
// head the keyframes must reach (see framesToFeed()) - and for live input, part of the latency.
constexpr float kGuardFade = 256.0f;
constexpr float kLiveGuardFade = 128.0f;

// Grain size: how far the reading head may drift from the stretch grid before splicing back onto it - in keyframes
// (capicola's measure, which adapts to the material), but never more than a time limit.
constexpr int32_t kLeashKeyframes = 128;
constexpr float kMaxGrainSamples = 2048.0f;

// Analyzer threshold (capicola's "quality" at maximum): the smallest step between successive extrema worth storing.
constexpr float kQuality = 0.001f;

// Insert a boundary keyframe when the signal has had no extremum for this long, so the ring always reaches close to
// the write head (silence and very low frequencies otherwise leave the readers with nothing to interpolate towards).
// Not too short: such a keyframe usually lands mid-slope, where the interpolation flattens the waveform out. At this
// length that only touches content below ~40 Hz. Live input can't read ahead, so this gap is directly part of the
// latency there - which buys a shorter one at the cost of touching content below ~85 Hz.
constexpr double kGuardGap = 512.0;
constexpr double kLiveGuardGap = 256.0;
constexpr int32_t kGuardCheckInterval = 32;

// When the signal suddenly departs from quiet (a hit out of silence), mark where it was still quiet with a keyframe.
// Otherwise the nearest keyframe before the hit could be a whole guard gap earlier, and the interpolation would fade
// the hit in across all of that - smearing its attack backwards.
constexpr float kDepartureFloor = 0.004f; // About -48 dBFS
constexpr float kDepartureRatio = 4.0f;   // ...and this much louder than anything since the last keyframe
constexpr double kDepartureMinGap = 8.0;

// Crossfade when punching a fresh grain in: for transients, and for jumps (loops, resyncs).
constexpr float kPunchFade = 128.0f;

// Onset detector: SVF cutoff (normalised; ~110 Hz at 44.1 kHz) and keep-ratio over the envelope average.
constexpr float kOnsetCutoff = 0.005f;
constexpr float kOnsetThreshold = 2.0f;

// Have a transient's grain fully faded in this many output samples before the transient.
constexpr float kOnsetPreRoll = 32.0f;

// How far past the furthest read head to keep the ring filled. The minimum is what the grains need not to run off the
// end; the extra is so that transients are detected before the grid reaches them. Only kMaxExtraFeedBlocks blocks'
// worth of the extra is fed per render, so starting a voice doesn't cost a burst of analysis.
constexpr int32_t kOnsetLookahead = 768;
constexpr int32_t kMaxExtraFeedBlocks = 2;

// The grid may be moved forward within the ring by up to this much without starting a new segment.
constexpr double kMaxForwardMove = 1024.0;

// A grid this far from where the last render left it gets a crossfade rather than a slide.
constexpr double kDiscontinuity = 256.0;

// Highest pitch ratio honoured; beyond this the lookahead wouldn't fit the ring.
constexpr float kMaxPitch = 16.0f;

inline int32_t toQ31(float y) {
	y = std::clamp(y, -1.0f, 0.99999994f);
	return static_cast<int32_t>(y * kOutputScale);
}

} // namespace

int32_t KeyframeStretcher::liveDelay(int32_t maxBlockSize) {
	// The grid must stay far enough behind the input that the reading head - which runs ahead of it when pitching up
	// - has room for a grain before it reaches the newest keyframe (up to a guard gap behind the input, and read
	// no closer than a block) and the boundary guard pulls it back. See Granule::Read().
	constexpr int32_t kRoomForGrain = 512;
	return static_cast<int32_t>(kLiveGuardGap) + maxBlockSize + static_cast<int32_t>(kLiveGuardFade) + kRoomForGrain;
}

KeyframeStretcher* KeyframeStretcher::create(int32_t numChannels, Mode mode) {
	void* memory = allocLowSpeed(sizeof(KeyframeStretcher));
	if (memory == nullptr) {
		return nullptr;
	}
	auto* stretcher = new (memory) KeyframeStretcher();
	stretcher->init(numChannels, mode);
	return stretcher;
}

void KeyframeStretcher::destroy(KeyframeStretcher* stretcher) {
	if (stretcher != nullptr) {
		stretcher->~KeyframeStretcher();
		delugeDealloc(stretcher);
	}
}

void KeyframeStretcher::init(int32_t numChannels, Mode mode) {
	numChannels_ = (numChannels == 2) ? 2 : 1;
	guardGap_ = (mode == Mode::LIVE) ? kLiveGuardGap : kGuardGap;
	guardFade_ = (mode == Mode::LIVE) ? kLiveGuardFade : kGuardFade;
	for (int32_t c = 0; c < numChannels_; c++) {
		Channel& ch = channels_[c];
		ch.sparse.Init();
		ch.analyzer.Init(ch.sparse, static_cast<float>(kSampleRate));
		ch.analyzer.SetThreshold(kQuality);
		ch.peakSinceKeyframe = 0.0f;
		for (Granule& grain : ch.grain) {
			grain.Init(ch.sparse, 0.0);
			grain.SetFade(guardFade_);
			grain.SetLeash(kLeashKeyframes);
			grain.SetMaxLead(kMaxGrainSamples);
		}
	}
	onsets_.Init(static_cast<float>(kSampleRate), kOnsetCutoff);
	onsets_.SetThreshold(kOnsetThreshold);

	fed_ = 0;
	segmentKf_ = 0.0;
	segmentSrcBig_ = 0;
	sourceEndKf_ = -1;
	started_ = false;
	punchPending_ = false;
	punchPos_ = 0.0;
	lastGridEnd_ = 0.0;
	liveGrain_ = 0;
	xGain_ = 1.0f;
	xStep_ = 0.0f;
	onsetHead_ = 0;
	onsetCount_ = 0;
}

void KeyframeStretcher::beginSegment(int32_t sourceSample) {
	segmentKf_ = static_cast<double>(fed_);
	segmentSrcBig_ = static_cast<int64_t>(sourceSample) << 24;
	sourceEndKf_ = -1;
	onsetCount_ = 0; // Anything detected but not yet reached belonged to the old segment
	if (started_) {
		requestPunch(segmentKf_);
	}
}

double KeyframeStretcher::gridFor(int64_t samplePosBig, int32_t playDirection) const {
	return segmentKf_ + static_cast<double>((samplePosBig - segmentSrcBig_) * playDirection) * (1.0 / 16777216.0);
}

bool KeyframeStretcher::canMoveGridTo(double grid) const {
	return grid >= segmentKf_ && grid >= oldestTime() + 2.0 && grid <= static_cast<double>(fed_) + kMaxForwardMove;
}

bool KeyframeStretcher::isDiscontinuity(double grid) const {
	return started_ && std::abs(grid - lastGridEnd_) > kDiscontinuity;
}

void KeyframeStretcher::requestPunch(double grid) {
	punchPending_ = true;
	punchPos_ = grid;
}

double KeyframeStretcher::front() const {
	double f = 0.0;
	for (int32_t c = 0; c < numChannels_; c++) {
		for (const Granule& grain : channels_[c].grain) {
			f = std::max(f, grain.Front());
		}
	}
	return f;
}

double KeyframeStretcher::oldestTime() const {
	double t = 0.0;
	for (int32_t c = 0; c < numChannels_; c++) {
		Sparse& sparse = const_cast<Sparse&>(channels_[c].sparse);
		t = std::max(t, sparse.GetSample(sparse.GetOldestIndex())->time);
	}
	return t;
}

int32_t KeyframeStretcher::framesToFeed(int32_t numSamples, double gridStart, float gridSpeed, float pitch) const {
	pitch = std::min(pitch, kMaxPitch);
	double furthest = gridStart + static_cast<double>(gridSpeed) * numSamples;
	if (punchPending_) {
		furthest = std::max(furthest, punchPos_);
	}
	if (started_) {
		furthest = std::max(furthest, front());
	}
	furthest += static_cast<double>(pitch) * numSamples;

	// The grains may read up to one block behind the newest keyframe, and the newest keyframe may sit up to
	// kGuardGap behind the write head.
	int64_t minTarget = static_cast<int64_t>(std::ceil(furthest)) + 2 * numSamples + static_cast<int64_t>(guardGap_)
	                    + static_cast<int64_t>(guardFade_) + 32;
	int64_t mustFeed = std::max<int64_t>(minTarget - fed_, 0);
	int64_t wouldLikeToFeed = std::max<int64_t>(minTarget + kOnsetLookahead - fed_, 0);
	return static_cast<int32_t>(std::min(wouldLikeToFeed, mustFeed + kMaxExtraFeedBlocks * numSamples));
}

void KeyframeStretcher::pushOnset(double onset) {
	if (onset < segmentKf_) {
		return; // The rise started before this segment did - it's not a transient in this material
	}
	if (onsetCount_ == kMaxPendingOnsets) {
		onsetHead_ = (onsetHead_ + 1) % kMaxPendingOnsets;
		onsetCount_--;
	}
	pendingOnsets_[(onsetHead_ + onsetCount_) % kMaxPendingOnsets] = onset;
	onsetCount_++;
}

void KeyframeStretcher::feed(const int32_t* frames, int32_t numFrames) {
	if (numChannels_ == 2) {
		for (int32_t i = 0; i < numFrames; i++) {
			const float l = static_cast<float>(frames[0]) * kInputScale;
			const float r = static_cast<float>(frames[1]) * kInputScale;
			frames += 2;
			analyze(channels_[0], l);
			analyze(channels_[1], r);
			double onset;
			if (onsets_.Analyze(0.5f * (l + r), &onset)) {
				pushOnset(onset);
			}
			if ((i & (kGuardCheckInterval - 1)) == 0) {
				guardKeyframes();
			}
		}
	}
	else {
		for (int32_t i = 0; i < numFrames; i++) {
			const float x = static_cast<float>(frames[i]) * kInputScale;
			analyze(channels_[0], x);
			double onset;
			if (onsets_.Analyze(x, &onset)) {
				pushOnset(onset);
			}
			if ((i & (kGuardCheckInterval - 1)) == 0) {
				guardKeyframes();
			}
		}
	}
	fed_ += numFrames;
	guardKeyframes();
}

void KeyframeStretcher::guardKeyframes() {
	for (int32_t c = 0; c < numChannels_; c++) {
		if (channels_[c].analyzer.GuardKeyframe(guardGap_)) {
			channels_[c].peakSinceKeyframe = 0.0f;
		}
	}
}

inline void KeyframeStretcher::analyze(Channel& ch, float x) {
	const float magnitude = std::abs(x);
	if (magnitude > kDepartureFloor && magnitude > kDepartureRatio * ch.peakSinceKeyframe) {
		ch.analyzer.GuardKeyframe(kDepartureMinGap);
		ch.peakSinceKeyframe = 0.0f;
	}
	if (ch.analyzer.Analyze(x)) {
		ch.peakSinceKeyframe = magnitude;
	}
	else {
		ch.peakSinceKeyframe = std::max(ch.peakSinceKeyframe, magnitude);
	}
}

void KeyframeStretcher::feedSilence(int32_t numFrames) {
	for (int32_t i = 0; i < numFrames; i++) {
		for (int32_t c = 0; c < numChannels_; c++) {
			analyze(channels_[c], 0.0f);
		}
		if ((i & (kGuardCheckInterval - 1)) == 0) {
			guardKeyframes();
		}
	}
	fed_ += numFrames;
	guardKeyframes();
}

void KeyframeStretcher::markSourceEnded() {
	if (sourceEndKf_ < 0) {
		sourceEndKf_ = fed_;
	}
}

bool KeyframeStretcher::drained() const {
	if (sourceEndKf_ < 0 || xGain_ < 1.0f) {
		return false;
	}
	const double end = static_cast<double>(sourceEndKf_);
	for (int32_t c = 0; c < numChannels_; c++) {
		const Granule& grain = channels_[c].grain[liveGrain_];
		if (grain.GetActual().uniform < end || grain.GetIdeal().uniform < end) {
			return false;
		}
	}
	return true;
}

void KeyframeStretcher::punch(double gridPos, double headPos, float fadeSamples) {
	liveGrain_ ^= 1;
	for (int32_t c = 0; c < numChannels_; c++) {
		channels_[c].grain[liveGrain_].RetriggerAt(gridPos, headPos);
	}
	xGain_ = 0.0f;
	xStep_ = 1.0f / fadeSamples;
}

void KeyframeStretcher::popOnset() {
	onsetHead_ = (onsetHead_ + 1) % kMaxPendingOnsets;
	onsetCount_--;
}

// The next pending onset, as seen from output sample i of this block. Either acts on it now and returns false, or
// returns true having shortened *end to the next sample worth looking at it again.
//
// The aim is for the reading head to reach the onset at exactly the moment the grid does, coming from a grain that
// has fully faded in by then. So every grain seated for it is seated where, moving at the pitch rate, it'll arrive at
// the onset in the time the grid takes to.
bool KeyframeStretcher::handleOnset(int32_t i, double gridStart, float gridSpeed, float pitch, int32_t* end) {
	const double onset = pendingOnsets_[onsetHead_];
	const double gridNow = gridStart + static_cast<double>(gridSpeed) * i;
	const Granule& live = channels_[0].grain[liveGrain_];
	const double head = live.GetActual().uniform;
	const double leadIn = kPunchFade + kOnsetPreRoll; // Output samples a grain needs before the onset

	if (onset <= gridNow || onset < head - pitch) {
		popOnset(); // Too late
		return false;
	}
	if (gridSpeed <= 1e-6f) {
		return true; // Frozen - the grid will never get there
	}

	auto arrivalSeat = [&]() {
		double seat = onset - static_cast<double>(pitch) * (onset - gridNow) / static_cast<double>(gridSpeed);
		// Not further behind the grid than the grain could lag it without splicing back onto it, nor off the ring
		Sparse& sparse = const_cast<Sparse&>(channels_[0].sparse);
		const double maxLag = std::max(sparse.GetSample(live.GetIdeal().sparse - (kLeashKeyframes * 3) / 4)->time,
		                               gridNow - 0.75 * kMaxGrainSamples);
		return std::max({seat, maxLag, oldestTime() + 2.0, segmentKf_});
	};

	// The grid is a lead-in away: punch in a fresh grain - unless the head's already on course
	const double gridTrigger = onset - static_cast<double>(gridSpeed) * leadIn;
	if (gridNow >= gridTrigger) {
		const double seat = arrivalSeat();
		if (xGain_ >= 1.0f && std::abs(head - seat) > 2.0 * pitch) {
			punch(gridNow, seat, kPunchFade);
		}
		popOnset();
		return false;
	}
	*end = std::min(*end, std::max(static_cast<int32_t>(std::ceil((gridTrigger - gridStart) / gridSpeed)), i + 1));

	// A head running ahead of the grid (pitch above the stretch rate) would otherwise play the transient early - by up
	// to a whole grain, and grains are long where keyframes are sparse, like in silence - and then the punch would
	// play it again. So before it gets close enough for the outgoing grain not to have faded out by then, re-seat it.
	if (pitch <= gridSpeed) {
		return true;
	}
	const double barrier = onset - static_cast<double>(pitch) * leadIn;
	if (head >= barrier + pitch) {
		return true; // Already past it - too late to help
	}
	if (xGain_ < 1.0f) {
		// Mid-takeover: look again once it's done
		const int32_t fadeLeft = static_cast<int32_t>(std::ceil((1.0f - xGain_) / xStep_));
		*end = std::min(*end, i + std::max<int32_t>(fadeLeft, 1));
		return true;
	}
	// Render up to within a sample of the barrier, then re-seat
	const double samplesUntilHead = (barrier - head) / static_cast<double>(pitch);
	if (samplesUntilHead > 1.0) {
		*end = std::min(*end, i + static_cast<int32_t>(std::ceil(samplesUntilHead)) - 1);
		return true;
	}
	punch(gridNow, arrivalSeat(), kPunchFade);
	return false;
}

void KeyframeStretcher::renderSpan(int32_t* out, int32_t from, int32_t to, int32_t outChannels, int32_t* amplitude,
                                   int32_t amplitudeIncrement) {
	int32_t* outPos = out + from * outChannels;
	const int32_t live = liveGrain_;
	const int32_t idle = live ^ 1;

	for (int32_t i = from; i < to; i++) {
		float y[2];
		if (xGain_ >= 1.0f) {
			for (int32_t c = 0; c < numChannels_; c++) {
				y[c] = channels_[c].grain[live].Read();
			}
		}
		else {
			const float w = capicola::SmoothStep(xGain_);
			for (int32_t c = 0; c < numChannels_; c++) {
				const float in = channels_[c].grain[live].Read();
				const float outgoing = channels_[c].grain[idle].Read();
				y[c] = in * w + outgoing * (1.0f - w);
			}
			xGain_ = std::min(xGain_ + xStep_, 1.0f);
		}

		*amplitude += amplitudeIncrement;

		if (outChannels == 2) {
			const float r = (numChannels_ == 2) ? y[1] : y[0];
			outPos[0] = multiply_accumulate_32x32_rshift32_rounded(outPos[0], toQ31(y[0]), *amplitude);
			outPos[1] = multiply_accumulate_32x32_rshift32_rounded(outPos[1], toQ31(r), *amplitude);
			outPos += 2;
		}
		else {
			const float m = (numChannels_ == 2) ? 0.5f * (y[0] + y[1]) : y[0];
			*outPos = multiply_accumulate_32x32_rshift32_rounded(*outPos, toQ31(m), *amplitude);
			outPos++;
		}
	}
}

void KeyframeStretcher::render(int32_t* out, int32_t numSamples, int32_t outChannels, double gridStart, float gridSpeed,
                               float pitch, int32_t amplitude, int32_t amplitudeIncrement) {
	pitch = std::min(pitch, kMaxPitch);

	for (int32_t c = 0; c < numChannels_; c++) {
		for (Granule& grain : channels_[c].grain) {
			grain.BeginBlock(numSamples);
			grain.SetPitch(pitch);
			grain.SetStretch(gridSpeed);
		}
	}

	if (!started_) {
		// First render: start right where we are, no fade - keeps the attack of a freshly triggered note.
		for (int32_t c = 0; c < numChannels_; c++) {
			channels_[c].grain[0].RetriggerAt(gridStart);
			channels_[c].grain[1].RetriggerAt(gridStart);
		}
		liveGrain_ = 0;
		xGain_ = 1.0f;
		started_ = true;
		punchPending_ = false;
	}
	else if (punchPending_) {
		punch(punchPos_, punchPos_, kPunchFade);
		punchPending_ = false;
	}

	for (int32_t c = 0; c < numChannels_; c++) {
		channels_[c].grain[liveGrain_].SetGrid(gridStart);
	}

	int32_t i = 0;
	while (i < numSamples) {
		int32_t end = numSamples;
		if (onsetCount_ != 0 && !handleOnset(i, gridStart, gridSpeed, pitch, &end)) {
			continue; // Acted on it - look again from the same sample
		}
		renderSpan(out, i, end, outChannels, &amplitude, amplitudeIncrement);
		i = end;
	}

	lastGridEnd_ = gridStart + static_cast<double>(gridSpeed) * numSamples;
}

} // namespace deluge::dsp::timestretch
