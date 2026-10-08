#include "CppUTest/TestHarness.h"
#include "dsp/timestretch/keyframe_stretcher.h"

#include <cmath>
#include <cstdint>
#include <vector>

using deluge::dsp::timestretch::KeyframeStretcher;

namespace {

constexpr float kSampleRateF = 44100.0f;
constexpr int32_t kBlock = 128;

// Plays a mono source through the engine the way VoiceSample::renderKeyframeStretched() does: samplePosBig is the
// authority, the "reader" feeds at native speed from wherever the current segment started.
struct Player {
	explicit Player(std::vector<float> source) : src(std::move(source)) { engine = KeyframeStretcher::create(1); }
	~Player() { KeyframeStretcher::destroy(engine); }

	void renderBlock(std::vector<float>& out, float pitch, float stretch) {
		uint64_t combinedIncrement = (uint64_t)(stretch * 16777216.0f);
		float gridSpeed = (float)combinedIncrement / 16777216.0f;

		double grid = engine->gridFor(samplePosBig, 1);
		if (needsSegment || !engine->canMoveGridTo(grid)) {
			needsSegment = false;
			int32_t samplePos = (int32_t)(samplePosBig >> 24);
			readPos = samplePos;
			engine->beginSegment(samplePos);
			if (samplePos < 0 || samplePos >= (int32_t)src.size()) {
				engine->markSourceEnded();
			}
			grid = engine->gridFor(samplePosBig, 1);
		}
		else if (engine->isDiscontinuity(grid)) {
			engine->requestPunch(grid);
		}

		int32_t toFeed = engine->framesToFeed(kBlock, grid, gridSpeed, pitch);
		std::vector<int32_t> frames;
		for (int32_t i = 0; i < toFeed; i++) {
			if (engine->sourceEnded()) {
				engine->feedSilence(toFeed - i);
				break;
			}
			if (readPos >= (int64_t)src.size()) {
				if (!frames.empty()) {
					engine->feed(frames.data(), (int32_t)frames.size());
					frames.clear();
				}
				engine->markSourceEnded();
				i--;
				continue;
			}
			frames.push_back((int32_t)std::lround(src[readPos++] * 1073741824.0f)); // half-scale q31
		}
		if (!frames.empty()) {
			engine->feed(frames.data(), (int32_t)frames.size());
		}

		std::vector<int32_t> block(kBlock, 0);
		engine->render(block.data(), kBlock, 1, grid, gridSpeed, pitch, 2147483647, 0);
		for (int32_t v : block) {
			out.push_back((float)v / 1073741824.0f); // amplitude 0x7FFFFFFF halves full-scale q31
		}
		samplePosBig += (int64_t)combinedIncrement * kBlock;
	}

	std::vector<float> render(int32_t numSamples, float pitch, float stretch) {
		std::vector<float> out;
		while ((int32_t)out.size() < numSamples) {
			renderBlock(out, pitch, stretch);
		}
		out.resize(numSamples);
		return out;
	}

	std::vector<float> src;
	KeyframeStretcher* engine;
	int64_t samplePosBig = 0;
	int64_t readPos = 0;
	bool needsSegment = true;
};

std::vector<float> sine(float freq, float amplitude, float seconds) {
	std::vector<float> s((size_t)(seconds * kSampleRateF));
	for (size_t i = 0; i < s.size(); i++) {
		s[i] = amplitude * std::sin(2.0f * (float)M_PI * freq * (float)i / kSampleRateF);
	}
	return s;
}

// Rising zero crossings per second over [from, to).
float crossingRate(const std::vector<float>& x, size_t from, size_t to) {
	int32_t n = 0;
	for (size_t i = from + 1; i < to; i++) {
		if (x[i - 1] < 0.0f && x[i] >= 0.0f) {
			n++;
		}
	}
	return (float)n * kSampleRateF / (float)(to - from);
}

float rms(const std::vector<float>& x, size_t from, size_t to) {
	double acc = 0.0;
	for (size_t i = from; i < to; i++) {
		acc += (double)x[i] * x[i];
	}
	return (float)std::sqrt(acc / (double)(to - from));
}

bool allFinite(const std::vector<float>& x) {
	for (float v : x) {
		if (!std::isfinite(v)) {
			return false;
		}
	}
	return true;
}

// Decaying 1 kHz bursts out of silence, a bit like a drum loop.
std::vector<float> bursts(const std::vector<float>& atSeconds, float seconds) {
	std::vector<float> s((size_t)(seconds * kSampleRateF), 0.0f);
	for (float t : atSeconds) {
		size_t start = (size_t)(t * kSampleRateF);
		for (size_t i = 0; i < (size_t)(0.05f * kSampleRateF) && start + i < s.size(); i++) {
			float time = (float)i / kSampleRateF;
			s[start + i] = 0.8f * std::exp(-time * 80.0f) * std::sin(2.0f * (float)M_PI * 1000.0f * time);
		}
	}
	return s;
}

int32_t firstAbove(const std::vector<float>& x, float threshold, size_t from) {
	for (size_t i = from; i < x.size(); i++) {
		if (std::abs(x[i]) > threshold) {
			return (int32_t)i;
		}
	}
	return -1;
}

} // namespace

TEST_GROUP(KeyframeStretcherTest){};

TEST(KeyframeStretcherTest, unityReproducesInput) {
	std::vector<float> src = sine(440.0f, 0.5f, 1.0f);
	Player player(src);
	std::vector<float> out = player.render(30000, 1.0f, 1.0f);

	CHECK(allFinite(out));
	// Reconstruction from extrema is close, but not exact, and the analysis may delay by a sample or so
	double best = 0.0;
	for (int32_t lag = -4; lag <= 4; lag++) {
		double num = 0.0, da = 0.0, db = 0.0;
		for (size_t i = 2000; i < 28000; i++) {
			num += (double)out[i] * src[i + lag];
			da += (double)out[i] * out[i];
			db += (double)src[i + lag] * src[i + lag];
		}
		best = std::max(best, num / std::sqrt(da * db));
	}
	CHECK(best > 0.995);
	DOUBLES_EQUAL(rms(src, 2000, 28000), rms(out, 2000, 28000), 0.02);
}

TEST(KeyframeStretcherTest, unityReproducesBass) {
	// Extrema 400+ samples apart, so this leans on the analyzer's boundary keyframes not distorting it
	std::vector<float> src = sine(50.0f, 0.5f, 1.0f);
	Player player(src);
	std::vector<float> out = player.render(30000, 1.0f, 1.0f);

	double best = 0.0;
	for (int32_t lag = -4; lag <= 4; lag++) {
		double num = 0.0, da = 0.0, db = 0.0;
		for (size_t i = 4000; i < 28000; i++) {
			num += (double)out[i] * src[i + lag];
			da += (double)out[i] * out[i];
			db += (double)src[i + lag] * src[i + lag];
		}
		best = std::max(best, num / std::sqrt(da * db));
	}
	CHECK(best > 0.995);
}

TEST(KeyframeStretcherTest, slowingDownKeepsPitch) {
	Player player(sine(220.0f, 0.5f, 1.0f));
	std::vector<float> out = player.render(80000, 1.0f, 0.5f);

	CHECK(allFinite(out));
	DOUBLES_EQUAL(220.0, crossingRate(out, 4000, 80000), 220.0 * 0.03);
	DOUBLES_EQUAL(0.5 / std::sqrt(2.0), rms(out, 4000, 80000), 0.05);
	// Twice the output for the same source
	DOUBLES_EQUAL(40000.0, (double)(player.samplePosBig >> 24), 200.0);
}

TEST(KeyframeStretcherTest, speedingUpKeepsPitch) {
	Player player(sine(220.0f, 0.5f, 2.0f));
	std::vector<float> out = player.render(40000, 1.0f, 2.0f);

	CHECK(allFinite(out));
	DOUBLES_EQUAL(220.0, crossingRate(out, 2000, 40000), 220.0 * 0.03);
}

TEST(KeyframeStretcherTest, pitchingUpKeepsTime) {
	Player player(sine(220.0f, 0.5f, 1.5f));
	std::vector<float> out = player.render(40000, 2.0f, 1.0f);

	CHECK(allFinite(out));
	DOUBLES_EQUAL(440.0, crossingRate(out, 2000, 40000), 440.0 * 0.05);
	DOUBLES_EQUAL(40000.0, (double)(player.samplePosBig >> 24), 200.0);
}

TEST(KeyframeStretcherTest, pitchingDownKeepsTime) {
	Player player(sine(440.0f, 0.5f, 1.5f));
	std::vector<float> out = player.render(40000, 0.5f, 1.0f);

	CHECK(allFinite(out));
	DOUBLES_EQUAL(220.0, crossingRate(out, 2000, 40000), 220.0 * 0.05);
}

// Each hit should come out once, where the stretch puts it, with its attack intact and nothing of it beforehand.
void checkTransients(float pitch, float stretch) {
	std::vector<float> hits = {0.25f, 0.75f, 1.25f};
	Player player(bursts(hits, 1.5f));
	std::vector<float> out = player.render((int32_t)(1.5f / stretch * kSampleRateF), pitch, stretch);

	CHECK(allFinite(out));
	for (float hit : hits) {
		int32_t expected = (int32_t)(hit / stretch * kSampleRateF);
		int32_t found = firstAbove(out, 0.05f, expected - (int32_t)(0.1f * kSampleRateF));
		CHECK(found >= 0);
		// Within 2 ms of where the hit belongs
		CHECK(std::abs(found - expected) < (int32_t)(0.002f * kSampleRateF));

		// Its attack played at full level
		float peak = 0.0f;
		for (int32_t i = found; i < found + (int32_t)(0.005f * kSampleRateF); i++) {
			peak = std::max(peak, std::abs(out[i]));
		}
		CHECK(peak > 0.6f);
	}
}

TEST(KeyframeStretcherTest, transientsLandOnTheGridSlowingDown) {
	checkTransients(1.0f, 0.5f);
}

TEST(KeyframeStretcherTest, transientsLandOnTheGridSpeedingUp) {
	checkTransients(1.0f, 2.0f);
}

TEST(KeyframeStretcherTest, transientsLandOnTheGridPitchedUp) {
	checkTransients(2.0f, 1.0f);
}

TEST(KeyframeStretcherTest, transientsLandOnTheGridPitchedDown) {
	checkTransients(0.5f, 1.0f);
}

TEST(KeyframeStretcherTest, transientsLandOnTheGridSlowedAndPitchedUp) {
	checkTransients(1.5f, 0.75f);
}

TEST(KeyframeStretcherTest, jumpBackRestartsCleanly) {
	std::vector<float> src = sine(330.0f, 0.5f, 1.0f);
	Player player(src);
	std::vector<float> out = player.render(20000, 1.0f, 1.0f);

	// Loop back to the start, as TimeStretcher::reInit() would
	player.samplePosBig = 0;
	std::vector<float> after = player.render(20000, 1.0f, 1.0f);

	CHECK(allFinite(after));
	// Once the crossfade is done, we're playing the start of the source again
	double num = 0.0, da = 0.0, db = 0.0;
	for (size_t i = 1000; i < 19000; i++) {
		num += (double)after[i] * src[i];
		da += (double)after[i] * after[i];
		db += (double)src[i] * src[i];
	}
	CHECK(num / std::sqrt(da * db) > 0.99);
}

TEST(KeyframeStretcherTest, drainsAfterSourceEnds) {
	Player player(sine(440.0f, 0.5f, 0.25f));
	std::vector<float> out = player.render((int32_t)(0.5f * kSampleRateF), 1.0f, 1.0f);

	CHECK(allFinite(out));
	CHECK(player.engine->sourceEnded());
	CHECK(player.engine->drained());
	CHECK(rms(out, (size_t)(0.3f * kSampleRateF), out.size()) < 0.001f);
}

TEST(KeyframeStretcherTest, stereoChannelsStayIndependent) {
	KeyframeStretcher* engine = KeyframeStretcher::create(2);
	CHECK_EQUAL(2, engine->numChannels());

	std::vector<float> left = sine(220.0f, 0.5f, 1.0f);
	std::vector<float> right = sine(550.0f, 0.5f, 1.0f);
	engine->beginSegment(0);
	std::vector<float> outL, outR;
	int64_t samplePosBig = 0;
	int64_t readPos = 0;
	for (int32_t b = 0; b < 200; b++) {
		double grid = engine->gridFor(samplePosBig, 1);
		int32_t toFeed = engine->framesToFeed(kBlock, grid, 1.0f, 1.0f);
		std::vector<int32_t> frames;
		for (int32_t i = 0; i < toFeed; i++, readPos++) {
			frames.push_back((int32_t)std::lround(left[readPos] * 1073741824.0f));
			frames.push_back((int32_t)std::lround(right[readPos] * 1073741824.0f));
		}
		if (toFeed) {
			engine->feed(frames.data(), toFeed);
		}
		std::vector<int32_t> block(kBlock * 2, 0);
		engine->render(block.data(), kBlock, 2, grid, 1.0f, 1.0f, 2147483647, 0);
		for (int32_t i = 0; i < kBlock; i++) {
			outL.push_back((float)block[2 * i] / 1073741824.0f);
			outR.push_back((float)block[2 * i + 1] / 1073741824.0f);
		}
		samplePosBig += (int64_t)kBlock << 24;
	}
	KeyframeStretcher::destroy(engine);

	DOUBLES_EQUAL(220.0, crossingRate(outL, 2000, outL.size()), 220.0 * 0.03);
	DOUBLES_EQUAL(550.0, crossingRate(outR, 2000, outR.size()), 550.0 * 0.03);
}

TEST(KeyframeStretcherTest, survivesWildModulation) {
	// Noise with hits in it, every block a different pitch and stretch, and jumps all over the place
	std::vector<float> src = bursts({0.1f, 0.3f, 0.35f, 0.7f, 0.9f}, 1.0f);
	uint32_t seed = 12345;
	auto random01 = [&]() {
		seed = seed * 1664525u + 1013904223u;
		return (float)(seed >> 8) / 16777216.0f;
	};
	for (float& v : src) {
		v += 0.05f * (random01() - 0.5f);
	}

	Player player(src);
	std::vector<float> out;
	for (int32_t b = 0; b < 3000; b++) {
		float pitch = 0.25f * std::pow(64.0f, random01());  // 0.25 .. 16
		float stretch = 0.1f * std::pow(40.0f, random01()); // 0.1 .. 4
		if (random01() < 0.02f) {
			player.samplePosBig = (int64_t)(random01() * 1.2f * (float)src.size()) << 24;
		}
		player.renderBlock(out, pitch, stretch);
	}

	CHECK(allFinite(out));
	for (float v : out) {
		CHECK(std::abs(v) < 2.0f);
	}
}

namespace {

// Live input, as LivePitchShifter::renderKeyframe() does it: each render feeds that window's input, then reads back a
// fixed delay behind it, at real-time speed. Window sizes vary, as they do on the Deluge.
std::vector<float> playLive(const std::vector<float>& input, float pitch) {
	KeyframeStretcher* engine = KeyframeStretcher::create(1, KeyframeStretcher::Mode::LIVE);
	engine->beginSegment(0);
	const int32_t delay = KeyframeStretcher::liveDelay(kBlock);
	std::vector<float> out;
	int64_t grid = -1;
	uint32_t seed = 99;
	size_t start = 0;
	while (true) {
		seed = seed * 1664525u + 1013904223u;
		int32_t n = 1 + (int32_t)((seed >> 8) % kBlock);
		if (start + n > input.size()) {
			break;
		}
		std::vector<int32_t> frames(n);
		for (int32_t i = 0; i < n; i++) {
			frames[i] = (int32_t)std::lround(input[start + i] * 1073741824.0f);
		}
		engine->feed(frames.data(), n);
		if (grid < 0 && (int64_t)start >= delay) {
			grid = (int64_t)start - delay;
		}
		std::vector<int32_t> block(n, 0);
		if (grid >= 0) {
			engine->render(block.data(), n, 1, (double)grid, 1.0f, pitch, 2147483647, 0);
			grid += n;
		}
		for (int32_t v : block) {
			out.push_back((float)v / 1073741824.0f);
		}
		start += n;
	}
	KeyframeStretcher::destroy(engine);
	return out;
}

} // namespace

TEST(KeyframeStretcherTest, liveUnityIsTheInputDelayed) {
	std::vector<float> in = sine(440.0f, 0.5f, 1.0f);
	std::vector<float> out = playLive(in, 1.0f);
	const int32_t latency = KeyframeStretcher::liveDelay(kBlock);
	CHECK(latency < (int32_t)(0.03f * kSampleRateF)); // Under 30 ms

	double best = 0.0;
	for (int32_t lag = latency - 4; lag <= latency + 4; lag++) {
		double num = 0.0, da = 0.0, db = 0.0;
		for (size_t i = 4000; i < 40000; i++) {
			num += (double)out[i] * in[i - lag];
			da += (double)out[i] * out[i];
			db += (double)in[i - lag] * in[i - lag];
		}
		best = std::max(best, num / std::sqrt(da * db));
	}
	CHECK(best > 0.995);
}

TEST(KeyframeStretcherTest, livePitchUp) {
	std::vector<float> out = playLive(sine(220.0f, 0.5f, 1.0f), 1.5f);
	CHECK(allFinite(out));
	DOUBLES_EQUAL(330.0, crossingRate(out, 4000, 40000), 330.0 * 0.05);
	DOUBLES_EQUAL(0.5 / std::sqrt(2.0), rms(out, 4000, 40000), 0.06);
}

TEST(KeyframeStretcherTest, livePitchUpOctave) {
	std::vector<float> out = playLive(sine(220.0f, 0.5f, 1.0f), 2.0f);
	CHECK(allFinite(out));
	DOUBLES_EQUAL(440.0, crossingRate(out, 4000, 40000), 440.0 * 0.05);
}

TEST(KeyframeStretcherTest, livePitchDown) {
	std::vector<float> out = playLive(sine(440.0f, 0.5f, 1.0f), 0.5f);
	CHECK(allFinite(out));
	DOUBLES_EQUAL(220.0, crossingRate(out, 4000, 40000), 220.0 * 0.05);
}

TEST(KeyframeStretcherTest, liveTransientsStayOnTime) {
	std::vector<float> hits = {0.25f, 0.75f, 1.25f};
	const int32_t latency = KeyframeStretcher::liveDelay(kBlock);
	for (float pitch : {0.7f, 1.0f, 1.5f}) {
		std::vector<float> out = playLive(bursts(hits, 1.5f), pitch);
		CHECK(allFinite(out));
		for (float hit : hits) {
			int32_t expected = (int32_t)(hit * kSampleRateF) + latency;
			int32_t found = firstAbove(out, 0.05f, expected - (int32_t)(0.1f * kSampleRateF));
			CHECK(found >= 0);
			CHECK(std::abs(found - expected) < (int32_t)(0.002f * kSampleRateF));
		}
	}
}

// Plain re-pitching, like a record: the time rate equals the pitch rate, so the head never leaves the grid
TEST(KeyframeStretcherTest, repitchingChangesPitchAndLengthTogether) {
	Player player(sine(220.0f, 0.5f, 2.0f));
	std::vector<float> out = player.render(40000, 1.5f, 1.5f);

	CHECK(allFinite(out));
	DOUBLES_EQUAL(330.0, crossingRate(out, 2000, 40000), 330.0 * 0.02);
	DOUBLES_EQUAL(0.5 / std::sqrt(2.0), rms(out, 2000, 40000), 0.03);
	DOUBLES_EQUAL(60000.0, (double)(player.samplePosBig >> 24), 300.0); // 1.5x the source for the output
}

TEST(KeyframeStretcherTest, repitchingIsSmooth) {
	// No splices should happen: the output of a sine stays a clean sine, with no jumps between samples bigger than
	// the sine itself can make
	Player player(sine(220.0f, 0.5f, 2.0f));
	std::vector<float> out = player.render(40000, 0.8f, 0.8f);
	const float maxStep = 0.5f * 2.0f * (float)M_PI * 220.0f * 0.8f / kSampleRateF;
	for (size_t i = 2001; i < out.size(); i++) {
		CHECK(std::abs(out[i] - out[i - 1]) < 1.15f * maxStep);
	}
}

TEST(KeyframeStretcherTest, transientsLandOnTheGridRepitchedUp) {
	checkTransients(1.5f, 1.5f);
}

TEST(KeyframeStretcherTest, transientsLandOnTheGridRepitchedDown) {
	checkTransients(0.7f, 0.7f);
}

TEST(KeyframeStretcherTest, keepsTheTopEnd) {
	// Keyframe values come from the raw signal, not the B-spline that finds them, so high frequencies keep their level
	for (float freq : {8000.0f, 12000.0f, 14000.0f}) {
		std::vector<float> src = sine(freq, 0.5f, 1.0f);
		Player player(src);
		std::vector<float> out = player.render(30000, 1.0f, 1.0f);
		CHECK(allFinite(out));
		const double levelDb = 20.0 * std::log10(rms(out, 4000, 30000) / rms(src, 4000, 30000));
		CHECK(levelDb > -1.0 && levelDb < 0.5);
	}
}
