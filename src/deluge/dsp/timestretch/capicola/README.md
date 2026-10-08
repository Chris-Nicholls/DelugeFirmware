# Capicola keyframe engine (vendored)

DSP core of [capicola](https://github.com/heavylight-industries/capicola) by
heavylight-industries: the keyframe time-stretching engine described in the
DAFx26 paper *Keyframe Time Stretching via Extrema Sampling*.

Imported from upstream commit `120b0b843c18bda373d96eb79d49805545f61556`.

**License:** GNU Affero General Public License v3.0 (upstream `LICENSE`). The
rest of the Deluge firmware is GPL-3.0-or-later; GPLv3 section 13 explicitly
permits combining the two, with these files remaining under the AGPL.

This directory has its own `.clang-format` that disables formatting, so the
files stay diffable against upstream.

## Files

| File | Upstream | Changes |
|---|---|---|
| `SparseLine.h` | `lib/SparseLine.h` | none |
| `DeluxeLine.h` | `lib/DeluxeLine.h` | none |
| `filter.h` | `lib/filter.h` | none |
| `Analyzer.h` | `lib/Analyzer.h` | include paths; a frame's time is only converted to double when it's about to be stored (an int64 → double conversion is a library call on ARM, and was being made every sample). Changes are marked `[Deluge]`. |
| `Granule.h` | `lib/Granule.h` | include path; `RetriggerAt()`, `SetGrid()`, `Front()` added for a host-owned timeline; `SetMaxLead()` caps a grain's length in time as well as in keyframes; the "grid reached the oldest frame" back-guard only applies to a reversing grid; `BeginBlock()` no longer walks every new keyframe each block to track the delayed tap and the live edge's keyframe index (neither is used per block - the tap is now found by binary search when a re-anchor needs it). Changes are marked `[Deluge]`. |
| `OnsetDetector.h` | `lib/Detector.h` | trimmed rewrite: no event history, integer clock, integer-lag spline reads, and reports where the envelope started rising towards each kept peak as the onset |

The Deluge-specific glue (feeding the engine from a `Sample`, keeping it in
sync with the sequencer, transient punch-in) lives in
`../keyframe_stretcher.{h,cpp}`.
