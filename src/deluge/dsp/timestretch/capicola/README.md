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
| `Analyzer.h` | `lib/Analyzer.h` | include paths; each extremum's value is read from the raw signal with an 8-point Lanczos window instead of from the B-spline (which only locates it), with analysis running 3 samples later to give that window its newer samples; boundary keyframes take the raw sample at their own time. Restores the top end (−0.4 dB at 14 kHz, was −6.5 dB) without adding distortion. Changes are marked `[Deluge]`. |
| `Granule.h` | `lib/Granule.h` | include path; `RetriggerAt()`, `SetGrid()`, `Front()` added for a host-owned timeline; `SetMaxLead()` caps a grain's length in time as well as in keyframes; the "grid reached the oldest frame" back-guard only applies to a reversing grid. Changes are marked `[Deluge]`. |
| `OnsetDetector.h` | `lib/Detector.h` | trimmed rewrite: no event history, integer clock, integer-lag spline reads, and reports where the envelope started rising towards each kept peak as the onset |

The Deluge-specific glue (feeding the engine from a `Sample`, keeping it in
sync with the sequencer, transient punch-in) lives in
`../keyframe_stretcher.{h,cpp}`.
