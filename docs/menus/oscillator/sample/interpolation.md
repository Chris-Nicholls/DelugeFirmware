Interpolation mode to use when pitch shifting, which also selects the time stretching algorithm.

Possible values are:
    - <string-for name="STRING_FOR_LINEAR">LINEAR<string-for> - Lower quality but much lower CPU load. Time stretching
      uses the classic algorithm.
    - <string-for name="STRING_FOR_SINC">SINC<string-for> - Higher quality but more CPU intensive. Time stretching
      uses the classic algorithm.
    - <string-for name="STRING_FOR_KEYFRAME">KEYFRAME<string-for> - Uses the keyframe algorithm from capicola for all
      of it: time stretching, pitch shifting, and plain changes of pitch and speed together. It rebuilds the audio from
      its waveform peaks and troughs, and keeps drum hits crisp and on time. On live audio input it runs about 23 ms
      behind the input.
