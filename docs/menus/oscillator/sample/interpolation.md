Interpolation mode to use when pitch shifting, which also selects the time stretching algorithm.

Possible values are:
    - <string-for name="STRING_FOR_LINEAR">LINEAR<string-for> - Lower quality but much lower CPU load. Time stretching
      uses the classic algorithm.
    - <string-for name="STRING_FOR_SINC">SINC<string-for> - Higher quality but more CPU intensive. Time stretching
      uses the classic algorithm.
    - <string-for name="STRING_FOR_KEYFRAME">KEYFRAME<string-for> - Time stretching uses the keyframe algorithm from
      capicola, which rebuilds the audio from its waveform peaks and troughs and keeps drum hits crisp and on time.
      Pitch shifting without time stretching works as in SINC.
