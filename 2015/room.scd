(
/*
* We are sitting in a room - John Burnett (c)2015
*
* Instructions: Language -> Evaluate File
*
* IO:
* in - 1ch
* out - 2ch
*
* Structure:
* Initial recording
* for num_iterations:
*     -rec_buf -> pb_buf
*     -clear rec_buf
*     -play pb_buf, record rec_buf
* Close and free buffers
*/


/**************************************************************************************************/
//_Init

s = Server.local;

~iter = 50;
~len = 10.0;
~num_channels = 1;


s.waitForBoot({

	~pb_buf = Buffer.alloc(s, 44100 * ~len, ~num_channels);
	~rec_buf = Buffer.alloc(s, 44100 * ~len, ~num_channels);

	/**************************************************************************************************/
	//_Synths

	SynthDef(\record,{ |bufnum=0|
		var in, formant;
		in = SoundIn.ar(0, ~num_channels);
		formant = XLine.kr(400, 1000, ~len, doneAction: 2);
		RecordBuf.ar(in, bufnum);
	}).load(s);


	SynthDef(\playback,{ |bufnum=0|
		var playbuf = PlayBuf.ar(~num_channels ,bufnum);
		FreeSelfWhenDone.kr(playbuf);
		Out.ar(0, playbuf);
	}).load(s);


	/**************************************************************************************************/
	//_Main

	Routine {
		"Initial recording.".postln;
		Synth.new(\record, [\bufnum, ~rec_buf.bufnum], s);
		(~len + 0.2).wait;

		~iter.do { |i|
			("Iteration " + (i+1)).postln;
			~pb_buf = ~rec_buf;
			~rec_buf = Buffer.alloc(s, 44100 * ~len, ~num_channels);

			Synth.new(\playback, [\bufnum, ~pb_buf.bufnum]);
			Synth.new(\record, [\bufnum, ~rec_buf.bufnum]);
			(~len+0.2).wait;
			~pb_buf.close; ~pb_buf.free;
			~pb_buf = Buffer.alloc(s, 44100 * ~len, ~num_channels);
		};

		~pb_buf.close; ~pb_buf.free;
		~rec_buf.close; ~rec_buf.free;
	}.play;
});
)


// (
// //_Recording
// s.prepareForRecord;
// s.record;
// s.stopRecording;
// )
