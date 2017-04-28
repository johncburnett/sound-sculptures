(
//--------------------------------------------------------------
//_Init

~width = 1000;
~height = 600;

//_Server
s = Server.local;
o = s.options;
o.memSize_(2e6); // 2GB of RAM
o.numInputBusChannels  = 3; // 3 in
o.numOutputBusChannels = 2; // 2 out

//_Debugging
~test = false; // use buffers for audio in

s.waitForBoot({

	//--------------------------------------------------------------
	//_Buffers
	~cwd = thisProcess.nowExecutingPath.dirname +/+ "samples";
	~b1 = Buffer.read(s, ~cwd +/+ "almglocken.wav");
	// ~b1 = Buffer.read(s, ~cwd +/+ "transient.wav");
	// ~b2 = Buffer.read(s, ~cwd +/+ "vox.wav");
	~b2 = Buffer.read(s, ~cwd +/+ "TheBridge.aif");
	s.sync; /* wait for server */

	//--------------------------------------------------------------
	//_Resonance Data

	~fifthGen = { |fundamental=200.0, numFreqs=4, freqRatio=1.5, ampRatio=0.5, amp=0.2|
		var freqs, amps, times;
		freqs = Array(numFreqs).add(fundamental);
		amps  = Array(numFreqs).add(amp);
		times = Array(numFreqs).add(exprand(0.5, 1.5));
		for(1, numFreqs, { |i|
			freqs.add(freqs[i-1] * freqRatio);
			amps.add(amps[i-1] * ampRatio);
			times.add(exprand(10.5, 11.5));
		});
		[freqs, amps, times];
	};

	~spatializer = { |freqAmpSet, numChannels=2|
		var spread = Array.fill(3, {
			Array.fill(numChannels, { Array(freqAmpSet[0].size / numChannels) })
		});
		for(0, freqAmpSet[0].size-1, { |i|
			spread[0][i % numChannels].add(freqAmpSet[0][i]);
			spread[1][i % numChannels].add(freqAmpSet[1][i]);
			spread[2][i % numChannels].add(freqAmpSet[2][i]);
		});
		spread;
	};

	~numVoices = 15;
	~resonance = ~fifthGen.(
		fundamental: 13.75, // fundamental of A
		numFreqs:(~numVoices*1.5)
	);
	~resonance = [
		~resonance[0][4..14],
		~resonance[1][0..12],
		~resonance[2][0..12]
	];
	 ~spread = ~spatializer.(~resonance, numChannels:2);
	~resFreqs = ~spread[0];
	~resAmps  = ~spread[1];
	~resTimes = ~spread[2];

	//--------------------------------------------------------------
	//_Input

	SynthDef(\perc, { |amp=0.0, bufnum=0, rate=1.0, pan=0, send=0.0|
		var in, mix;
		in = [SoundIn.ar(0), SoundIn.ar(1)];
		mix = LPF.ar(in, 3000);
		Out.ar(10, mix * send); // to processing
		Out.ar(20, mix * amp);	// to mixer
		Out.ar(30, in);         // to convolver
	}).add;


	SynthDef(\voice, { |amp=0.0, bufnum=0, rate=1.0, pan=0, send=0.0|
		var in, mix;
		in = SoundIn.ar(2) * 4;
		mix = Pan2.ar(LPF.ar(in, 5000), pan) * amp;
		Out.ar(10, in!2 * send); // to processing
		Out.ar(20, mix * amp); 	 // to mixer
		Out.ar(32, in);          // to convolver
	}).add;


	SynthDef(\looper_perc, { |bufnum=0, rate=1.0, amp=0.0, send=0.0|
		var in = PlayBuf.ar(2, bufnum, rate, loop:1);
		Out.ar(10, in * send); // to processing
		Out.ar(20, in * amp);  // to mixer
		Out.ar(30, in);        // to convolver
	}).add;


	SynthDef(\looper_voice, { |bufnum=0, rate=1.0, amp=0.0, send=0.0|
		var in = PlayBuf.ar(1, bufnum, rate, loop:1).dup;
		Out.ar(10, in * send); // to processing
		Out.ar(20, in * amp);  // to mixer
		Out.ar(32, in);        // to convolver
	}).add;

	//--------------------------------------------------------------
	//_Resonators

	SynthDef(\convolve, { | amp=0.0|
		var in0, in1, conv, mix;
		in0 = In.ar(30, 2);
		in1 = In.ar(32, 2);
		conv = Convolution.ar(in1, in0, 2048, 0.25);
		mix = conv * amp;
		Out.ar(20, mix);
	}).add;


	SynthDef(\resBank, { |mod=1.0, amp=0.0|
		var in, res1, res2, mix;
		in = In.ar(10, 2);

		res1 = Klank.ar(
			`[
				Select.kr(0, ~resFreqs),
				Select.kr(0, ~resAmps),
				2.0!4
			],
			in[0],
			mod
		);

		res2 = Klank.ar(
			`[
				Select.kr(1, ~resFreqs),
				Select.kr(1, ~resAmps),
				2.0!4
			],
			in[1],
			mod
		);

		mix = [res1, res2] * amp * 0.03;
		Out.ar(20, mix);
	}).add;


	SynthDef(\chorus, { |amp=0.0, predelay=0.08, speed=0.05, depth=0.1, ph_diff=0.5|
		var in, mod1, mod2, sig1, sig2, mix, numDelays = 3;
		in = In.ar(10, 2) * numDelays.reciprocal;

		mod1 = Array.fill(numDelays, { |i|
			LFPar.kr(speed * rrand(0.94, 1.06), ph_diff * i, depth, predelay);
		});
		mod2 = Array.fill(numDelays, { |i|
			LFPar.kr(speed * rrand(0.94, 1.06), ph_diff * i, depth, predelay);
		});
		sig1 = DelayC.ar(in[0], 0.5, mod1).sum;
		sig2 = DelayC.ar(in[1], 0.5, mod2).sum;

		mix = [sig1, sig2] * amp;
		Out.ar(20, mix);
	}).add;


	SynthDef(\resonator1, { |amp=0.0, panWidth=0.7|
		var in, res0, res1, mix, pan;
		in = In.ar(10, 2) * amp;
		res0 = Ringz.ar(in[0], {exprand(300, 20000)}.dup(100), 15).mean;
		res1 = Ringz.ar(in[1], {exprand(300, 20000)}.dup(100), 14.8).mean;
		res0 = Pan2.ar(res0, -1 * panWidth);
		res1 = Pan2.ar(res1, panWidth);
		mix = (res0 + res1);
		Out.ar(20, mix * 0.1);
	}).add;


	SynthDef(\resonator2, { |amp=0.0, panWidth=0.7|
		var in, res0, res1, mix, pan;
		in = In.ar(10, 2) * amp;
		res0 = Ringz.ar(in[0], {exprand(300, 20000)}.dup(100), 60).mean;
		res1 = Ringz.ar(in[1], {exprand(300, 20000)}.dup(100), 59.8).mean;
		res0 = Pan2.ar(res0, -1 * panWidth);
		res1 = Pan2.ar(res1, panWidth);
		mix = (res0 + res1);
		Out.ar(20, mix * 0.1);
	}).add;


	SynthDef(\collage,{ |amp=0.0|
		var in, d1, d2, p1, p2, p3, p4, pan, mix;

		in = In.ar(10, 2);

		d1 = Array.fill(6, {
			BPF.ar(
				CombC.ar(in[0], 15.0, rrand(0.5, 10.0), 40),
				rrand(100, 2000),
				0.3)
		} ).sum / 2;

		d2 = Array.fill(6, {
			BPF.ar(
				CombC.ar(in[1], 15.0, rrand(0.5, 10.0), 40),
				rrand(100, 2000),
				0.3)
		} ).sum / 2;

		pan = LinLin.kr(SinOsc.kr(0.25), -1.0, 1.0, -0.6, 0.6);
		d1 = Pan2.ar(d1, pan);
		d2 = Pan2.ar(d2, -1.0*pan);
		mix = [d1, d2] * amp;
		Out.ar(20, mix);
	}).add;

	//--------------------------------------------------------------
	//_Mixer

	SynthDef(\mixer,{ |amp=0.0|
		var in, mix;
		in = In.ar(20, 2);
		mix = Limiter.ar(MidEQ.ar(in, 15000, 0.7, 8)) * amp;
		Out.ar(0, mix);
	}).add;

	//--------------------------------------------------------------
	//_Routing

	s.sync; /* wait for server */

	// mixer
	~mixer = Synth(\mixer, [\amp, 0.0]);

	// ~convolve  = Synth(\convolve,  [\amp, 0.0]);
	~resBank   = Synth(\resBank,   [\amp, 0.0]);
	~chorus    = Synth(\chorus,    [\amp, 0.0]);
	// ~resBank   = Synth(\resBank,   [\amp, 0.0]);
	~resonator1 = Synth(\resonator1, [\amp, 0.0]);
	~resonator2 = Synth(\resonator2, [\amp, 0.0]);
	~collage   = Synth(\collage,   [\amp, 0.0]);

	if(not(~test), {
		~perc  = Synth(\perc,  [\bufnum, ~b1.bufnum]);
		~voice = Synth(\voice, [\bufnum, ~b2.bufnum]);
	},{
		~perc  = Synth(\looper_perc,  [\bufnum, ~b1.bufnum]);
		~voice = Synth(\looper_voice, [\bufnum, ~b2.bufnum]);
	});

	//--------------------------------------------------------------
	//_Main

	~synths = [
		[~perc,      \amp],
		[~perc,      \send],
		[~voice,     \amp],
		[~voice,     \send],
		[~collage,   \amp],
		// [~convolve,  \amp],
		[~chorus,    \amp],
		[~resonator1, \amp],
		[~resonator2, \amp],
		// [~resBank,   \amp],
		[~mixer,     \amp]
	];

	//--------------------------------------------------------------
	//_GUI

	//_Window
	w = Window("GUI", Rect(
		Window.screenBounds.width/2 - (~width/2),
		Window.screenBounds.height/2 - (~height/2),
		~width,
		~height
	))
	.background_(Color(0.1, 0.1, 0.1))
	.front
	.alwaysOnTop_(true)
	.onClose_({ Server.killAll });

	//_Slider Generator
	~sliderGen = { |width = 800, height = 600, x = 0, y = 0, numSliders = 10|
		var sliderSet = Array.newClear(numSliders);
		var valueSet = Array.newClear(numSliders);
		var titleSet = Array.newClear(numSliders);

		for(0, numSliders-1,{ |i|
			var div, val, title;

			// create fader
			div = (width - 40) / numSliders;
			sliderSet[i] = Slider(w, Rect(
				(div * i) + 20 + x,
				50 + y,
				div - 10,
				height - 90
			))
			.background_(Color(0, 0, 0.15))
			.knobColor_(Color.black);

			// show title
			title = TextField(w, Rect(
				(div * i) + 20 + x,
				y + 20,
				div - 10,
				30
			))
			.value_(~synths[i][0].defName)
			.align_(\center);

			titleSet[i] = title;

			// show value
			val = TextField(w, Rect(
				(div * i) + 20 + x,
				height - 40 + y,
				div - 10,
				20
			))
			.value_(0)
			.align_(\center);

			// link fader and text box
			sliderSet[i].action = { |obj|
				val.value_((sliderSet[i].value * 1000).round / 1000)
			};

			valueSet[i] = val;
		});

		[sliderSet, valueSet, titleSet];
	};

	//_Create Faders
	~faders = ~sliderGen.value(~width, ~height, 0, 0, ~synths.size);

	//_Scale and Round Values
	~scale = { |x, inputMin, inputMax, outputMin, outputMax|
		var numer, denom, scaled;
		numer = (outputMax-outputMin) * (x-inputMin);
		denom = inputMax - inputMin;
		scaled = (numer / denom) + outputMin;
		scaled;
	};

	~round = { |n, dec = 1000| (n * dec).round / dec };

	//_Link Faders
	for(0, ~synths.size-1, { |i|
		~faders[0][i].action = { |obj|
			var value = obj.value;
			~synths[i][0].set(~synths[i][1], value);
			~faders[1][i].value_(~round.(value));
		};
	});

	w.front;

	//--------------------------------------------------------------
	//_QuNeo

/*	~voice.set(\amp, ~quNeo.vSliders[5].loc.map);
	~chorus.set(\amp, ~quNeo.vSliders[9].loc.map);
	~resonator.set(\amp, ~quNeo.vSliders[9].loc.map);
	~mixer.set(\amp, ~quNeo.vSliders[13].loc.map);*/
});
)

/*s.prepareForRecord;
s.record;
s.stopRecording;*/