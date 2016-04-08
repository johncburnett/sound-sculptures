(
/**
* Performance Code for meshes by John Burnett (c)2016
*
* Instructions: Edit -> Evaluate File
*
* Quad:
*       1    2
*
*       3    4
*
*/

/**************************************************************************************************/
//_Init

//_Routing
~num_inputs = 2; // 1 or 2
~quad = True;   // stereo or quad


//_GUI
~width = 1000;
~height = 600;


s = Server.local;
o = s.options;
o.memSize_(2e6); // 2GB
o.numOutputBusChannels = 4;


s.waitForBoot({

	/**************************************************************************************************/
	//_Synths

	SynthDef(\Input,{ |amp = 1.0|
		var in, in2, in3, mix;

		if( ~num_inputs == 2,
			{ in = [SoundIn.ar(0), SoundIn.ar(1)] },
			{ in = SoundIn.ar(0)!2 });

		mix = in * amp;
		Out.ar(10, mix);
	}).load(s);


	SynthDef(\Dry,{ |amp = 0.0|
		var in = In.ar(10, 2) * amp;
		if( ~quad == True,
			{ Out.ar(20, in ++ in) },
			{ Out.ar(20, in) });
	}).load(s);


	SynthDef(\Waves,{ |amp = 0.0|
		var in1, in2, lfo1, lfo2, pan, p1, p2, p3, p4, mix;

		in1 = In.ar(10, 1);
		in2 = In.ar(11, 1);

		lfo1 = LinLin.kr(
			SinOsc.kr( LinLin.kr(SinOsc.ar(1/16), -1.0, 1.0, 0.25, 1) ),
			-1.0,
			1.0,
			1000,
			5000);

		lfo2 = LinLin.kr(
			SinOsc.kr( LinLin.kr(SinOsc.ar(1/16), -1.0, 1.0, 0.25, 1) ),
			-1.0,
			1.0,
			5000,
			1000);

		pan = LinLin.kr(SinOsc.kr(1/8), -1.0, 1.0, -0.7, 0.7);

		in1 = LPF.ar(in1, lfo1);
		in2 = HPF.ar(in2, lfo2);

		if( ~quad == True,{
			p1 = Pan4.ar(in1, pan, pan);
			p2 = Pan4.ar(in2, -1*pan, pan);
			p3 = Pan4.ar(in1, pan, -1*pan);
			p4 = Pan4.ar(in2, -1*pan, -1*pan);
			mix = [p1, p2, p3, p4] * amp;
		},{
			in1 = Pan2.ar(in1, pan);
			in2 = Pan2.ar(in2, -1 * pan);
			mix = [in1, in2] * amp;
		});

		Out.ar(20, mix);
	}).load(s);


	SynthDef(\Collage,{ |amp = 0.0|
		var in, d1, d2, p1, p2, p3, p4, pan, mix;

		in = In.ar(10, 2);

		d1 = Array.fill(6, {
			BPF.ar(
				CombC.ar(in[0], 15.0, rrand(0.5, 10.0), 20),
				rrand(200, 10000),
				0.3)
		} ).sum / 2;

		d2 = Array.fill(6, {
			BPF.ar(
				CombC.ar(in[1], 15.0, rrand(0.5, 10.0), 20),
				rrand(200, 10000),
				0.3)
		} ).sum / 2;

		pan = LinLin.kr(SinOsc.kr(0.25), -1.0, 1.0, -0.6, 0.6);

		if( ~quad == True,{
			p1 = Pan4.ar(d1, pan, pan);
			p2 = Pan4.ar(d2, -1*pan, pan);
			p3 = Pan4.ar(d1, pan, -1*pan);
			p4 = Pan4.ar(d2, -1*pan, -1*pan);
			mix = [p1, p2, p3, p4] * amp;
		},{
			d1 = Pan2.ar(d1, pan);
			d2 = Pan2.ar(d2, -1.0*pan);
			mix = [d1, d2] * amp;
		});

		Out.ar(20, mix);
	}).load(s);


	SynthDef(\Melting,{ |amp = 0.0|
		var in, d1, d2, d3, d4, pan, mix;

		in = In.ar(10, 2);

		d1 = Array.fill(6, {
			BPF.ar(
				CombC.ar(
					in[0],
					15.0,
					LinLin.kr( SinOsc.kr(rrand(1/16, 1/32)), -1.0, 1.0, rrand(4.0, 6.0), rrand(8.0, 12.0) ),
					10),
				rrand(200, 10000),
				0.3)
		} ).sum / 2;

		d2 = Array.fill(6, {
			BPF.ar(
				CombC.ar(
					in[1],
					15.0,
					LinLin.kr( SinOsc.kr(rrand(1/16, 1/32)), -1.0, 1.0, rrand(4.0, 6.0), rrand(8.0, 12.0) ),
					10),
				rrand(200, 10000),
				0.3)
		} ).sum / 2;

		pan = LinLin.kr(SinOsc.kr(0.125), -1.0, 1.0, -0.6, 0.6);
		d1 = Pan2.ar(d1, pan);
		d2 = Pan2.ar(d2, -1.0*pan);
		mix = [d1, d2] * amp;
		Out.ar(20, mix);
	}).load(s);


	SynthDef(\Reverb,{ |amp = 0.0|
		var input, in, predelay, reverb, full, low, lowmid, highmid, high, pan, mix;

		input = In.ar(10, 2);

		low     = BBandPass.ar(input,250,0.25).sum;
		lowmid  = BBandPass.ar(input,1000,0.25).sum;
		highmid = BBandPass.ar(input,4000,0.25).sum;
		high    = BBandPass.ar(input,8000,0.25).sum;

		in = [low + highmid, lowmid + high] * 0.125;
		predelay = DelayN.ar(in, 0.1, [0.044, 0.048, 0.052, 0.526]);
		reverb = Mix.ar(Array.fill(7,{ CombL.ar(predelay, 0.05, rrand(0.04, 0.05), 15) }));
		6.do({ reverb = AllpassN.ar(reverb, 0.050, rrand(0.04, 0.05), 1) });

		pan = LinLin.kr(SinOsc.kr(0.125), -1.0, 1.0, -0.6, 0.6);

		if( ~quad == True,
			{
				reverb[0] = Pan4.ar(reverb[0], pan, pan);
				reverb[1] = Pan4.ar(reverb[1], -1*pan, pan);
				reverb[2] = Pan4.ar(reverb[2], pan, -1*pan);
				reverb[3] = Pan4.ar(reverb[3], -1*pan, -1*pan);
				mix = reverb * amp;
			},{
				reverb[0] = Pan2.ar(reverb[0] + reverb[2], pan);
				reverb[1] = Pan2.ar(reverb[1] + reverb[3], -1*pan);
				mix = reverb * 0.5 * amp;
			}
		);

		Out.ar(20, mix);
	}).load(s);


	SynthDef(\Master,{ |amp = 1.0|
		var in, mix;
		if( ~quad == True,
			{ in = In.ar(20, 4) },
			{ in = In.ar(20, 2) });
		mix = in * 0.5 * amp;
		Out.ar(0, mix);
	}).load(s);


	/**************************************************************************************************/
	//_Main

	g = Synth(\Master);

	f = Synth(\Reverb);
	e = Synth(\Melting);
	d = Synth(\Collage);
	c = Synth(\Waves);
	b = Synth(\Dry);

	a = Synth(\Input);


	~synths = [a, b, c, d, e, f, g];


	/**************************************************************************************************/
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
	.alwaysOnTop_(true);

	s.meter;


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
			));
			title.value = ~synths[i].defName;
			titleSet[i] = title;

			// show value
			val = TextField(w, Rect(
				(div * i) + 20 + x,
				height - 40 + y,
				div - 10,
				20
			))
			.value_(0);

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
			NodeControl(~synths[i], 0).value = value;
			~faders[1][i].value_(~round.value(value));
		};
	});


	//_Initial Value
	~faders[0][0].value_(1.0);
	~faders[0][6].value_(1.0);
});
)
