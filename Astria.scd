(
/**
* Performance Code for Astria by John Burnett (c)2015
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

~quad = False;

s = Server.local;
o = s.options;
o.memSize_(2e6); // 2GB


//creates harmonic array of frequencies
~harmonic = [Array(8), Array(8), Array(8), Array(8)];
i = 1;
while( {i < 14},{
	for(0, 5,{ arg j;
		~harmonic[j].add(2 ** (i + j));
		~harmonic[j].add( (2**(i+j)) + (2**(i+j)/2) );
	});
	i = i + 4;
});


//creates inharmonic array of frequencies
~inharmonic = [Array(16), Array(16), Array(16), Array(16)];
for (0, 64,{ arg i;
	if( i%4 == 0,{
		~inharmonic[0].add(i*45.3838 + 22.0226)});
	if( i%4 == 1,{
		~inharmonic[1].add(i*45.3838 + 22.0226)});
	if( i%4 == 2,{
		~inharmonic[2].add(i*45.3838 + 22.0226)});
	if( i%4 == 3,{
		~inharmonic[3].add(i*45.3838 + 22.0226)});
});


//combines ~harmonic and ~inharmonic
~hybrid = {
	a = Array(4);
	for(0, 4,{ arg i;
		a.add(~harmonic[i] ++ ~inharmonic[i]);
	});
	a;
}.value();


s.waitForBoot({

	/**************************************************************************************************/
	//_Synths


	/**
	* Oboe
	*
	* Takes in vertical X-Y microphrone pair.
	* This creates a stereo image of the
	* linear frequency spectrum of the oboe.
	*/
	SynthDef("Oboe",{ arg dry = 0.0, toProc1 = 0.0, toProc2 = 0.0;
		var source;
		source = [SoundIn.ar(0), SoundIn.ar(1)] * 6;

		Out.ar(0, source * dry * 0.1);
		Out.ar(20, source ++ source);
		Out.ar(30, source ++ source * toProc1);
		Out.ar(50, source ++ source * toProc2);
	}).load(s);


	//resonates harmonic frequency set
	SynthDef("Resonator1",{ arg dry = 0.0, toProc1 = 0.0, toProc2 = 0.0, decayTime = 4.0;
		var tones1, tones2, tones3, tones4;
		tones1 = Klank.ar(`[Select.kr(0, ~harmonic), nil, decayTime!8], In.ar(20)) * 0.06;
		tones2 = Klank.ar(`[Select.kr(1, ~harmonic), nil, decayTime!8], In.ar(21)) * 0.06;
		tones3 = Klank.ar(`[Select.kr(2, ~harmonic), nil, decayTime!8], In.ar(20)) * 0.06;
		tones4 = Klank.ar(`[Select.kr(3, ~harmonic), nil, decayTime!8], In.ar(21)) * 0.06;

		//to CombShift
		Out.ar(30, tones1 * toProc1);
		Out.ar(31, tones2 * toProc1);
		Out.ar(32, tones3 * toProc1);
		Out.ar(33, tones4 * toProc1);

		//to DownShift
		Out.ar(50, tones1 * toProc2);
		Out.ar(51, tones2 * toProc2);
		Out.ar(52, tones3 * toProc2);
		Out.ar(53, tones4 * toProc2);

		//to Mixer
		Out.ar(40, tones1 * dry * 1.2);
		Out.ar(41, tones2 * dry * 1.2);
		Out.ar(42, tones3 * dry * 1.2);
		Out.ar(43, tones4 * dry * 1.2);
	}).load(s);


	//resonates inharmonic frequency set
	SynthDef("Resonator2",{ arg dry = 0.0, toProc1 = 0.0, toProc2 = 0.0, decayTime = 6.0;
		var tones1, tones2, tones3, tones4;
		tones1 = Klank.ar(`[Select.kr(0, ~inharmonic), nil, decayTime!16], In.ar(20)) * 0.01;
		tones2 = Klank.ar(`[Select.kr(1, ~inharmonic), nil, decayTime!16], In.ar(21)) * 0.01;
		tones3 = Klank.ar(`[Select.kr(2, ~inharmonic), nil, decayTime!16], In.ar(20)) * 0.01;
		tones4 = Klank.ar(`[Select.kr(3, ~inharmonic), nil, decayTime!16], In.ar(22)) * 0.01;

		//to CombShift
		Out.ar(30, tones1 * toProc1);
		Out.ar(31, tones2 * toProc1);
		Out.ar(32, tones3 * toProc1);
		Out.ar(33, tones4 * toProc1);

		//to DownShift
		Out.ar(50, tones1 * toProc2);
		Out.ar(51, tones2 * toProc2);
		Out.ar(52, tones3 * toProc2);
		Out.ar(53, tones4 * toProc2);

		//to Mixer
		Out.ar(40, tones1 * dry);
		Out.ar(41, tones2 * dry);
		Out.ar(42, tones3 * dry);
		Out.ar(43, tones4 * dry);
	}).load(s);


	/**
	* CombShift
	*
	* Bends pitch into a double wedge
	* according to the amplitude and
	* then comb filters each bend
	* according to frequency.
	*/
	SynthDef("CombShift",{
		var source, amp, pitch, p1, p2, p3, p4;
		source = In.ar(20);
		//amplitude mapping to pitch ratio
		amp = Amplitude.ar(source);
		//pitch mapping to delay time
		pitch = LinLin.ar(Pitch.kr(source), 0, 3000, 0.1, 0.5);

		p1 = CombC.ar(
			PitchShift.ar(
				In.ar(30, 1),
				pitchRatio:LinLin.ar(amp, 0.0, 1.0, 1.0, 2.0)
			),
			2.0,
			pitch,
			Rand(0.25, 2.0),
			0.45
		);
		p2 = CombC.ar(
			PitchShift.ar(
				In.ar(31, 1),
				pitchRatio:LinLin.ar(amp, 0.0, 1.0, 1.0, 0.1)
			),
			2.0,
			pitch * 0.9,
			Rand(0.25, 2.0),
			0.45
		);
		p3 = CombC.ar(
			PitchShift.ar(
				In.ar(32, 1),
				pitchRatio:LinLin.ar(amp, 0.0, 1.0, 1.0, 1.5)
			),
			2.0,
			pitch * 1.1,
			Rand(0.25, 2.0),
			0.45
		);
		p4 = CombC.ar(
			PitchShift.ar(
				In.ar(33, 1),
				pitchRatio:LinLin.ar(amp, 0.0, 1.0, 1.0, 0.5)
			),
			2.0,
			pitch * 0.8,
			Rand(0.25, 2.0),
			0.45
		);

		Out.ar(40, p1);
		Out.ar(41, p2);
		Out.ar(42, p3);
		Out.ar(43, p4);
	}).load(s);


	/**
	* DownShift
	*
	* Shifts each stream down by a P12
	*/
	SynthDef("DownShift",{
		var p1, p2, p3, p4;
		p1 = PitchShift.ar(
			In.ar(50),
			pitchRatio:0.4
		);
		p2 = PitchShift.ar(
			In.ar(51),
			pitchRatio:0.4
		);
		p3 = PitchShift.ar(
			In.ar(52),
			pitchRatio:0.4
		);
		p4 = PitchShift.ar(
			In.ar(53),
			pitchRatio:0.4
		);

		Out.ar(40, p1);
		Out.ar(41, p2);
		Out.ar(42, p3);
		Out.ar(43, p4);
	}).load(s);


	/**
	* Mixer
	*/
	SynthDef("Mixer",{ arg master = 0.0;
		var pan1, pan2, pan3, pan4, panRate, mix1, mix2;
		panRate = SinOsc.ar(LinLin.kr(Amplitude.ar(In.ar(20)), 0.0, 1.0, 0.25, 1.25));
		if (~quad == True,
			{
				pan1 = Pan4.ar(In.ar(40), panRate, panRate);
				pan2 = Pan4.ar(In.ar(41), panRate * (-1), panRate);
				pan3 = Pan4.ar(In.ar(42), panRate, panRate * (-1));
				pan4 = Pan4.ar(In.ar(43), panRate * (-1), panRate * (-1));
				Out.ar(0, pan1 * master);
				Out.ar(0, pan2 * master);
				Out.ar(0, pan3 * master);
				Out.ar(0, pan4 * master);
			},{
				mix1 = In.ar(40) + In.ar(43);
				mix2 = In.ar(41) + In.ar(42);
				pan1 = Pan2.ar(mix1, panRate * 0.7);
				pan2 = Pan2.ar(mix2, panRate * (-1) * 0.7);
				Out.ar(0, pan1 * master * 0.05);
				Out.ar(0, pan2 * master * 0.05);
		});
	}).load(s);


	/**
	* Glacial freeze
	*
	* Resonates all sound running to the mixer.
	*/
	SynthDef("Glacier",{ arg vol = 0.0, freezeTime = 10;
		var tones1, tones2, tones3, tones4, mix1, mix2;
		freezeTime = freezeTime!24;
		tones1 = Klank.ar(`[Select.kr(0, ~hybrid), nil, freezeTime], In.ar(40)) * 0.001;
		tones2 = Klank.ar(`[Select.kr(1, ~hybrid), nil, freezeTime], In.ar(41)) * 0.001;
		tones3 = Klank.ar(`[Select.kr(2, ~hybrid), nil, freezeTime], In.ar(42)) * 0.001;
		tones4 = Klank.ar(`[Select.kr(3, ~hybrid), nil, freezeTime], In.ar(43)) * 0.001;

		mix1 = tones1 + tones4;
		mix2 = tones2 + tones3;
		Out.ar(0, mix1 * vol * 0.02);
		Out.ar(1, mix2 * vol * 0.02);
	}).load(s);


/**************************************************************************************************/
//_Main

~mix = Synth(\Mixer);
~freeze = Synth(\Glacier);
~frozen = Synth(\Glacier, [\freezeTime, 60]);
~proc1 = Synth(\CombShift);
~proc2 = Synth(\DownShift);
~res1 = Synth(\Resonator1);
~res2 = Synth(\Resonator2);
~oboe = Synth(\Oboe);


/**************************************************************************************************/
//_GUI
//_Window

~windowGUI = Window.new("Astria", Rect.new(
	Window.screenBounds.width/2 - (1120/2),
	Window.screenBounds.height/2 - (400/2),
	1120,
	400
))
.front
.alwaysOnTop_(true);

//dry
~label1 = TextField(~windowGUI, Rect(30, 50, 80, 30));
~label1.value = "Dry";
~value1 = TextField(~windowGUI, Rect(30, 320, 80, 30));
~value1.value = 0;
~value1.background = Color.yellow;
~fader1 = Slider(~windowGUI, Rect(30, 80, 80, 240));
~fader1.action = { arg slider;
	NodeControl(~oboe, 0).value = slider.value;
	~value1.value = (slider.value * 10000).round / 10000;
};

//dry to proc1
~label2 = TextField(~windowGUI, Rect(110, 50, 80, 30));
~label2.value = "CombShift";
~value2 = TextField(~windowGUI, Rect(110, 320, 80, 30));
~value2.value = 0;
~value2.background = Color.yellow;
~fader2 = Slider(~windowGUI, Rect(110, 80, 80, 240));
~fader2.action = { arg slider;
	NodeControl(~oboe, 1).value = slider.value;
	~value2.value = (slider.value * 10000).round / 10000;
};

//dry to proc2
~label3 = TextField(~windowGUI, Rect(190, 50, 80, 30));
~label3.value = "DownShift";
~value3 = TextField(~windowGUI, Rect(190, 320, 80, 30));
~value3.value = 0;
~value3.background = Color.yellow;
~fader3 = Slider(~windowGUI, Rect(190, 80, 80, 240));
~fader3.action = { arg slider;
	NodeControl(~oboe, 2).value = slider.value;
	~value3.value = (slider.value * 10000).round / 10000;
};

//res1
~label4 = TextField(~windowGUI, Rect(310, 50, 80, 30));
~label4.value = "Resonator1";
~value4 = TextField(~windowGUI, Rect(310, 320, 80, 30));
~value4.value = 0;
~value4.background = Color.yellow;
~fader4 = Slider(~windowGUI, Rect(310, 80, 80, 240));
~fader4.action = { arg slider;
	NodeControl(~res1, 0).value = slider.value;
	~value4.value = (slider.value * 10000).round / 10000;
};

//res1 to proc1
~label5 = TextField(~windowGUI, Rect(390, 50, 80, 30));
~label5.value = "CombShift";
~value5 = TextField(~windowGUI, Rect(390, 320, 80, 30));
~value5.value = 0;
~value5.background = Color.yellow;
~fader5 = Slider(~windowGUI, Rect(390, 80, 80, 240));
~fader5.action = { arg slider;
	NodeControl(~res1, 1).value = slider.value;
	~value5.value = (slider.value * 10000).round / 10000;
};

//res1 to proc2
~label6 = TextField(~windowGUI, Rect(470, 50, 80, 30));
~label6.value = "DownShift";
~value6 = TextField(~windowGUI, Rect(470, 320, 80, 30));
~value6.value = 0;
~value6.background = Color.yellow;
~fader6 = Slider(~windowGUI, Rect(470, 80, 80, 240));
~fader6.action = { arg slider;
	NodeControl(~res1, 2).value = slider.value;
	~value6.value = (slider.value * 10000).round / 10000;
};

//res2
~label7 = TextField(~windowGUI, Rect(590, 50, 80, 30));
~label7.value = "Resonator2";
~value7 = TextField(~windowGUI, Rect(590, 320, 80, 30));
~value7.value = 0;
~value7.background = Color.yellow;
~fader7 = Slider(~windowGUI, Rect(590, 80, 80, 240));
~fader7.action = { arg slider;
	NodeControl(~res2, 0).value = slider.value;
	~value7.value = (slider.value * 10000).round / 10000;
};

//res2 to proc1
~label8 = TextField(~windowGUI, Rect(670, 50, 80, 30));
~label8.value = "CombShift";
~value8 = TextField(~windowGUI, Rect(670, 320, 80, 30));
~value8.value = 0;
~value8.background = Color.yellow;
~fader8 = Slider(~windowGUI, Rect(670, 80, 80, 240));
~fader8.action = { arg slider;
	NodeControl(~res2, 1).value = slider.value;
	~value8.value = (slider.value * 10000).round / 10000;
};

//res2 to proc2
~label9 = TextField(~windowGUI, Rect(750, 50, 80, 30));
~label9.value = "DownShift";
~value9 = TextField(~windowGUI, Rect(750, 320, 80, 30));
~value9.value = 0;
~value9.background = Color.yellow;
~fader9 = Slider(~windowGUI, Rect(750, 80, 80, 240));
~fader9.action = { arg slider;
	NodeControl(~res2, 2).value = slider.value;
	~value9.value = (slider.value * 10000).round / 10000;
};

//master
~label10 = TextField(~windowGUI, Rect(870, 50, 80, 30));
~label10.value = "Master";
~value10 = TextField(~windowGUI, Rect(870, 320, 80, 30));
~value10.value = 0.0;
~value10.background = Color.yellow;
~fader10 = Slider(~windowGUI, Rect(870, 80, 80, 240));
~fader10.action = { arg slider;
	NodeControl(~mix, 0).value = slider.value;
	~value10.value = (slider.value * 10000).round / 10000;
};

//freeze (10 second decay)
~label11 = TextField(~windowGUI, Rect(950, 50, 80, 30));
~label11.value = "Freeze";
~value11 = TextField(~windowGUI, Rect(950, 320, 80, 30));
~value11.value = 0.0;
~value11.background = Color.yellow;
~fader11 = Slider(~windowGUI, Rect(950, 80, 80, 240));
~fader11.action = { arg slider;
	NodeControl(~freeze, 0).value = slider.value;
	~value11.value = (slider.value * 10000).round / 10000;
};

//frozen (1 minute decay)
~label12 = TextField(~windowGUI, Rect(1030, 50, 80, 30));
~label12.value = "Frozen";
~value12 = TextField(~windowGUI, Rect(1030, 320, 80, 30));
~value12.value = 0.0;
~value12.background = Color.yellow;
~fader12 = Slider(~windowGUI, Rect(1030, 80, 80, 240));
~fader12.action = { arg slider;
	NodeControl(~frozen, 0).value = slider.value;
	~value12.value = (slider.value * 10000).round / 10000;
};

});

)