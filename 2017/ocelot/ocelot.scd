(
// Server
s = Server.local;
o = s.options;
o.sampleRate_(44.1e3); 		// 44.1k Hz
o.memSize_(2e6); 			// 2GB of RAM
o.numOutputBusChannels_(2);	// 2ch

// Busses
~procbus = 10;
~mixbus  = 20;

// Globals
~len = 10;

// GUI
~width = 1280;
~height = 800;

if(thisProcess.argv.size != 0, { ~debugIDE = false },{ ~debugIDE = true }); // debug from sc ide

s.waitForBoot({

	//--------------------------------------------------------------
	// Buffers

	~cwd = thisProcess.nowExecutingPath.dirname;
	~b0 = Buffer.read(s, ~cwd +/+ "samples/a11wlk01.wav");

	//
	~rb0 = Buffer.alloc(s, 44100 * ~len, numChannels:1);
	~rb1 = Buffer.alloc(s, 44100 * ~len, numChannels:1);
	~rb2 = Buffer.alloc(s, 44100 * ~len, numChannels:1);
	~rb3 = Buffer.alloc(s, 44100 * ~len, numChannels:1);
	~rb4 = Buffer.alloc(s, 44100 * ~len, numChannels:1);
	~rb5 = Buffer.alloc(s, 44100 * ~len, numChannels:1);

	~rb = [~rb0, ~rb1, ~rb2, ~rb3, ~rb4, ~rb5];

	s.sync; /* wait for server */

	//--------------------------------------------------------------
	// Grains

	SynthDef(\rGrain, { |inBuf, outBuf, amp=0.0, width=0.7, position=0.5, frequency=1.0, splay=1.5, pitch=1.0|
		var trate, dur, center, rate, r1, r2, pan, grains;
		trate = frequency;
		dur = BufDur.kr(inBuf) / trate;
		center = LinLin.kr(position, 0, 1, 0, BufDur.kr(inBuf));
		r1 = LinLin.kr(SinOsc.kr(5 / (2*3.14)), -1.0, 1.0, 0.1, pitch);
		r2 = LinLin.kr(SinOsc.kr(5 / (2*3.14)), -1.0, 1.0, pitch, 0.1);
		rate = Dseq([r1, r2, r2 * splay.reciprocal, r1 * splay.reciprocal, r1 * splay, r2 * splay], inf);
		pan = Dseq([(-1) * width, width], inf);
		grains = TGrains.ar(2, Impulse.ar(trate), inBuf, rate, center, dur, pan, 0.1, 4);

		Out.ar(~mixbus, grains * amp);

		RecordBuf.ar(grains.sum * 8, outBuf);
	}).add;

	//--------------------------------------------------------------
	// Mixer

	SynthDef(\mixer,{ |amp = 0.8|
		var in, mix, lo, mid, hi;
		in = In.ar(~mixbus, 2);
		mix = Limiter.ar(MidEQ.ar(in, 15000, 0.7, 8)) * amp;
		Out.ar(0, mix * 4);
	}).add;

	//--------------------------------------------------------------
	// Routing

	// wait for server
	s.sync;

	// mixer
	~mix = Synth(\mixer);

	// grain group
	~rgroup = ParGroup.new;

	// grains
	~g0 = Synth(\rGrain, [\inBuf, ~b0,  \outBuf, ~rb0], ~rgroup);
	~g1 = Synth(\rGrain, [\inBuf, ~rb0, \outBuf, ~rb1], ~rgroup);
	~g2 = Synth(\rGrain, [\inBuf, ~rb1, \outBuf, ~rb2], ~rgroup);
	~g3 = Synth(\rGrain, [\inBuf, ~rb2, \outBuf, ~rb3], ~rgroup);
	~g4 = Synth(\rGrain, [\inBuf, ~rb3, \outBuf, ~rb4], ~rgroup);

	~grains = [~g0, ~g1, ~g2, ~g3, ~g4, ~rgroup];

	// parameters (param, lo, hi, default)
	~p = [
		[\amp, 0.0, 1.0, 0.0],
		[\width, 0.0, 1.0, 0.7],
		[\position, 0.0, 1.0, 0.0],
		[\frequency, 0.1, 200, 1.0],
		[\splay, 1.0, 3.0, 1.5],
		[\pitch, 0.15, 10.0, 1.0]
	];


	//--------------------------------------------------------------
	// Utilities

	~freeGrains = { for(0, ~grains.size-2, { |i| ~grains[i].free.postln }); };

	~allocBuffer = { |fpath| ~b0.free; ~b0 = Buffer.readChannel(s, fpath, channels:[0]); };

	~createGrains = {
		~g0 = Synth(\rGrain, [\inBuf, ~b0,  \outBuf, ~rb0]);
		~g1 = Synth(\rGrain, [\inBuf, ~rb0, \outBuf, ~rb1]);
		~g2 = Synth(\rGrain, [\inBuf, ~rb1, \outBuf, ~rb2]);
		~g3 = Synth(\rGrain, [\inBuf, ~rb2, \outBuf, ~rb3]);
		~g4 = Synth(\rGrain, [\inBuf, ~rb3, \outBuf, ~rb4]);

		~grains = [~g0, ~g1, ~g2, ~g3, ~g4];
	};

	~init = { |fpath|
		~freeGrains.();
		~allocBuffer.(fpath);
		~createGrains.();
		~resetBank.(~banks);
	};


	//--------------------------------------------------------------
	// GUI

	// Window
	w = Window("Ocelot", Rect(
		Window.screenBounds.width/2 - (~width/2),
		Window.screenBounds.height/2 - (~height/2),
		~width,
		~height
	))
	.alwaysOnTop_(true)
	.background_(Color(0.4, 0.4, 0.4))
	.onClose_({
		Server.killAll;
		s.quit;
		w.close;
		if(not(~debugIDE), {
			thisProcess.shutdown;
			0.exit;
		});
	});

	// generate controls for a granulator
	~grainSliderGroup = { |x, y, width, height, synth, n|
		var coords, title, bank;

		coords = [[0,0], [1,0], [2,0], [0,1], [1,1], [2,1]];
		bank = [];

		title = StaticText.new(w, Rect(x, y+10, width, 30))
		.string_("Granulator " ++ n)
		.font_(Font("Helvetica", 18))
		.align_(\center);

		if(n==6, { title.string_("All Granulators") });

		y = y + 30;
		height = height - 30;

		for(0, ~p.size-1, { |i|
			var spec, slider, posX, posY;
			if((i==3) || (i==5),
				{ spec = ControlSpec(~p[i][1], ~p[i][2], \exp) },
				{ spec = ControlSpec(~p[i][1], ~p[i][2]) });
			posX = x + ((width/3)*coords[i][0]);
			posY = y + ((height/2)*coords[i][1]);
			slider = EZSlider( w,  Rect( posX, posY, (width/3), (height/2)),
				~p[i][0].asString, spec, initVal:~p[i][3], margin:(2@5), layout:\vert )
			.action_({ |sl| synth.set(~p[i][0], sl.value) })
			.setColors(Color.grey, Color.black, Color.grey(0.7), Color.grey,
            Color.white, Color.yellow, nil, nil, nil);
			bank = bank ++ slider;
		});

		bank;
	};

	// generate bank for each granulator
	~bankGroup = { |x, y, width, height|
		var coords, banks;
		coords = [
			[x, y], [x+(width/3), y], [x+(width*(2/3)), y],
			[x, y+(height/2)], [x+(width/3), y+(height/2)], [x+(width*(2/3)), y+(height/2)]
		];
		banks = [];

		for(0, ~grains.size-1, { |i|
			var group = ~grainSliderGroup.(coords[i][0], coords[i][1], width/4, height/2, ~grains[i], i+1);
			banks = banks ++ [group];
		});

		for(0, ~p.size-1,{ |i|
			banks[5][i].action_({ |sl|
				~grains[5].set(~p[i][0], sl.value);
				for(0, ~grains.size-2, { |j| banks[j][i].value_(sl.value) });
			});
		});

		banks;
	};

	~resetBank = { |bank|
		for(0, bank.size-1, { |i| bank[i].remove });
		~banks = ~bankGroup.(30, -10, ~width*(3/4), ~height);
	};

	~banks = ~bankGroup.(30, -10, ~width*(3/4), ~height);

	// dividers
	w.drawFunc_{
		Pen.strokeColor = Color.black;
		Pen.width = 2;
		Pen.line( (3*~width/4)@0, (3*~width/4)@~height ); Pen.stroke;
		for(1, 2,{ |i| Pen.line( (i*~width/4)@0, (i*~width/4)@~height ); Pen.stroke; });
		Pen.line( 0@(~height/2), (3*~width/4)@(~height/2) );
		Pen.stroke;
	};

	// title
	~title = StaticText.new(w, Rect(~width*(3/4), ~height*(1/3)-20, ~width/4, ~height/4))
	.string_("Ocelot\nrecursive granulator")
	.font_(Font("Helvetica", 24))
	.align_(\center);

	// select file
	~loadFile = Button(w, Rect(~width*(3/4) + (~width/16), ~height/2, ~width/8, ~height/16))
	.states_([
		["load audio file", Color.white, Color.black]
	])
	.action_({ Dialog.openPanel({ |path| ~init.(path); path.postln; }); });

	// ocelot
	if(~debugIDE, {
		~catpic = QtGUI.image.new(~cwd +/+ "cat.png");
		~container = CompositeView.new(w,Rect(~width*(3/4)+10, 10, ~width/4, ~width/4))
		.backgroundImage_(~catpic);
	});

	// master volume
	~vol = EZSlider(w, Rect(~width*(3/4) + (~width*(1/12)), ~height*(3/5), ~width*(1/12), ~height/3),
		"Volume", \db.asSpec.step_(0.01), unitWidth:(~width*(1/12)), initVal:0.8, layout:\vert)
	.action_({ |sl| ~mix.set(\amp, sl.value) })
	.setColors(Color.grey,Color.white, Color.grey(0.7),Color.grey,
            Color.white, Color.yellow,nil,nil, Color.grey(0.7));

	w.front;
});
)


