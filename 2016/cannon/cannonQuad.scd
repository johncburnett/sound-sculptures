(
/*
 * canonQuad - 4ch live looper and layer generator
 *
 * by  john burnett (c) 2016
 * for jonathon nussman
 */

//--------------------------------------------------------------
//_Init

//_Server
s = Server.local;
o = s.options;
o.memSize_(2e6);            // 2GB of RAM
o.sampleRate_(44.1e3); 		// 44.1k Hz
o.numInputBusChannels  = 2; // 1 in
o.numOutputBusChannels = 4; // 2 out

//_Globals
~len = 120; // max recording time in minutes
~len = ~len * 60;
~layers = 0;
~time = 0.0;
~rectime = 0.0;
~bufisempty = true;
~isrecording = false;

//_Busses
~procbus = 20;
~mixbus  = 10;

//_GUI
~width  = 400;
~height = 420;

//_Debugging
~displayGUI = true;
~debugServer = false;
if(thisProcess.argv.size != 0, { ~debugIDE = false },{ ~debugIDE = true }); // debug from sc ide

s.waitForBoot({

	//--------------------------------------------------------------
	//_Buffers

	~recbuf = Buffer.alloc(s, 44100 * ~len, numChannels:1);

	~livegroup = Group.new;

	// multithreaded
	~pbgroup   = ParGroup.new;
	~recgroup  = ParGroup.new;
	s.sync;

	//--------------------------------------------------------------
	//_Synths

	SynthDef(\live,{ |amp=0.0, outbuf=10|
		var in, mix, env;
		in = [ SoundIn.ar(0), SoundIn.ar(1) ];
		mix = LPF.ar(in, 8000);
		mix = mix!4 * amp;
		Out.ar(outbuf, mix);
	}).add;


	SynthDef(\record,{ |bufnum, len, recLevel=1.0|
		var in, mix, env;
		in = SoundIn.ar(0, 1);
		mix = LPF.ar(in, 8000) * recLevel;
		RecordBuf.ar(mix, bufnum);

		env = Line.kr(dur:len, doneAction:2); // free
		SendReply.kr(TDelay.kr(Impulse.kr(0.1), len*0.999), '/endRec', 1.0);
	}).add;


	SynthDef(\playback,{ |amp=0.0, bufnum, len, pan=0, outbuf=10, trig=0|
		var head, loop, mix, env;
		loop = PlayBuf.ar(1, bufnum);
		env  = EnvGen.ar(Env.linen(len*0.0001, len*0.9998, len*0.0001), doneAction:2);
		mix  = loop * env * amp * 2.5;
		Out.ar(outbuf+pan, mix);

		SendReply.kr(TDelay.kr(Impulse.kr(0.1), len*0.999), '/endLayer', 1.0);
		SendReply.kr(trig, '/endLayer', 1.0);
		FreeSelf.kr(trig);
	}).add;


	SynthDef(\pitchshift,{ |inbuf, outbuf, len, ratio=1.0|
		var in, shift, env;
		in = In.ar(inbuf, 4);
		shift = PitchShift.ar(in, 0.01, ratio) * 3.5;
		Out.ar(outbuf, shift);

		env = Line.kr(dur:len, doneAction:2); // free
	}).add;


	SynthDef(\mixer,{ |amp=0.0, inbuf=10|
		var in, mix;
		in = Limiter.ar(In.ar(inbuf, 4));
		mix = MidEQ.ar(in, 15000, 0.7, 8) * amp;
		Out.ar(0, mix * 4.0);
	}).add;

	//--------------------------------------------------------------
	//_Routing

	s.sync;

	~mix  = Synth(\mixer, [\amp, 0.0, \inbuf,  ~mixbus], ~livegroup);
	~live = Synth(\live,  [\amp, 0.0, \outbuf, ~mixbus], ~livegroup);

	~synths = [
		[~live, \amp, "live"],
		[~mix,  \amp, "mixer"],
	];

	//--------------------------------------------------------------
	//_OSC

	// decrements layer count when layer ends
	~rEndLayer = OSCresponder(nil, '/endLayer', { |t, r, msg|
		~layers = ~layers - 1;
		{ ~numLayers.value = ~layers }.defer; // defer GUI process to main thread
	}).add;

	// stops recording when buffer is full
	~rEndRec = OSCresponder(nil, '/endRec', { |t, r, msg|
		"recording stopped: max length reached".postln;
		{ ~toggleStart.value = 0 }.defer; // defer GUI process to main thread
	}).add;

	//--------------------------------------------------------------
	//_Functions

	~timekeeper = SystemClock.sched(0.0,{ |time| ~time = time; 1; });

	~startrec = {
		Synth(\record, [
			\bufnum, ~recbuf.bufnum,
			\len, ~len
		],
		~recgroup
		);

		~bufisempty = false;

		// keep track of length
		Routine.run{
			var t0, ti;
			t0 = ~time;
			while({ ~isrecording },{
				ti = ~time;
				~rectime = ti - t0;
				(0.1).wait;
				~rectime = ti - t0;
			});
		};
	};

	~startdub = { | pan|
		var synth = Synth(\playback, [
			\bufnum, ~recbuf.bufnum,
			\len, ~len,
			\amp, 1.0,
			\pan, pan,
			\outbuf, ~mixbus
		],
		~pbgroup
		);

		Routine.run{
			var ti, t0;
			t0 = ti = ~time;
			while({ (ti - t0) < ~rectime },{
				ti = ~time;
				(0.1).wait;
			});
			~fadeout.(synth, amp:1.0, len:1);
			1.wait;
			synth.set(\trig, 1);
		};
	};

	~shiftdub = {
		Synth(\pitchshift, [
			\len, ~len,
			\ratio, ~pitch,
			\inbuf, ~procbus,
			\outbuf, ~mixbus
		],
		~pbgroup
		);

		Synth(\playback, [
			\bufnum, ~recbuf.bufnum,
			\len, ~len,
			\amp, 1.0,
			\pan, ~pan,
			\outbuf, ~procbus
		],
		~pbgroup
		);

		~procbus = ~procbus + 1; // increment to next available bus
	};

	~fadeout = { |synth, amp=1.0, len=10.0, res=50|
		Routine({
			res.do({ |i|
				synth.set(\amp, amp - (i/res*amp));
				(len/res).wait;
			});
		}).play;
	};

	//--------------------------------------------------------------
	//_GUI

	//_Window
	w = Window("canon", Rect(
		Window.screenBounds.width/2 - (~width/2),
		Window.screenBounds.height/2 - (~height/2),
		~width,
		~height
	))
	.alwaysOnTop_(true)
	.background_(Color(0.1, 0.1, 0.1));

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
			))
			.value_(~synths[i][2])
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
				val.value_(~round.(sliderSet[i].value));
			};

			valueSet[i] = val;
		});

		[sliderSet, valueSet, titleSet];
	};


	~round = { |n, dec = 100| (n * dec).round / dec }; // round for display


	// audio controls
	~toggleAudio = Button(w, Rect(10, 10, 180, 80))
	.states_([
		["audio off", Color.black, Color.red],
		["audio on", Color.black, Color.green]
	])
	.action_({ |state|
		if(state.value == 0, {
			~mix.set(\amp, 0.0);
			~faders[0][1].value_(0.0);
			~faders[1][1].value_(0.0);
		});
		if(state.value == 1, {
			~mix.set(\amp, 1.0);
			~faders[0][1].value_(1.0);
			~faders[1][1].value_(1.0);
		});
	});


	~closeButton = Button(w, Rect(10, 90, 180, 80))
	.states_([
		["quit", Color.white, Color.blue]
	])
	.action_({ |state|
		"".postln;
		"exiting canon audio\n".postln;
		s.quit;
		w.close;
		if(not(~debugIDE), {
			thisProcess.shutdown;
			0.exit;
		});
	});


	// looping controls
	~toggleLive = Button(w, Rect(10, 190, 180, 40))
	.states_([
		["live in: off", Color.black, Color.red],
		["live in: on ", Color.black, Color.green]
	])
	.action_({ |state|
		if(state.value == 0, {
			~live.set(\amp, 0.0);
			~faders[0][0].value_(0.0);
			~faders[1][0].value_(0.0);
		});
		if(state.value == 1, {
			~live.set(\amp, 1.0);
			~faders[0][0].value_(1.0);
			~faders[1][0].value_(1.0);
		});
	});


	~toggleStart = Button(w, Rect(10, 230, 180, 40))
	.states_([
		["recording: off", Color.black, Color.red],
		["recording: on ", Color.black, Color.green]
	])
	.action_({ |state|
		if(state.value == 0, {
			"".postln;
			"recording stopped".postln;
			~recgroup.freeAll;
			// ~fadeout.(~pbgroup, amp:1.0, len:10);
			~isrecording = false;
/*			Routine.run{
				10.wait;
				~pbgroup.freeAll;
				~layers = 0;
				{ ~numLayers.value = ~layers }.defer; // defer GUI process to main thread
				"layers cleared\n".postln;
			};*/
		});
		if(state.value == 1, {
			"recording started".postln;
			~startrec.();
			~isrecording = true;
			~recgroup.set(\recLevel, 1.0);
		});
	});


	~toggleDub0 = Button(w, Rect(10, 270, 90, 40))
	.states_([
		["dub: ch0", Color.black, Color.white]
	])
	.action_({ |state|
		if(not(~bufisempty), {
			~startdub.(pan:0);
			~layers = ~layers + 1;
			~numLayers.value = ~layers;
			("layer added: ch0").postln;
		},{
			"layer failed: nothing recorded".postln;
		});
	});


	~toggleDub1 = Button(w, Rect(100, 270, 90, 40))
	.states_([
		["dub: ch1", Color.black, Color.white]
	])
	.action_({ |state|
		if(not(~bufisempty), {
			~startdub.(pan:1);
			~layers = ~layers + 1;
			~numLayers.value = ~layers;
			("layer added: ch1").postln;
		},{
			"layer failed: nothing recorded".postln;
		});
	});


	~toggleDub2 = Button(w, Rect(10, 310, 90, 40))
	.states_([
		["dub: ch2", Color.black, Color.white]
	])
	.action_({ |state|
		if(not(~bufisempty), {
			~startdub.(pan:2);
			~layers = ~layers + 1;
			~numLayers.value = ~layers;
			("layer added: ch2").postln;
		},{
			"layer failed: nothing recorded".postln;
		});
	});


	~toggleDub3 = Button(w, Rect(100, 310, 90, 40))
	.states_([
		["dub: ch3", Color.black, Color.white]
	])
	.action_({ |state|
		if(not(~bufisempty), {
			~startdub.(pan:3);
			~layers = ~layers + 1;
			~numLayers.value = ~layers;
			("layer added: ch3").postln;
		},{
			"layer failed: nothing recorded".postln;
		});
	});


	// layer indicator
	~textLayers = StaticText.new(w, Rect(10, 370, 140, 40))
	.string_("active layers: ")
	.stringColor_(Color.white)
	.align_(\center);

	~numLayers = TextField(w, Rect(150, 370, 40, 40)).value_(0).align_(\center);


	// info
	~title = StaticText.new(w, Rect(200, 0, ~width/2, 80))
	.string_("canon")
	.stringColor_(Color.white)
	.font_(Font("Helvetica", 24))
	.align_(\center);

	~desc = StaticText.new(w, Rect(200, 80, ~width/2, 80))
	.string_("infinite looper\n4ch edition")
	.stringColor_(Color.white)
	.font_(Font("Helvetica", 18))
	.align_(\center);


	// amplitude control
	~faders = ~sliderGen.value(~width/2, 200, 200, 170, ~synths.size);
	~toggles = [~toggleLive, ~toggleAudio];

	for(0, ~synths.size-1, { |i|
		~faders[0][i].action = { |obj|
			var value = obj.value;
			~synths[i][0].set(~synths[i][1], value);
			~faders[1][i].value_(~round.(value));
			if(obj.value > 0, { ~toggles[i].value_(1) }, { ~toggles[i].value_(0) });
		};
	});


	// reset
	~reset = Button(w, Rect(210, 370, 180, 40))
	.states_([
		["reset", Color.white, Color.black]
	])
	.action_({ |state|
		~recgroup.freeAll;
		~pbgroup.freeAll;
		~toggleStart.value_(0);
		~isrecording = false;
		~layers = 0;
		~numLayers.value = ~layers;
		~bufisempty = true;
		~rectime = 0;
		"".postln;
		"reset: recording stopped, layers cleared\n".postln;
	});


	// server GUI items
	if(~debugServer, {
		s.meter;
		s.scope;
		s.plotTree;
	});


	// display gui
	if( (~displayGUI), { w.front });

	//--------------------------------------------------------------
	//_Status

	"".postln;
	"canon audio is now running\n".postln;
	("sampling rate: \t\t" + o.sampleRate).postln;
	("allocated RAM: \t\t" + (o.memSize/1000000) + "GB").postln;
	("# input channels: \t" + o.numInputBusChannels).postln;
	("# output channels: \t" + o.numOutputBusChannels + "\n").postln;
});
)