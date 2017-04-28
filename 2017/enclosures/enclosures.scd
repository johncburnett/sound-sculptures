(
/*
* enclosures.scd
*
* enclosures
* Copyright (C) 2017 - John Burnett
* <http://www.johncburnett.com/>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//--------------------------------------------------------------
//_Init

//_Routing
~num_inputs = 2; // 1 or 2

//_Server
s = Server.local;
o = s.options;
o.memSize_(5e5); // 500MB of RAM
o.numInputBusChannels  = 0; // 2 in
o.numOutputBusChannels = 2; // 2 out

//_Globals
~displayGUI = true;

//_Synthesis globals
~dur = 20.0;  // duration of each swell
~space = 1;   // space between swell onsets

~x = 0.5;
~y = 0.5;
~state = 0!9;
~isRunning = true;

//_Debugging
~debugIDE = true;
~debugServer = false;

// OSC
// ~ip = "127.0.0.1";
~port = 7771;
thisProcess.openUDPPort(~port);
// ~osc = NetAddr.new(~ip, ~port);

s.waitForBoot({

	//--------------------------------------------------------------
	//_Buffers

	~cwd = thisProcess.nowExecutingPath.dirname +/+ "samples";
/*	~b1 = Buffer.read(s, ~cwd +/+ "hallway.wav");
	~b2 = Buffer.read(s, ~cwd +/+ "hallway.wav");
	~b3 = Buffer.read(s, ~cwd +/+ "hallway.wav");*/

/*	~b1 = Buffer.read(s, ~cwd +/+ "water.wav");
	~b2 = Buffer.read(s, ~cwd +/+ "park.wav");
	~b3 = Buffer.read(s, ~cwd +/+ "alm.wav");*/

	~b1 = Buffer.read(s, ~cwd +/+ "drone.wav");
	~b2 = Buffer.read(s, ~cwd +/+ "scraping.wav");
	~b3 = Buffer.read(s, ~cwd +/+ "alm.wav");

	s.sync; /* wait for server */

	//--------------------------------------------------------------
	//_Live Input

	// stereo input
	SynthDef(\live_in, { |amp=0.0, send=0.0|
		var in, mix, chain, onsets;
		in = [SoundIn.ar(0), SoundIn.ar(1)];
		mix = in * amp;
		Out.ar(10, in * send); 	// to processing
		Out.ar(20, mix);	    // to mixer
	}).add;

	//--------------------------------------------------------------
	//_Samplers

	SynthDef(\sampler_1ch, { |bufnum=0, rate=1.0, pan=0, amp=0.0, send=0.0|
		var in, mix;
		in = PlayBuf.ar(1, bufnum, rate, doneAction:2);
		mix = Pan2.ar(in, pan) * amp;
		Out.ar(10, in * send); 	// to processing
		Out.ar(20, mix);	    // to mixer
	}).add;


	SynthDef(\sampler_2ch, { |bufnum=0, rate=1.0, amp=0.0, send=0.0|
		var in, mix;
		in = PlayBuf.ar(2, bufnum, rate, doneAction:2);
		mix = in * amp;
		Out.ar(10, in * send); 	// to processing
		Out.ar(20, mix);	    // to mixer
	}).add;

	SynthDef(\looper_1ch, { |bufnum=0, rate=1.0, pan=0, amp=0.0, send=0.0|
		var in, mix;
		in = PlayBuf.ar(1, bufnum, rate, loop:1);
		mix = Pan2.ar(in, pan) * amp;
		Out.ar(10, in * send); 	// to processing
		Out.ar(20, mix);	    // to mixer
	}).add;


	SynthDef(\looper_2ch, { |bufnum=0, rate=1.0, amp=0.0, send=0.0, pos=0|
		var in, mix;
		in = PlayBuf.ar(2, bufnum, rate, loop:1, startPos:pos);
		mix = in * amp;
		Out.ar(10, in * send); 	// to processing
		Out.ar(20, mix);	    // to mixer
	}).add;

	SynthDef(\looper_2ch_alt, { |bufnum=0, rate=1.0, amp=0.0, send=0.0, dur=8, pos=0|
		var in, mix, env;
		in = PlayBuf.ar(2, bufnum, rate, loop:1, startPos:pos);
		mix = in * amp * (SinOsc.kr(0.0625) + 1 * 0.5);
		mix = Pan2.ar(mix, SinOsc.kr(0.0625*0.5));
		env = EnvGen.ar(Env.linen(dur*0.1, dur*0.8, dur*0.1), LFNoise1.kr(0.0025)*10);
		Out.ar(15, in * send * env); 	// to processing
		Out.ar(20, mix);	    // to mixer
	}).add;

	//--------------------------------------------------------------
	//_Processing

	SynthDef(\chorus, { |amp=0.0, predelay=0.08, speed=0.05, depth=0.1, ph_diff=0.5|
		var in1, in2, mod1, mod2, sig1, sig2, mix, numDelays = 12;
		in1 = In.ar(10, 1) * numDelays.reciprocal;
		in2 = In.ar(11, 1) * numDelays.reciprocal;

		mod1 = Array.fill(numDelays, { |i|
			LFPar.kr(speed * rrand(0.94, 1.06), ph_diff * i, depth, predelay);
		});
		mod2 = Array.fill(numDelays, { |i|
			LFPar.kr(speed * rrand(0.94, 1.06), ph_diff * i, depth, predelay);
		});
		sig1 = DelayC.ar(in1, 0.5, mod1);
		sig2 = DelayC.ar(in2, 0.5, mod2);

		mix = [sig1, sig2];
		Out.ar(20, mix);
	}).add;


	SynthDef(\collage,{ |amp=0.8|
		var in, d1, d2, p1, p2, p3, p4, pan, mix;

		in = In.ar(15, 2);

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
		d1 = Pan2.ar(d1, pan);
		d2 = Pan2.ar(d2, -1.0*pan);
		mix = [d1, d2] * amp;
		Out.ar(20, mix);
	}).add;


	SynthDef(\collage_alt,{ |amp=0.8|
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
		d1 = Pan2.ar(d1, pan);
		d2 = Pan2.ar(d2, -1.0*pan);
		mix = [d1, d2] * amp;
		Out.ar(20, mix);
	}).add;


	SynthDef(\melt,{ |amp=0.5, mod_scale=1.0, dur=6, gate=0|
		var in, d1, d2, pan, mix, env;

		in = In.ar(10, 2);

		d1 = Array.fill(6, {
			BPF.ar(
				CombC.ar(
					in[0],
					15.0,
					LinLin.kr(
						SinOsc.kr(rrand(1/16, 1/32) * mod_scale),
						-1.0,
						1.0,
						rrand(4.0, 6.0) * mod_scale,
						rrand(8.0, 12.0) * mod_scale ),
					10) * mod_scale,
				rrand(200, 10000),
				0.3)
		} ).sum / 2;

		d2 = Array.fill(6, {
			BPF.ar(
				CombC.ar(
					in[1],
					15.0,
					LinLin.kr(
						SinOsc.kr(rrand(1/16, 1/32) * mod_scale),
						-1.0,
						1.0,
						rrand(4.0, 6.0) * mod_scale,
						rrand(8.0, 12.0) * mod_scale ),
					10) * mod_scale,
				rrand(200, 10000),
				0.3)
		} ).sum / 2;

		pan = LinLin.kr(SinOsc.kr(0.125), -1.0, 1.0, -0.6, 0.6);
		d1 = Pan2.ar(d1, pan);
		d2 = Pan2.ar(d2, -1.0*pan);
		mix = ([d1, d2]*4) * amp;

		env = (SinOsc.kr(0.0625 * 0.6) + 1 * 0.5);


		Out.ar(20, mix * env);	// to mixer
	}).add;

	//--------------------------------------------------------------
	//_Synthesis

	SynthDef(\phaseMod, { |amp=0.0, carrier=1000.0, mod=400.0, pan=0.0|
		var phaseEnv, ampEnv, pmOsc, mix;
		phaseEnv = Line.ar(0.0, 10.0.rand, 8);
		ampEnv   = EnvGen.ar(Env.linen(~dur*0.25, ~dur*0.5, ~dur*0.25), doneAction:2);
		pmOsc    = PMOsc.ar(carrier, mod, phaseEnv, mul:0.1);
		mix      = Pan2.ar(pmOsc * ampEnv, pan) * amp;
		Out.ar(20, mix);
	}).add;

	//--------------------------------------------------------------
	//_Mixer

	SynthDef(\mixer,{ |amp=0.0|
		var in, mix;
		in = In.ar(20, 2);
		mix = Limiter.ar(MidEQ.ar(in, 15000, 0.7, 8)) * amp;
		Out.ar(0, mix);

		SendPeakRMS.kr(mix, 20, 3, '/rms');
	}).add;

	/**************************************************************************************************/
	//_Routing

	s.sync; /* wait for server */

	// mixer
	~mix = Synth(\mixer, [\amp, 0.0]);

	// processing
	~collage   = Synth(\collage,   [\amp, 0.4]);
	~collage_alt = Synth(\collage,   [\amp, 0.8]);
	~melting   = Synth(\melt,      [\amp, 0.00]);

	// samplers
	~drone    = Synth(\looper_2ch, [\bufnum, ~b1.bufnum]);
	~scraping1 = Synth(\looper_2ch_alt, [\bufnum, ~b2.bufnum, \rate, 0.9]);
	~scraping2 = Synth(\looper_2ch_alt, [\bufnum, ~b2.bufnum, \rate, 0.9, \pos, ~b2.numFrames/2]);
	~alm      = Synth(\looper_2ch, [\bufnum, ~b3.bufnum, \send, 1.0]);

	// live input
	~live = Synth(\live_in, [\amp, 0.0, \send, 0.0]);

	// synth arrays
	~gens = [~drone, ~scraping, ~alm, ~live]; // generators
	~proc = [~collage, ~melting];         // processing

	// group for pm instances
	~pm = Group.new;

	// phase modulation
	Routine.run({
		while({~isRunning}, {
			Synth(\phaseMod, [
				\carrier,	2000.0.rand,
				\mod, 		800.rand,
				\pan, 		1.0.rand2,
				\amp, 		0.3
			], ~pm);
			~space.wait;
		});
	});

	//--------------------------------------------------------------
	//_Utilities

	~p0 = [0.6, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.8]; // world 0
	~p1 = [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.8, 0.0]; // world 1
	~p2 = [0.0, 0.0, 0.0, 0.0, 0.2, 0.8, 0.0, 0.0, 0.1]; // world 2
	~p3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0]; // world 3

	~lerp = { |a0, a1, x|
		var s0, v0;
		s0 = ControlSpec(a0, a1, \lin, 0.00001);
		v0 = s0.map(x);
		v0;
	};

	~lerpV = { |v0, v1, x|
		var lerped = Array.newClear(v0.size);
		for(0, v0.size-1, { |i|
			lerped[i] = ~lerp.(v0[i], v1[i], x);
		});
		lerped;
	};

	// BL, BR, TL, TR
	~lerpQuad = { |v0, v1, v2, v3, x, y|
		var a0, a1, a2, a3;
		a0 = (1.0-x) * (1.0-y) * v0;
		a1 = x * (1.0-y) * v1;
		a2 = (1.0-x) * y * v2;
		a3 = x * y * v3;
		a0 + a1 + a2 + a3;
	};

	// BL, BR, TL, TR
	~lerpQuadV = { |v0, v1, v2, v3, x, y|
		var lerped;
		lerped = Array.newClear(v0.size);
		for(0, v0.size-1, { |i|
			var a0, a1, a2, a3, val;
			a0 = (1.0-x) * (1.0-y) * v0[i];
			a1 = x * (1.0-y) * v1[i];
			a2 = (1.0-x) * y * v2[i];
			a3 = x * y * v3[i];
			val = a0 + a1 + a2 + a3;
			lerped[i] = val;
		});
		lerped;
	};

	~dist = { |x0, y0, x1=0.5, y1=0.5| sqrt(((x1-x0)**2) + ((y1-y0)**2)) };

	/**************************************************************************************************/
	//_OSC

	~rX = OSCresponder(nil, '/x', { |t, r, msg| ~x = msg[1]; }).add;
	~rY = OSCresponder(nil, '/y', { |t, r, msg| ~y = msg[1]; }).add;
	~rXY = OSCresponder(nil, '/xy', { |t, r, msg| ~x = msg[1]; ~y = msg[2]; }).add;
	~rVol = OSCresponder(nil, '/volume', { |t, r, msg| ~mix.set(\amp, msg[1]); }).add;

	// OSCFunc({ |msg, time| ~osc.sendMsg('/rms', msg[4]); },'/rms', s.addr);

/*	~pmRoutine = Routine.run({
		while({~isRunning}, {
			~state = ~lerpQuadV.(~p0, ~p1, ~p2, ~p3, ~x, ~y);
			~park.set(\amp, ~state[0], \send, ~state[1]);
			~alm.set(\amp, ~state[2], \send, ~state[3]);
			~water.set(\amp, ~state[4], \send, ~state[5]);
			~pm.set(\amp, ~state[6]);
			~melting.set(\amp, ~state[7]);
			~collage.set(\amp, ~state[8]);
			~state.postln;
			0.01.wait;
	    });
	});*/

	~pmRoutine = Routine.run({
		while({~isRunning}, {
			var dist0, dist1;
			dist0 = ~dist.(~x, ~y);
			dist1 = ~dist.(~x, ~y, 0.5, 0.125);
			if( dist0 >= 0.375, {
				~pm.set(\amp, (dist0 - 0.375) * 4);
				~drone.set(\amp, 0);
				~scraping1.set(\amp, 0);
				~scraping1.set(\send, 0);
				~scraping2.set(\amp, 0);
				~scraping2.set(\send, 0);
			},{
				~pm.set(\amp, 0);
				~drone.set(\amp, (1 - dist0 - 0.625 * 4));
				~scraping1.set(\amp, (1 - dist0 - 0.625 * 2));
				~scraping1.set(\send, (1 - dist0 - 0.625 * 2));
				~scraping2.set(\amp, (1 - dist0 - 0.625 * 2));
				~scraping2.set(\send, (1 - dist0 - 0.625 * 2));

			});

			0.01.wait;
		});
	});

/*	fork{
		while({~isRunning}, {
			if(~dist.(~x, ~y) < 0.375, {
				~melting.set(\amp, 0.7, \gate, 1);
			});
			(10.rand + 5).wait;
			~melting.set(\gate, -1);
		});
	};*/


	/**************************************************************************************************/
	//_GUI

	// window config
	w = Window("multiplicity", Rect(
		Window.screenBounds.width - 200,
		Window.screenBounds.height,
		200,
		200
	))
	.alwaysOnTop_(true)
	.background_(Color(0.1, 0.1, 0.1));

	// audio controls
	~audioToggle = Button(w, Rect(10, 10, 180, 80))
	.states_([
		["audio off", Color.black, Color.red],
		["audio on", Color.black, Color.green]
	])
	.action_({ |state|
		if(state.value == 0, { ~mix.set(\amp, 0.0) });
		if(state.value == 1, { ~mix.set(\amp, 1.0) });
	});

	~closeButton = Button(w, Rect(10, 110, 180, 80))
	.states_([
		["quit", Color.white, Color.blue]
	])
	.action_({ |state|
		~isRunning = false;
		~pmRoutine.stop.reset;
		s.quit;
		w.close;
		if(not(~debugIDE), {
			thisProcess.shutdown;
			0.exit;
		});
	});

	// server GUI items
	if(~debugServer, {
		s.meter;
		s.scope;
		s.plotTree;
	});

	// display gui
	if( (~displayGUI), { w.front });
});
)
