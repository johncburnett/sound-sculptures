(
/*
* bathysphere.scd
*
* john burnett (C) 2017
*/

//--------------------------------------------------------------
// init

// server
s = Server.local;
o = s.options;
o.memSize_(2e6); // 2GB of RAM
o.numOutputBusChannels = 2;

// globals
~len = 5 * 60;
~len = ~len - 30 / 2;
~n = 1000;

~dur = 5;
~width = 0.5;
~rate = 1;
~pscale = 2;

// ~runRoutine = True;

// recording
~doRecord = True;
~recName = "renders/gauss.aiff";

s.waitForBoot({

	//--------------------------------------------------------------
	// buffers

	~cwd = thisProcess.nowExecutingPath.dirname +/+ "samples";
	// ~b1 = Buffer.read(s, ~cwd +/+ "eclipse.wav");
	~b2 = Buffer.readChannel(s, ~cwd +/+ "gauss-drone.aiff", channels:1);

	s.sync;

	// ~nom = ~cwd +/+ "eclipse.wav";
	// ~r1 = RedBuffer(s, ~nom, 0.0, 5);

	~red = RedGrain(s);
	~red.buf_(~b2)
	.rate_(~rate)
	.pos_(0)
	.dur_(~dur)
	.pan_(0)
	.delta_( { 1.001.rrand(1.01) } )
	.start;

	s.sync;

	//--------------------------------------------------------------
	// gui

	w = Window("descend", Rect(20, 400, 440, 50));
	w.front;
	w.view.decorator = FlowLayout(w.view.bounds);
	w.view.decorator.gap=2@2;
	w.onClose_({ ~descend.stop.reset; Server.killAll; });

	~rs1 = ControlSpec(1.0, 0.01, \lin, 0.0, 1.0);
	~rs2 = ControlSpec(1.0, ~pscale,  \lin, 0.0, 1.0);
	~ds1 = ControlSpec(2.5, 0.0004, \exp, 0.0, 1.0);
	~ds2 = ControlSpec(2.5, 0.004,  \exp, 0.0, 1.0);

	w.view.decorator.nextLine;
	~rateControl = EZSlider(w, 440 @ 40, "Bathysphere ", ControlSpec(0.0, 1.0, \lin, 0.0, 1.0), unitWidth:30)
	.action_({ |ez|
		var val, di, d0, ri, r0;
		val = ez.value;
		r0 = ~rs1.map(val) * ~rate;
		ri = ~rs2.map(val) * ~rate;
		~red.rate_( {(r0).rrand(ri)} );
		~red.pos_( {(val*~width).rand2} );
		~red.dur_( {(~dur-(val*~dur*0.8)).rrand(~dur-(val*~dur*0.82))} );
		~red.pan_( {(0 - val).rrand(val)} );
		d0 = ~ds1.map(val);
		di = ~ds2.map(val);
		~red.delta_( {(d0).rrand(di)} );
	})
	.value_(0);

	//--------------------------------------------------------------
	// processes

	~descend = Routine {
		var step = ~len / ~n;
		10.wait;
		for(0, ~n, { |i|
			var interp = (1.0 / ~n) * i;
			{ ~rateControl.valueAction_(interp) }.defer;
			step.wait;
		});
		10.wait;
		for(~n, 0, { |i|
			var interp = (1.0 / ~n) * i;
			{ ~rateControl.valueAction_(interp) }.defer;
			step.wait;
		});
		10.wait;
	}.play;

	//--------------------------------------------------------------
	// main

	if(~doRecord == True, {
		var path = thisProcess.nowExecutingPath.dirname +/+ ~recName;
		s.prepareForRecord(path);
		s.sync;
		fork{
			s.record;
			(~len*2 + 30).wait;
			s.stopRecording;
			{ w.close() }.defer;
		};
	});
});
)