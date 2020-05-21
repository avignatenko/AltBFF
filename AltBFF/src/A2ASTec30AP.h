#pragma once

#include "PID.h"

#include <spdlog/spdlog.h>
#include <optional>

class A2AStec30AP
{
public:

	struct Settings
	{
		struct PID
		{
			double p;
			double i;
			double d;
		};

		PID rollPID = { 0.0, 0.0, 0.0 };
		PID pitchPID = { 0.0, 0.0, 0.0 };

	};

	A2AStec30AP(const Settings& settings): settings_(settings)
	{

	}

	// own state
	void setSettings(const Settings& settings) { settings_ = settings; }

	void enableRollAxis(bool enable) { rollEnabled_ = enable; }
	void enablePitchAxis(bool enable)
	{
		if (!pitchEnabled_ && enable)
		{
			pitchController_ = PIDController(settings_.pitchPID.p, settings_.pitchPID.i, settings_.pitchPID.d, -100, 100, 1000 / 30);
			pitchController_.value().setSetPoint(simPitch_);

			elevatorOut_ = simElevator_;
			spdlog::trace("AP pitch enabled with target: {}", simPitch_);
		}

		pitchEnabled_ = enable;
	}

	// sim vars
	void setSimAileron(double aileron)
	{
		simAileron_ = aileron;
		spdlog::trace("Aileron set to AP: {}", simAileron_);

	}
	void setSimElevator(double elevator)
	{
		simElevator_ = elevator;
		spdlog::trace("Elevator set to AP: {}", simElevator_);
	}

	// rad
	void setSimPitch(double pitch) { simPitch_ = pitch; }

	// Pa
	void setAirPressure(double pressure)
	{
		simPressure_ = pressure * 1e-5; // convert to bar
		spdlog::trace("Air pressture set to AP: {}", simPressure_);
	}

	// model vars
	
	//[-100, 100]
	void setTotalAxisCLForceElevator(double force) { clForceElevator_ = force; }
	void setTotalAxisCLForceAileron(double force) { clForceAileron_ = force; }
	
	void process();

	// [-100, 100]
	std::optional<double> getCLAileron();

	// [-100, 100]
	std::optional<double> getCLElevator();


	std::optional<double> getSimAileron();
	std::optional<double> getSimElevator();

private:

	bool rollEnabled_ = false;
	bool pitchEnabled_ = false;

	std::optional<PIDController>  pitchController_;
	

	double simAileron_ = 0.0;
	double simElevator_ = 0.0;
	double simPressure_ = 0.0;
	double simPitch_ = 0.0;

	double clForceElevator_ = 0.0;
	double clForceAileron_ = 0.0;

	double aileronOut_ = 0.0;
	double elevatorOut_ = 0.0;

	

	Settings settings_;


};