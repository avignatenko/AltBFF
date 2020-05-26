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

		// pitch
		int pitchmode = 0;

		PID elevatorPID = { 0.0, 0.0, 0.0 };

		PID pitchRatePID = { 0.0, 0.0, 0.0 };
		double pitchRate = 0;

		PID pitchPID = { 0.0, 0.0, 0.0 };
		double pitchMax = 0;

		PID fpmPID = { 0.0, 0.0, 0.0 };
		double fpmMax = 0;

		bool doStepResponse = false;
		std::string stepResponseInputFile;

		// 0..100%
		double pitchWarningCLForce = 80;
		// 0..100%
		double pitchStartDegradeCLForce = 90.0;
		// 0..100%
		double pitchMaxCLForce = 100.0;
	};

	A2AStec30AP(const Settings& settings) 
	{
		setSettings(settings);
	}

	// own state
	void setSettings(const Settings& settings)
	{
		settings_ = settings; 
		if (settings_.doStepResponse)
			readStepResponseInput();
	}

	void enableRollAxis(bool enable);
	void enablePitchAxis(bool enable);

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
	void setSimPitch(double pitch)
	{
		simPitch_ = pitch;
		spdlog::trace("Pitch set to AP: {}", simPitch_);
	}

	void setSimPitchRate(double pitchRate)
	{
		simPitchRate_ = pitchRate;
		spdlog::trace("Pitch rate set to AP: {}", simPitchRate_);
	}

	void setSimFpm(double fpm)
	{
		simFpm_ = fpm;
		spdlog::trace("FPM set to AP: {}", simFpm_);
	}

	// Pa
	void setAirPressure(double pressure)
	{
		simPressure_ = pressure * 1e-5; // convert to bar
		spdlog::trace("Air pressure set to AP: {}", simPressure_);
	}

	// model vars

	//[-100, 100]
	void setTotalAxisCLForceElevator(double force)
	{
		clForceElevator_ = force;
		spdlog::trace("CL Force elevator set to AP: {}", clForceElevator_);
	}

	void setTotalAxisCLForceAileron(double force)
	{
		clForceAileron_ = force;
		spdlog::trace("CL Force aileron set to AP: {}", clForceAileron_);
	}

	void process();

	// [-100, 100]
	std::optional<double> getCLAileron();

	// [-100, 100]
	std::optional<double> getCLElevator();


	std::optional<double> getSimAileron();
	std::optional<double> getSimElevator();

	struct TrimNeededWarning
	{
		enum PitchDirection
		{
			NA,
			Up,
			Down
		};

		double forceDelta = 0.0;
		PitchDirection pitchDirection = NA;
		int warningLevel = 0; // 0 - no warning, 1 - warning, 2 - level 2 warning
	};

	TrimNeededWarning getTrimNeededWarning();

private:

	bool enabled()
	{
		return rollEnabled_ || pitchEnabled_;
	}

	void computeStepResponseInput(PIDController& controller);
	void readStepResponseInput();
	void writeStepResponse();

private:

	bool rollEnabled_ = false;
	bool pitchEnabled_ = false;

	struct PitchController
	{
		enum class Mode
		{
			Pitch = 0,
			FPM,
			Alt
		};
		Mode mode = Mode::Alt;
		PIDController fpmController;
		PIDController pitchController;
		PIDController pitchRateController;
		PIDController elevatorController;
	};

	std::optional<PitchController>  pitchController_;
	

	double simAileron_ = 0.0;
	double simElevator_ = 0.0;
	double simPressure_ = 0.0;
	double simPitch_ = 0.0;
	double simPitchRate_ = 0.0;
	double simFpm_ = 0.0;

	double clForceElevator_ = 0.0;
	double clForceAileron_ = 0.0;

	double aileronOut_ = 0.0;
	double elevatorOut_ = 0.0;

	// for pid tuning
	bool stepResponseInProgress = false;
	unsigned long currentInputSample_ = 0;
	std::vector<std::pair<long, double>> stepResponseInput_;
	std::vector<std::array<double, 3>> stepResponseOutput_;
	unsigned long timeSamplesPitch_ = 0;
	

	Settings settings_;


};