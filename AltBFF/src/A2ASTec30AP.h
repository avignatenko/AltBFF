
#include <spdlog/spdlog.h>

#include <optional>

class A2AStec30AP
{
public:

	A2AStec30AP()
	{

	}

	// own state

	void enableRollAxis(bool enable) { rollEnabled_ = enable; }
	void enablePitchAxis(bool enable)
	{
		if (!pitchEnabled_ && enable)
		{
			//targetPressure_ = simPressure_;
			//////////// !!!!!!!!!!!!!!!!!!!!!!!!!!!1
			targetPressure_ = simElevator_; // for test
			//////////// !!!!!!!!!!!!!!!!!!!!!!!!!!!1

			spdlog::trace("AP pitch enabled with target: {}", targetPressure_);
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

	// Pa
	void setAirPressure(double pressure)
	{
		simPressure_ = pressure;
		spdlog::trace("Air pressture set to AP: {}", simPressure_);
	}

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

	double targetPressure_ = 0.0;

	double simAileron_ = 0.0;
	double simElevator_ = 0.0;
	double simPressure_ = 0.0;

	double aileronOut_ = 0.0;
	double elevatorOut_ = 0.0;


};