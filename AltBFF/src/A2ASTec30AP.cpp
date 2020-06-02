#include "A2ASTec30AP.h"

#include "Sim.h"
#include "Model.h"
#include <BFFCLAPI/CLStructures.h>
#include <CSV/CSV.hpp>
#include <Utils/Common.h>
#include <fmt/ranges.h>

namespace
{
const double kLoopTimeMs = 1000.0 / 30;
}

void A2AStec30AP::enableRollAxis(bool enable)
{
    if (rollEnabled_ == enable) return;
    rollEnabled_ = enable;
}

void A2AStec30AP::enablePitchAxis(bool enable)
{
    if (pitchEnabled_ == enable) return;

    if (enable)
    {
      
        pitchController_ = PitchController
        {
            static_cast<PitchController::Mode>(settings_.pitchmode),
            // fpm controller
            PIDController(settings_.fpmPID.p, settings_.fpmPID.i, settings_.fpmPID.d,
                          settings_.fpmDuMax, -settings_.fpmMax, settings_.fpmMax, kLoopTimeMs, simPressureAltitude_, simFpm_),
            // pitch controller
            PIDController(settings_.pitchPID.p, settings_.pitchPID.i, settings_.pitchPID.d,
                          settings_.pitchDuMax, -settings_.pitchMax, settings_.pitchMax, kLoopTimeMs, simFpm_, simPitch_),
            // pitch rate controller
      //      PIDController(settings_.pitchRatePID.p, settings_.pitchRatePID.i, settings_.pitchRatePID.d,
      //                   settings_.pitchRateDuMax, -settings_.pitchRate, settings_.pitchRate, kLoopTimeMs, simPitch_, simPitchRate_),
            // elevator controller
            PIDController(settings_.elevatorPID.p, settings_.elevatorPID.i, settings_.elevatorPID.d,
                        settings_.elevatorDuMax, -100, 100 , kLoopTimeMs, simPitch_, simElevator_)
        };

        spdlog::info("AP Pitch mode: {}", settings_.pitchmode);

        switch (pitchController_.value().mode)
        {
        case PitchController::Mode::Pitch:
            pitchController_.value().elevatorController.setSetPoint(simPitch_);
            spdlog::info("AP pitch enabled with target pitch: {}", simPitch_);
            break;
        case PitchController::Mode::FPM:
            pitchController_.value().pitchController.setSetPoint(500.0);
            spdlog::info("AP pitch enabled with target fpm: {}", 500.0);
            break;
        case PitchController::Mode::Alt:
            pitchController_.value().fpmController.setSetPoint(simPressureAltitude_);
            spdlog::info("AP pitch enabled with target pressure altitude: {}", simPressureAltitude_);
            break;
        }

        if (settings_.doStepResponse)
        {
            stepResponseInProgress = true;
           
            spdlog::info("Started impulse response test");
        }
    }
    else
    {
        stepResponseInProgress = false;
        spdlog::info("AP pitch disabled");
    }

    pitchEnabled_ = enable;
    
    timeSamplesPitch_ = 0;
    currentInputSample_ = 0;
    stepResponseOutput_.clear();
}

void A2AStec30AP::process()
{
    if (!enabled()) return;

    // aileron
    if (rollEnabled_)
    {
        aileronOut_ = simAileron_;
        spdlog::trace("AP aileron calculated: {}", aileronOut_);
    }

    // elevator
    if (pitchEnabled_)
    {
        // mode Alt = 2
        // pressure -> fpm (absolute!)
        // error = ? set fmp = 0
        if (settings_.pitchmode == 2)
        {
            PIDController& fpmController = pitchController_.value().fpmController;
            fpmController.setInput(simPressureAltitude_);


            if (stepResponseInProgress && settings_.pitchmode == 2)
                computeStepResponseInput(fpmController);
            else
               fpmController.compute();

            spdlog::trace("fpm pid: {}", fpmController.dumpInternals());
            spdlog::trace("total output fpm: {}", fpmController.getOutput());
        }

        // model FPM = 1
        // pitch controller: fpm -> pitch offset
        // error = 0 ? keep pitch as is
        if (settings_.pitchmode >= 1)
        {

            PIDController& pitchController = pitchController_.value().pitchController;
            if (settings_.pitchmode > 1)
                pitchController.setSetPoint(pitchController_.value().fpmController.getOutput());
            pitchController.setInput(simFpm_);
  
            if (stepResponseInProgress && settings_.pitchmode == 1)
                computeStepResponseInput(pitchController);
            else
              pitchController.compute();

            spdlog::trace("pitch pid: {}", pitchController.dumpInternals());
            spdlog::trace("total output pitch: {}", pitchController.getOutput());
        }

      
        // elevator controller: pitch rate -> elevator offset
        // error = 0 ? keep elevator as is
        PIDController& elevatorController = pitchController_.value().elevatorController;
        if (settings_.pitchmode > 0)
            elevatorController.setSetPoint(pitchController_.value().pitchController.getOutput());
        elevatorController.setInput(simPitch_);
  
        if (stepResponseInProgress && settings_.pitchmode == 0)
            computeStepResponseInput(elevatorController);
        else
            elevatorController.compute();

        spdlog::trace("elevator pid: {}", elevatorController.dumpInternals());
        spdlog::trace("total output elevator: {}", elevatorController.getOutput());

        /*
        // check for excessive forces

        double forceError = std::abs(clForceElevator_) - settings_.pitchStartDegradeCLForce;

        double forceShift = 0.0;
        if (forceError > 0)
        {
            forceShift = settings_.pitchMaxCLForce * std::copysign(forceError, clForceElevator_);
            spdlog::trace("AP force coeff: {}, AP Force Shift: {}", forceError, forceShift);
        }*/

        // finally send back
        elevatorOut_ = pitchController_.value().elevatorController.getOutput();// + forceShift;

        timeSamplesPitch_++;

        spdlog::debug("AP elevator calculated: {}", elevatorOut_);
    }

}

std::optional<double> A2AStec30AP::getCLAileron()
{
    return rollEnabled_ ? aileronOut_ : std::optional<double>();
}

std::optional<double> A2AStec30AP::getCLElevator()
{
    return pitchEnabled_ ? elevatorOut_ : std::optional<double>();
}

std::optional<double> A2AStec30AP::getSimAileron()
{
    return std::optional<double>(); // we follow and not write ailerons
}

std::optional<double> A2AStec30AP::getSimElevator()
{
    return getCLElevator();
}

A2AStec30AP::TrimNeededWarning A2AStec30AP::getTrimNeededWarning()
{
    TrimNeededWarning warning;
    if (std::abs(clForceElevator_) < settings_.pitchWarningCLForce)
    {
        warning.pitchDirection = TrimNeededWarning::NA;
        warning.warningLevel = 0;
        return warning;
    }

    warning.pitchDirection = clForceElevator_ > 0 ? TrimNeededWarning::Up : TrimNeededWarning::Down;
    warning.warningLevel = 1; // for now (todo)
    warning.forceDelta = std::abs(clForceElevator_) - settings_.pitchWarningCLForce;
    return warning;
}

void A2AStec30AP::computeStepResponseInput(PIDController& controller)
{
    long curTimeMs = timeSamplesPitch_ * kLoopTimeMs;
    int i = currentInputSample_;
    for ( ; i < stepResponseInput_.size(); ++i)
    {
        if (stepResponseInput_[i].first > curTimeMs)
            break;
    }
    currentInputSample_ = std::max(0, i - 1);


    // 7 -- iterm
    controller.setSimulatedOutput(
        controller.dumpInternals()[0] * controller.dumpInternals()[7] +
        stepResponseInput_[currentInputSample_].second);

    stepResponseOutput_.push_back(
        {
        timeSamplesPitch_ * kLoopTimeMs / 1000.0,
        stepResponseInput_[currentInputSample_].second,
        controller.getInput() 
        });

    if (currentInputSample_ == stepResponseInput_.size() - 1)
    {
        writeStepResponse();
        spdlog::info("Finished step responce generation");
        stepResponseInProgress = false;
    }
    
}
void A2AStec30AP::readStepResponseInput()
{
    jay::util::CSVread csv(settings_.stepResponseInputFile);
    if (csv.error) { throw std::runtime_error(fmt::format("Cannot read step response input: {}", csv.error_msg)); }
    while (csv.ReadRecord())
        stepResponseInput_.push_back(std::make_pair(std::stoi(csv.fields[0]), std::stod(csv.fields[1])));
}

void A2AStec30AP::writeStepResponse()
{
    jay::util::CSVwrite csv(std::string("o-") + settings_.stepResponseInputFile);
    for (const auto& line : stepResponseOutput_)
        csv.WriteRecord({
            std::to_string(line[0]), 
            std::to_string(line[1]),
            std::to_string(line[2])});
}