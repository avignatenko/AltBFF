#include "A2ASTec30AP.h"

#include "Sim.h"
#include "Model.h"
#include <BFFCLAPI/CLStructures.h>

#include <fmt/ranges.h>

namespace 
{
const double kPi = std::acos(-1);
}

void A2AStec30AP::enableRollAxis(bool enable) 
{ 
    rollEnabled_ = enable; 
}

void A2AStec30AP::enablePitchAxis(bool enable)
{
    if (pitchEnabled_ == enable) return;

    if (enable)
    {
        const double kLoopTimeMs = 1000.0 / 30;
        pitchController_ = PitchController
        {
            static_cast<PitchController::Mode>(settings_.pitchmode),
            // fpm controller
            PIDController(settings_.fpmPID.p, settings_.fpmPID.i, settings_.fpmPID.d,
                         -settings_.fpmMax, settings_.fpmMax, kLoopTimeMs),
            // pitch controller
            PIDController(settings_.pitchPID.p, settings_.pitchPID.i, settings_.pitchPID.d, 
                          -settings_.pitchMax, settings_.pitchMax, kLoopTimeMs),
            // pitch rate controller
            PIDController(settings_.pitchRatePID.p, settings_.pitchRatePID.i, settings_.pitchRatePID.d, 
                          -settings_.pitchRate, settings_.pitchRate, kLoopTimeMs),
            // elevator controller
            PIDController(settings_.elevatorPID.p, settings_.elevatorPID.i, settings_.elevatorPID.d, 
                          -100, 100 , kLoopTimeMs)
        };

        spdlog::info("AP Pitch mode: {}", settings_.pitchmode);

        switch (pitchController_.value().mode)
        {
        case PitchController::Mode::Pitch: 
            pitchController_.value().pitchRateController.setSetPoint(simPitch_);    
            spdlog::info("AP pitch enabled with target pitch: {}", simPitch_); 
            break;
        case PitchController::Mode::FPM:   
            pitchController_.value().pitchController.setSetPoint(simFpm_);
            spdlog::info("AP pitch enabled with target fpm: {}", simFpm_);
            break;
        case PitchController::Mode::Alt:   
            pitchController_.value().fpmController.setSetPoint(simPressure_);
            spdlog::info("AP pitch enabled with target pressure: {}", simPressure_);
            break;
        }
     
        elevatorOut_ = simElevator_;
      
    }
    else
        spdlog::info("AP pitch disabled");

    pitchEnabled_ = enable;

  
}

void A2AStec30AP::process()
{

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
            fpmController.setInput(simPressure_);
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
            pitchController.setOutputBase(simPitch_);
            pitchController.compute();

            spdlog::trace("pitch pid: {}", pitchController.dumpInternals());
            spdlog::trace("total output pitch: {}", pitchController.getOutput());
        }

        // mode Pitch = 0
        // pitch rate controller: pitch -> pitch rate (absolute!)
        // todo: check with pitch rate offset
        // error = 0 ? set pitch rate to 0
        PIDController& pitchRateController = pitchController_.value().pitchRateController;
        if (settings_.pitchmode > 0)
          pitchRateController.setSetPoint(pitchController_.value().pitchController.getOutput());
        pitchRateController.setInput(simPitch_);
        pitchRateController.compute();
        
        // elevator controller: pitch rate -> elevator offset
        // error = 0 ? keep elevator as is
        PIDController& elevatorController = pitchController_.value().elevatorController;   
        elevatorController.setSetPoint(pitchRateController.getOutput());
        elevatorController.setInput(simPitchRate_);
        elevatorController.setOutputBase(elevatorOut_);
        elevatorController.compute();

        // check for excessive forces

        double forceError = std::abs(clForceElevator_) - settings_.pitchStartDegradeCLForce;

        double forceShift = 0.0; 
        if (forceError > 0)
        {
            forceShift = settings_.pitchMaxCLForce * std::copysign(forceError, clForceElevator_);
            spdlog::trace("AP force coeff: {}, AP Force Shift: {}", forceError, forceShift);
        }
       
        // finally send back
        elevatorOut_ = elevatorController.getOutput() + forceShift;

    
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