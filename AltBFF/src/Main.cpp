
#include "A2ASTec30AP.h"
#include "ControlLoop.h"
#include "Model.h"
#include "PeriodicLoop.h"
#include "ReadSettings.h"
#include "Sim.h"

#include <BFFCLAPI/UDPClient.h>

#include <Utils/Common.h>
#include <Utils/PeriodicTimer.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <asio/io_context.hpp>

#include <catch2/catch_all.hpp>

#include <stdio.h>
#include <filesystem>
#include <iostream>
#include <string>

using namespace std;

#if defined(_WIN32)

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    // todo: add closing
    spdlog::shutdown();

    return FALSE;  // let others work on this
}
#endif

using std::filesystem::file_time_type;
using std::filesystem::path;

void initLogging(const LogSettings& settings)
{
    // init logging

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("altbff.log", true);
    file_sink->set_level(settings.logLevel);

    auto logger = std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list{console_sink, file_sink});

    logger->set_level(spdlog::level::trace);

    spdlog::set_default_logger(logger);

    spdlog::info("AltBFF logging started");
}

void updateCLDefaultsFromModel(bffcl::UDPClient& cl, Model& model)
{
    auto& input = cl.lockInput();
    input.aileron.frictionCoeff = model.getFrictionCoeff(Model::Aileron);
    input.aileron.dumpingCoeff = model.getDumpingCoeff(Model::Aileron);

    input.aileron.positionFollowingP = model.getPositionFollowingP(Model::Aileron);
    input.aileron.positionFollowingI = model.getPositionFollowingI(Model::Aileron);
    input.aileron.positionFollowingD = model.getPositionFollowingD(Model::Aileron);

    input.elevator.frictionCoeff = model.getFrictionCoeff(Model::Elevator);
    input.elevator.dumpingCoeff = model.getDumpingCoeff(Model::Elevator);

    input.elevator.positionFollowingP = model.getPositionFollowingP(Model::Elevator);
    input.elevator.positionFollowingI = model.getPositionFollowingI(Model::Elevator);
    input.elevator.positionFollowingD = model.getPositionFollowingD(Model::Elevator);
}

int runTests(int argc, char** argv)
{
    // remove "test" from command line
    int argc2 = argc - 1;
    std::vector<char*> argv2;
    for (int i = 0; i < argc; ++i)
        if (i != 1) argv2.push_back(argv[i]);

    int result = Catch::Session().run(argc2, &argv2[0]);
    return result;
}

bool settingsUpdateLoop(bffcl::UDPClient& cl, Model& model, A2AStec30AP& autopilot, const path& settingsPath,
                        file_time_type& settingsWriteTime)
{
    file_time_type newSettingsWriteTime = last_write_time(settingsPath);
    if (newSettingsWriteTime > settingsWriteTime)
    {
        spdlog::info("Settings updated, reloading model settings...");
        auto settings = readSettings(settingsPath);
        Model::Settings modelSettings = readModelSettings(settings);
        model.setSettings(modelSettings);

        updateCLDefaultsFromModel(cl, model);

        spdlog::info("Model settings updated");

        A2AStec30AP::Settings apSettings = readAPSettings(settings);
        autopilot.setSettings(apSettings);
        spdlog::info("AP settings updated");

        settingsWriteTime = newSettingsWriteTime;
    }
    return false;  // call again
}

int checkedMain(int argc, char** argv)
{
    path exeName = argv[0];
    path settingsPath = exeName.parent_path() / "settings.toml";

    auto settings = readSettings(settingsPath);

    ControlSettings controlSettings = readControlSettings(settings);
    bffcl::UDPClient::Settings clSettings = readCLSettings(settings);
    SimImpl::Settings simSettings = readSimSettings(settings);
    Model::Settings modelSettings = readModelSettings(settings);
    LogSettings logSettings = readLogSettings(settings);
    A2AStec30AP::Settings apSettings = readAPSettings(settings);

    initLogging(logSettings);

    // create main components
    spdlog::info("Creating main components");

    asio::io_context runner;
    auto work = asio::make_work_guard(runner);

    bffcl::UDPClient cl(clSettings, runner);
    Sim sim(simSettings);
    Model model(modelSettings);
    A2AStec30AP autopilot(apSettings);

    spdlog::info("Main components created successfully");

    updateCLDefaultsFromModel(cl, model);

    PeriodicLoop clLoop(controlSettings.clFrequency, runner, [&cl] { cl.process(); });
    PeriodicLoop simLoop(controlSettings.simFrequency, runner, [&sim] { sim.process(); });
    PeriodicLoop modelLoop(controlSettings.modelFrequency, runner, [&model] { model.process(); });
    PeriodicLoop apLoop(controlSettings.aPFrequency, runner, [&autopilot] { autopilot.process(); });

    PeriodicLoop controlLooper(30, runner, [&cl, &sim, &model, &autopilot] { controlLoop(cl, sim, model, autopilot); });

    file_time_type settingsWriteTime = last_write_time(settingsPath);
    PeriodicLoop settingsLooper(0.5, runner, [&cl, &model, &autopilot, settingsPath, &settingsWriteTime]
                                { return settingsUpdateLoop(cl, model, autopilot, settingsPath, settingsWriteTime); });

    spdlog::info("Starting main loop");
    runner.run();

    spdlog::info("Main loop finished");

    return 0;
}

int main(int argc, char** argv)
{
    // check for tests session
    if (argc > 1 && strcmp(argv[1], "test") == 0) return runTests(argc, argv);

#if defined(_WIN32)
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#endif

    try
    {
        return checkedMain(argc, argv);
    }
    catch (const std::exception& e)
    {
        spdlog::critical("Exception: {}", e.what());
        spdlog::shutdown();
        throw;
    }
    catch (...)
    {
        spdlog::critical("Unknown exception");
        spdlog::shutdown();
        throw;
    }
}
