#pragma once

#include <spdlog/spdlog.h>
#include <filesystem>

#include "A2ASTec30AP.h"
#include "Model.h"
#include "Sim.h"

#include <BFFCLAPI/UDPClient.h>

#include <toml.hpp>

struct LogSettings
{
    spdlog::level::level_enum logLevel = spdlog::level::info;
};

toml::value readSettings(const std::filesystem::path& file);

LogSettings readLogSettings(const toml::value& settings);
bffcl::UDPClient::Settings readCLSettings(const toml::value& settings);
Sim::Settings readSimSettings(const toml::value& settings);
A2AStec30AP::Settings readAPSettings(const toml::value& settings);
Model::Settings readModelSettings(const toml::value& settings);
