#pragma once

#include "SimImpl.h"

class SimDummy : public SimImpl
{
public:
    SimDummy(const Settings& settings) {}

    bool connect() override { return true; }
    void disconnect() override {}
    [[nodiscard]] bool connected() const override { return true; }

    bool process(SimData& data, SimDataWriteFlags& flags) override { return true; }
};
