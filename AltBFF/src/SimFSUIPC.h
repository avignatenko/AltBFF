#pragma once

#include "SimImpl.h"

class SimFSUIPC : public SimImpl
{
public:
    SimFSUIPC(const Settings& settings);

    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool connected() const override;

    bool process(SimData& data, SimDataWriteFlags& flags) override;

private:
    Settings settings_;
    bool connected_ = false;
};
