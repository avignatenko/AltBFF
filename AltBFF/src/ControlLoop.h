#pragma once

namespace bffcl
{
class UDPClient;
};
class Sim;
class Model;
class A2AStec30AP;

class ControlLoop
{
public:
    ControlLoop(bffcl::UDPClient& cl, Sim& sim, Model& model, A2AStec30AP& autopilot)
        : cl_(cl), sim_(sim), model_(model), autopilot_(autopilot)
    {
        updateCLDefaultsFromModel();
    }

    void process();

    void updateCLDefaultsFromModel();

private:
    bffcl::UDPClient& cl_;
    Sim& sim_;
    Model& model_;
    A2AStec30AP& autopilot_;
};
