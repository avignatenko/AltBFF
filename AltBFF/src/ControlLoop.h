#pragma once

namespace bffcl
{
class UDPClient;
};
class Sim;
class Model;
class A2AStec30AP;

bool controlLoop(bffcl::UDPClient& cl, Sim& sim, Model& model, A2AStec30AP& autopilot);
