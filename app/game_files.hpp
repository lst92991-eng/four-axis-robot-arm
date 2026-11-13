#pragma once
#include "game.hpp"

namespace scara
{
void Files_open(const std::string &directory, bool simulated);
void Motion_request(const std::string &purpose, const std::string &move, const std::vector<PickStep> &steps,
                    std::string &ticket);
void Motion_wait(const std::string &ticket, bool &success);
bool Motion_cancel();
std::string Runtime_path(const std::string &name);
} // namespace scara
