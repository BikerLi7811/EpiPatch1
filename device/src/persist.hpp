#pragma once

#include <cstdint>

namespace dev::persist {

int load(uint8_t &deployments);
int save(uint8_t deployments);

} // namespace dev::persist
