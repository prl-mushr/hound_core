#include "hound_core/pid.hpp"

// Header-only Pid — this TU exists so CMake can list src/pid.cpp cleanly and
// so future non-inline helpers have a home. Keep the class in the header for
// easy unit testing without link dependencies.
namespace hound_core
{
}  // namespace hound_core
