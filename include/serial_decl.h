// Ensure the global Serial instance is declared in every translation unit
// Some third-party libraries assume 'Serial' is available; include this
// header implicitly via -include in build_flags so all .cpp files see it.

#ifndef SERIAL_DECL_H
#define SERIAL_DECL_H

#include <Arduino.h>

// Forward-declare HardwareSerial to avoid depending on full type visibility
// when this header is force-included very early via -include.
class HardwareSerial;
extern HardwareSerial Serial;

#endif // SERIAL_DECL_H
