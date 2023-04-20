#pragma once

#include "main.h"
#include "cmsis_os.h"


namespace sml = boost::sml;

struct e_btn {};
struct e_call_end {};
struct e_timeout {};

enum class Event {
    BUTTON_PRESS,
    CALL_END
};
