#pragma once


#include "sml.hpp"
#include "cmsis_os2.h"
#include "stm32f4_discovery.h"

#define CONFIG_CALL_TARGET_USER 		"**611"
#define CONFIG_CALLER_DISPLAY_MESSAGE 	"Türklingel"

namespace sml = boost::sml;

struct e_btn {};
struct e_call_end {};
struct e_timeout {};

template<class SipClientT>
struct dependencies {
    auto operator()() const noexcept {
        using namespace sml;

        const auto action_call = [](SipClientT& d, const auto& event) {
            d.request_ring(CONFIG_CALL_TARGET_USER, CONFIG_CALLER_DISPLAY_MESSAGE);
        };

        const auto action_cancel = [](SipClientT& d, const auto& event) {
            d.request_cancel();
        };

        return make_transition_table(
                *"idle"_s        + event<e_btn> / action_call = "sRinging"_s
                , "sRinging"_s   + event<e_timeout> / action_cancel = "idle"_s
                , "sRinging"_s   + event<e_call_end> = "idle"_s
        );
    }
};

enum class Event {
    BUTTON_PRESS,
    CALL_END
};

template<class SipClientT, int RING_DURATION_TIMEOUT_MSEC>
class ButtonInputHandler {
public:
	osMessageQueueId_t m_queue;

    explicit ButtonInputHandler(SipClientT& client)
    : m_client{client}
    , m_sm{client}
    {
        m_queue = osMessageQueueNew(10, sizeof(Event), NULL);//xQueueCreate(10, sizeof(Event));

        BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
    }

    void run() {
        using namespace sml;
        for (;;)
        {
            Event event;
            TickType_t timeout = m_sm.is("idle"_s) ? osWaitForever : RING_DURATION_TICKS;
//xQueueReceive(m_queue, &event, timeout)
            if(osMessageQueueGet(m_queue, &event, NULL, timeout)) {
                if (event == Event::BUTTON_PRESS) {
                    m_sm.process_event(e_btn{});
                } else if (event == Event::CALL_END) {
                    m_sm.process_event(e_call_end{});
                }
            } else {
                m_sm.process_event(e_timeout{});
            }
        }
    }

    void call_end() {
        Event event = Event::CALL_END;
        // don't wait if the queue is full
        //xQueueSend(m_queue, &event, ( TickType_t ) 0);
        osMessageQueuePut(m_queue, &event, 0, osWaitForever);
    }

private:
    SipClientT& m_client;
    //QueueHandle_t m_queue;


//    using ButtonInputHandlerT = ButtonInputHandler<SipClientT, RING_DURATION_TIMEOUT_MSEC>;

//    static void int_handler(void *args) {
//        ButtonInputHandlerT* obj  = static_cast<ButtonInputHandlerT*>(args);
//        Event event = Event::BUTTON_PRESS;
//        osMessageQueuePut(obj->m_queue, &event, osPriorityISR, NULL);
//        //xQueueSendToBackFromISR(obj->m_queue, &event, NULL);
//    }

    sml::sm<dependencies<SipClientT>> m_sm;

    static constexpr uint32_t RING_DURATION_TICKS = RING_DURATION_TIMEOUT_MSEC / portTICK_PERIOD_MS;
};
