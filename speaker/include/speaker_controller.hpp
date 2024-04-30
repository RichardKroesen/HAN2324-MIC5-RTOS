#ifndef SPEAKER_CONTROLLER_HPP
#define SPEAKER_CONTROLLER_HPP

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

namespace AUDIO {
class SpeakerController {
public:
    SpeakerController(uint8_t audio_pin, uint32_t pwm_wrap_value, float clk_div_value, AudioBuffer& buffer)
    : audio_pin(audio_pin), pwm_wrap_value(pwm_wrap_value), clk_div_value(clk_div_value), audioBuffer(buffer) {
        SpeakerController::instance = this; // Register this instance
        init();
    }
    
    ~SpeakerController() {
        if (SpeakerController::instance == this) {
            SpeakerController::instance = nullptr;
        }
    }

    void start() {
        audioBuffer.reset();
        irq_set_enabled(PWM_IRQ_WRAP, true);
    }

    void stop() {
        irq_set_enabled(PWM_IRQ_WRAP, false);
        uint audioPinSlice = pwm_gpio_to_slice_num(audio_pin);
        pwm_set_gpio_level(audio_pin, 0); // Reset PWM output to 0 to silence the output
        pwm_clear_irq(audioPinSlice);     // Clear any pending IRQs
    }

private:
    uint8_t audio_pin;
    const static constexpr inline uint32_t base_clock_hz = 125'000'000; // System default clock frequency.
    const static constexpr inline uint8_t pwm_repeat_cycles = 8;

    const uint32_t pwm_wrap_value;
    const float clk_div_value;
    AudioBuffer& audioBuffer;
    uint32_t effective_sample_rate = base_clock_hz / (pwm_wrap_value * clk_div_value * 8);
    uint8_t current_repeat_counter = 0; // Counts the PWM cycles for each audio sample

    static SpeakerController* instance; // Static pointer for interrupt handling

    void init() {
        gpio_set_function(audio_pin, GPIO_FUNC_PWM);
        int audioPinSlice = pwm_gpio_to_slice_num(audio_pin);

        pwm_clear_irq(audioPinSlice);
        pwm_set_irq_enabled(audioPinSlice, true);
        irq_set_exclusive_handler(PWM_IRQ_WRAP, &SpeakerController::staticPwmInterruptHandler);
        irq_set_enabled(PWM_IRQ_WRAP, true);
        
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, clk_div_value); 
        pwm_config_set_wrap(&config, pwm_wrap_value); 
        pwm_init(audioPinSlice, &config, true);
        pwm_set_gpio_level(audio_pin, 0);  // Initialize PWM output to 0
    }

    static void staticPwmInterruptHandler() {
        if (instance != nullptr) {
            instance->pwmInterruptHandler();
        }
    }

    void pwmInterruptHandler() {
        pwm_clear_irq(pwm_gpio_to_slice_num(audio_pin));

        if (current_repeat_counter <= pwm_repeat_cycles) {
            current_repeat_counter++;
        } else {
            // Advance to the next sample after repeating 8 times
            uint8_t sample = 0;
            if (audioBuffer.getNextSample(sample)) {
                pwm_set_gpio_level(audio_pin, sample);
                current_repeat_counter = 0;  // Reset the repeat counter for the new sample
            } else {
                // audioBuffer.reset();
                // End of buffer, maybe I will add a flag here or a task notification ;)
            }
        }
    }
};

SpeakerController* SpeakerController::instance = nullptr; // Initialize the static pointer

} // namespace AUDIO
#endif  // SPEAKER_CONTROLLER_HPP