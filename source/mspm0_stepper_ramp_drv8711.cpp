
#include "ti_msp_dl_config.h"
#include "hal.h"
#include <array>
#include <algorithm> // for std::transform


import stepper_driver;                          // wakeup class
import drv8711;                                 // driver classes and registers
import drv8711_config;                          // register pre-configuration
import drv8711_mspm0;                           // hw driver classes and registers
import mspm0_pwm_stepper_driver;                // pwm step and dir driver
import mspm0_pwm_stepper_ramp_driver;           // pwm step and dir driver

const uint32_t dir = GPIO_GRP_DRIVER_PIN_DIR_PIN;
const uint32_t n_enable = GPIO_GRP_DRIVER_PIN_N_ENABLE_PIN;
const uint32_t cs = GPIO_GRP_DRIVER_PIN_SPI_0_CS_PIN;

const uint32_t microstep_x = 16;
const uint32_t max_step_freq = 300000;

// object to manage the drv8711 IC used for motor1
using driver_t = drv8711_mspm0::drv8711_mspm0;
driver_t driver1(SPI_0_INST, cs, GPIO_GRP_DRIVER_PORT, n_enable, microstep_x);

// object to manage the stepper motor using PWM for step and dir.
// uncomment the one to use:
// using pwm_stepper_driver_t = mspm0_pwm_stepper_driver::mspm0_pwm_stepper_driver;
using pwm_stepper_driver_t = mspm0_pwm_stepper_driver::mspm0_pwm_stepper_ramp_driver;
pwm_stepper_driver_t pwm_stepper_driver1(dir, GPIO_GRP_DRIVER_PORT, max_step_freq);


// S-curve pre-calculated profile (Speed values in Pulses Per Second / Hz)
// used by he mspm0_pwm_stepper_ramp_driver to implement a smooth acceleration and deceleration profile
const unsigned int RAMP_STEPS = 43;
// S-curve pre-calculated profile (Speed values in Pulses Per Second / Hz)
// TODO: translate to raw Hardware Timer Ticks (Highly Recommended for Performance)
inline static /*const*/ std::array<uint32_t, RAMP_STEPS> s_curve_lut = {
    150,  151,  152,  155,  160,  166,  174,  184,  196,  211,
    228,  247,  269,  293,  321,  352,  386,  424,  465,  509,  
    557,  608,  662,  719,  779,  842,  908,  976,  1046, 1118, 
    1191, 1265, 1340, 1414, 1489, 1563, 1636, 1707, 1777, 1845, 
    1910, 1971, 2000
};

// Function to introduce a delay of approximately 200 nanoseconds
// A4988 needs this time, after changing dir/enable/microstep pins, before the next step pulse is sent
inline void wait_200ns() {
    delay_cycles(7);
}

int main() {
    SYSCFG_DL_init();
    
    /* Initialize the HAL module */
    HAL_init(); // TODO step away from HAL?
    
    // drv8711::reg_torque.torque = 0x00ff; 
    driver1.init();
    driver1.microsteps(microstep_x);
    
    pwm_stepper_driver1.init();
    pwm_stepper_driver1.stop_motor();
    pwm_stepper_driver1.set_control_mode(mspm0_pwm_stepper_driver::control_mode::step);

    // not used by the ramp driver, but needed for the mspm0_pwm_stepper_driver::mspm0_pwm_stepper_driver to work
    // pwm_stepper_driver1.set_speed(1); // CURVE determines speed for ramp style motion, so set_speed() is not used in this example

    std::transform(s_curve_lut.begin(), s_curve_lut.end(), s_curve_lut.begin(), [](uint32_t val) {
        return val * 11.5; // Adjust the divisor to scale the speed values as needed
    });    

    pwm_stepper_driver1.set_ramp(s_curve_lut); // the vector has to stay alive for the duration of the motor run, so it is stored in a static variable in this example 

    /* Set the priority for interrupts */
    NVIC_SetPriority(GENERIC_PWM_CHANNEL_0_INST_INT_IRQN, 1);
    NVIC_SetPriority(TIMER_RAMP_INST_INT_IRQN, 2);

    while (1) {
        stepper_driver::wakeup w(driver1);

        
        pwm_stepper_driver1.set_dir(true);
        wait_200ns();
        pwm_stepper_driver1.set_step(100000);
        pwm_stepper_driver1.start_motor();
        
        while (pwm_stepper_driver1.is_motor_running()) {
            __WFI(); // Enter low-power sleep mode; wake on timer interrupts
        }
        
        pwm_stepper_driver1.set_dir(false);
        wait_200ns();
        pwm_stepper_driver1.set_step(100000);
        pwm_stepper_driver1.start_motor();       
        
        while (pwm_stepper_driver1.is_motor_running()) {
            __WFI(); // Enter low-power sleep mode; wake on timer interrupts
        }

    }
}

extern "C" {
    
    void GENERIC_PWM_CHANNEL_0_INST_IRQHandler() {
        if (DL_Timer_getPendingInterrupt(GENERIC_PWM_CHANNEL_0_INST) == DL_TIMER_IIDX_ZERO) {
            pwm_stepper_driver1.check_step();
        }
    }

    void TIMER_RAMP_INST_IRQHandler(void) {
        if (DL_Timer_getPendingInterrupt(TIMER_RAMP_INST) == DL_TIMER_IIDX_ZERO) {
            pwm_stepper_driver1.check_ramp();
        }
    }

}
