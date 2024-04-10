#include "utils.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "drivetrain_config.h"

volatile bool ESCdelayInProgress = false;
volatile bool ESCirqTriggered = false;

#define I2C_RECOVERY_CLOCKS 9

static int i2c_reinit_attempts = 0; // Counter for reinitialization attempts
const int max_i2c_reinit_attempts = 4;

// Wrapper function to manage recovery attempts and reinitialization
void handleI2CError(i2c_inst_t* &i2c_port) {
    printf("handling i2c bus error %d\n", i2c_reinit_attempts);
    fflush(stdout); 
    //sleep_ms(200);
    if (++i2c_reinit_attempts >= max_i2c_reinit_attempts ) {
        initI2C(i2c_port, true); // Force recovery on the nth attempt
        //initI2C(i2c_port, false); // Force recovery on the nth attempt
        i2c_reinit_attempts = 0; // Reset attempts after recovery
    } else {
        initI2C(i2c_port, false); // Regular reinitialization without forcing recovery
    }
}

void i2cBusRecovery(uint sda_pin, uint scl_pin) {
    printf("attempting bus recovery\n");
    fflush(stdout); 
    gpio_init(sda_pin);
    gpio_init(scl_pin);

    gpio_set_dir(sda_pin, GPIO_IN);
    gpio_set_dir(scl_pin, GPIO_OUT);

    // Attempt to clock out any stuck slave devices
    for (int i = 0; i < I2C_RECOVERY_CLOCKS; ++i) {
        gpio_put(scl_pin, 0);
        sleep_us(5); // Clock low period
        gpio_put(scl_pin, 1);
        sleep_us(5); // Clock high period

        // Check if SDA line has been released by slave
        if (gpio_get(sda_pin)) {
            break; // SDA line is high; assume bus is free
        }
    }

    // Generate a stop condition in case a slave is still in the middle of a transaction
    gpio_set_dir(sda_pin, GPIO_OUT);
    gpio_put(scl_pin, 0);
    sleep_us(5);
    gpio_put(sda_pin, 0);
    sleep_us(5);
    gpio_put(scl_pin, 1);
    sleep_us(5);
    gpio_put(sda_pin, 1);
    sleep_us(5);
    printf("recovery complete\n");
    fflush(stdout); 
}

void initI2C(i2c_inst_t* &i2c_port, bool force_recovery) {
    i2c_port = i2c_default; // or i2c0, i2c1, etc.

    // Attempt I2C bus recovery before de-initializing pins
    if (force_recovery) {
        i2cBusRecovery(CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN);
    }
    printf("attempting bus reinit\n");
    fflush(stdout); 
    // de-initialise I2C pins in case
    gpio_disable_pulls(CONFIG::I2C_SDA_PIN);
    gpio_set_function(CONFIG::I2C_SDA_PIN, GPIO_FUNC_NULL);
    gpio_disable_pulls(CONFIG::I2C_SCL_PIN);
    gpio_set_function(CONFIG::I2C_SCL_PIN, GPIO_FUNC_NULL);

    // initialise I2C with baudrate
    i2c_init(i2c_port, CONFIG::I2C_BAUD_RATE);

    gpio_set_function(CONFIG::I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(CONFIG::I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(CONFIG::I2C_SDA_PIN);
    gpio_pull_up(CONFIG::I2C_SCL_PIN);
    printf("reinit complete\n");
    fflush(stdout); 
}

float wrap_pi(const float heading) {
    // constrain heading to within +/-pi (+/-180 degrees) without changing the meaning of the angle
    // if its more than pi (+180), subtract 2*pi (subtract 360degrees) so we have the "smaller" angle 
    float wrapped = heading;

    if (heading > M_PI) {
        wrapped = heading - M_TWOPI;
    } else if (heading < -M_PI) {
        wrapped = heading + M_TWOPI;
    }

    return static_cast<float>(wrapped);
}

void initMotorMonitorPins() {
    using namespace CONFIG;
    
    // Configure motorSleepPin as output and set it to high
    gpio_init(motorSleepPin);
    gpio_set_dir(motorSleepPin, GPIO_OUT);
    gpio_put(motorSleepPin, true);

    // Configure motorStatusPin as input
    gpio_init(motorStatusPin);
    gpio_set_dir(motorStatusPin, GPIO_IN);
    printf("motor pins initialised\n");
}

void handlerMotorController(uint gpio, uint32_t events) {
    ESCirqTriggered = true;
}

// Non-blocking delay function
bool non_blocking_delay_us(uint32_t delay_us) {
    static absolute_time_t start_time;
    static bool delay_started = false;

    if (!delay_started) {
        start_time = get_absolute_time();
        delay_started = true;
    }

    if (absolute_time_diff_us(start_time, get_absolute_time()) >= delay_us) {
        delay_started = false; // Reset for the next delay call
        return true; // Delay completed
    }

    return false; // Delay still ongoing
}

void toggleMotorSleepPin(){
    gpio_put(CONFIG::motorSleepPin, false);
    sleep_us(15);
    gpio_put(CONFIG::motorSleepPin, true);
    sleep_us(10);
}