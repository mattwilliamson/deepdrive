#include <stdint.h>
#include <stdio.h>

// Error codes can be found in
// micro_ros_raspberrypi_pico_sdk/libmicroros/include/rcl/types.h
// and micro_ros_raspberrypi_pico_sdk/libmicroros/include/rmw/ret_types.h
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// On the Pico W, the LED is connected to the wifi module
#if LIB_PICO_CYW43_ARCH
#include "pico/cyw43_arch.h"
#endif

const uint PIN_MOTOR_A = 0;
const uint PIN_MOTOR_B = 1;
const uint PIN_PULSE_A = 2;
uint8_t on = 0;
int8_t direction = 1;
uint pulses = 0;
int speed = 65000;
const uint max_speed = 65000;
bool led_on = true;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

// Set up subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 cmd;

void blink_error() {
#if LIB_PICO_CYW43_ARCH
  for (int i = 0; i < 10; i++) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(200);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(100);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(400);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(200);
  }
#endif
}

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      blink_error();                                                           \
      printf("Failed status on line %d: (error code: %d) Aborting.\n",         \
             __LINE__, (int)temp_rc);                                          \
      return 1;                                                                \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      blink_error();                                                           \
      printf("Failed status on line %d: (error code: %d). Continuing.\n",      \
             __LINE__, (int)temp_rc);                                          \
    }                                                                          \
  }

// Pulse counter
void gpio_callback(uint gpio, uint32_t events) { pulses++; }

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // Blink to show activity
  led_on = !led_on;
#if LIB_PICO_CYW43_ARCH
  if (led_on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  } else {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  }
#endif

  // msg.data = pulses;
  msg.data = speed;
  if (RCL_RET_OK != rcl_publish(&publisher, &msg, NULL)) {
    blink_error();
  }
  pulses = 0;

  on = !on;
  gpio_put(PIN_MOTOR_A, 0);
  // gpio_put(PIN_MOTOR_B, on);
  // speed is max uint16_t or INT_MAX?
  // Set PWM level to speed
  if (speed > max_speed) {
    speed = max_speed;
  }
  pwm_set_gpio_level(PIN_MOTOR_B, speed);

  if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
    // Lost connection to agent. Stop motors.
    pwm_set_gpio_level(PIN_MOTOR_B, 0);
    printf("micro-ROS agent has stopped. Exiting...\n");
    exit(1);
  } else {
    printf("Agent is still up!\n\n");
  }
}

// Subscriber callback
void subscription_callback(const void *msgin) {
  // Cast received message to used type
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  speed = m->data;
}

int main() {
#if LIB_PICO_CYW43_ARCH
  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed");
    return -1;
  }
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
#endif

  gpio_put(14, 1);

  blink_error();

  uint PIN_OUT = 6;
  stdio_init_all();
  gpio_set_function(PIN_OUT, GPIO_FUNC_PWM);
  auto slice   = pwm_gpio_to_slice_num(PIN_OUT);
  auto channel = pwm_gpio_to_channel(PIN_OUT);
  pwm_set_clkdiv(slice, 256.0f);  /// Setting the divider to slow down the clock
  pwm_set_wrap(slice, 9804);      /// setting the Wrap time to 9764 (20 ms)
  pwm_set_enabled(slice, true);


    for (uint i = 0; i < 10; ++i)
    {

        pwm_set_chan_level(slice, channel, 490);  /// Setting the duty period (0.6 ms)
        sleep_ms(2000);
        pwm_set_chan_level(slice, channel, 735);  /// Setting the duty period (1.5 ms)
        sleep_ms(2000);
        pwm_set_chan_level(slice, channel, 1176);  /// Setting the duty period (2.4 ms)
        sleep_ms(2000);
    }



  led_loop();

  msg.data = 0;

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  gpio_init(PIN_MOTOR_A);
  gpio_set_dir(PIN_MOTOR_A, GPIO_OUT);

  // gpio_init(PIN_MOTOR_B);
  // gpio_set_dir(PIN_MOTOR_B, GPIO_OUT);

  // Pulse counter (rotary encoder) input
  gpio_set_irq_enabled_with_callback(PIN_PULSE_A,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, &gpio_callback);

  // PWM Out for motor control
  uint slice_num = pwm_gpio_to_slice_num(PIN_MOTOR_B);
  gpio_set_function(PIN_MOTOR_B, GPIO_FUNC_PWM);
  pwm_set_wrap(slice_num, max_speed); // default clock is 125MHz (8ns)
  pwm_set_enabled(slice_num, true);
  pwm_set_gpio_level(PIN_MOTOR_B, 0);

  // ROS2 Node
  rcl_timer_t timer;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  // Number of handles allowed in the executor (1 timer, 1 subscription)
  const size_t handles = 2;

  allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  // Connect to agent and init node
  RCCHECK(rmw_uros_ping_agent(timeout_ms, attempts));
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "deepdrive_micro_node", "", &support));

  // Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "deepdrive_micro/pulses"));

  // Timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100),
                                  timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Subscriber
  // ros2 topic pub --once deepdrive_micro/cmd std_msgs/msg/Int32 "{data:
  // 65000}"
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "deepdrive_micro/cmd"));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd,
                                         &subscription_callback, ON_NEW_DATA));

  // while (true) {
  //   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // }
  rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));

  return 0;
}
