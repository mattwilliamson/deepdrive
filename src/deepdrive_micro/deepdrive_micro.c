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

typedef struct {
  char *topic;
  uint pulses;
  std_msgs__msg__Int16 cmd_left;
  uint16_t pwm_output; // Added property for PWM output
  uint pin; // Added property for pin number
  // Add more fields here as needed
  rcl_subscription_t subscription;
  void (*callback)(const void *msg); // Callback function pointer
  // Add more fields here as needed
} Motor;

Motor motors[MOTOR_COUNT] = {
  {"front/left", 0, {0}, 0, 1, /* Initialize other fields as needed */, NULL},
  {"front/right", 0, {0}, 0, 2, /* Initialize other fields as needed */, NULL},
  {"back/left", 0, {0}, 0, 3, /* Initialize other fields as needed */, NULL},
  {"back/right", 0, {0}, 0, 4, /* Initialize other fields as needed */, NULL},
};

void motor_callback(const void *msg, void *context) {
  int motor_index = *(int *)context;
  // Rest of the function code...

  const std_msgs__msg__Int16 *m = (const std_msgs__msg__Int16 *)msg;
  Motor *motor = &motors[motor_index];
  // Set the pwm_output field for the motor
  motor->pwm_output = m->data;
}


void initialize_motors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Motor *motor = &motors[i];
    // Assign the pin number to the motor struct
    motor->pin = i;
    // Initialize the subscription for each motor
    rcl_subscription_options_t subscription_options =
        rcl_subscription_get_default_options();
    rcl_ret_t ret = rcl_subscription_init(
        &motor->subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        motor->topic, &subscription_options);
    // Handle the initialization error if needed
    // Set the callback function for each motor
    motor->callback = motor_callback; // Assign the callback function
    ret = rclc_executor_add_subscription_with_context(
      &executor,
      &motor->subscription,
      &msg,
      motor->callback,
      (void *)&i,
      RCLC_EXECUTOR_ALWAYS);
    // Handle the setting of callback error if needed
    // Initialize other motor fields as needed
  }
}

uint8_t on = 0;
uint pulses = 0;
int cmd_left = 65000;
const uint max_speed = 65000;
bool led_on = true;

#define MOTOR_COUNT 4
char *topics[] = {"front/left", "front/right", "back/left", "back/right"};

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

// Set up subscriber
rcl_subscription_t sub_left_motor;
rcl_subscription_t sub_right_motor;
std_msgs__msg__Int32 cmd_left;

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
  msg.data = cmd_left;
  if (RCL_RET_OK != rcl_publish(&publisher, &msg, NULL)) {
    blink_error();
  }
  pulses = 0;

  on = !on;
  gpio_put(PIN_MOTOR_A, 0);
  // gpio_put(PIN_MOTOR_B, on);
  // cmd_left is max uint16_t or INT_MAX?
  // Set PWM level to cmd_left
  if (cmd_left > max_speed) {
    cmd_left = max_speed;
  }
  pwm_set_gpio_level(PIN_MOTOR_B, cmd_left);

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
void sub_left_motor_cb(const void *msgin) {
  // Cast received message to used type
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  cmd_left = m->data;
}

void sub_right_motor_cb(const void *msgin) {
  // Cast received message to used type
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  cmd_right = m->data;
}

int main() {
#if LIB_PICO_CYW43_ARCH
  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed");
    return -1;
  }
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
#endif

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
  // TODO: publish to all 4 topics
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/diff_drive/{x}/{y}/pulses"));

  // Timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100),
                                  timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Subscriber
  // ros2 topic pub --once deepdrive_micro/cmd_left std_msgs/msg/Int32 "{data:
  // 65000}"

  // Subscriber for motor control left side
  RCCHECK(rclc_subscription_init_best_effort(
      &sub_left_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/diff_drive/back/left/pwm"));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_left_motor, &cmd_left,
                                         &sub_left_motor_cb, ON_NEW_DATA));

  // Subscriber for motor control right side
  RCCHECK(rclc_subscription_init_best_effort(
      &sub_right_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/diff_drive/back/right/pwm"));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_left_motor, &cmd_left,
                                         &sub_right_motor_cb, ON_NEW_DATA));

  // while (true) {
  //   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // }
  rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_subscription_fini(&sub_left_motor, &node));
  RCCHECK(rcl_node_fini(&node));

  return 0;
}
