/*
 * ============================================================================
 *   TAUVER Link Kit · micro-ROS firmware for NUCLEO-G491RE
 * ============================================================================
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// ---------------------------------------------------------------------------
// [A] CONFIG & ROS ENTITIES
// ---------------------------------------------------------------------------
#define FW_VERSION "microros-1.0.0"

const uint32_t TEL_MS_IDLE   = 1000;    
const uint32_t TEL_MS_SIM     = 250;     
const float    SIM_G_PER_TICK = 4.0f;    
const float    SIM_MAX_G      = 400.0f;  

#if defined(ARDUINO_ARCH_STM32)
  #define BOARD_NAME "NUCLEO-G491RE"
  const uint32_t PIN_LED = PA5;         // LD2, green user LED
#else
  #define BOARD_NAME "ARDUINO"
  const uint32_t PIN_LED = LED_BUILTIN; 
#endif

// micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__String msg_pub;

rcl_subscription_t subscriber;
std_msgs__msg__String msg_sub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ---------------------------------------------------------------------------
// [B] STATE
// ---------------------------------------------------------------------------
enum LedMode { LED_AUTO, LED_ON, LED_OFF };

LedMode  ledMode   = LED_AUTO;
bool     simOn     = false;
bool     lidClosed = false;
float    rockG     = 0.0f;
uint32_t seq       = 0;
uint32_t lastTel   = 0;

// Forward declarations
void executeCommand(const char* cmd_str);

// ---------------------------------------------------------------------------
// [C] SUBSCRIPTION CALLBACK (Incoming Commands from Jetson)
// ---------------------------------------------------------------------------
void subscription_callback(const void * msin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msin;
  if (msg->data.data != NULL) {
    executeCommand(msg->data.data);
  }
}

// ---------------------------------------------------------------------------
// [D] TELEMETRY & PUBLISHING HELPER
// ---------------------------------------------------------------------------
void publishTelemetry(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    uint32_t now = millis();

    // Simulation update logic
    uint32_t period = simOn ? TEL_MS_SIM : TEL_MS_IDLE;
    if ((uint32_t)(now - lastTel) >= period) {
      lastTel = now;
      if (simOn) {
        rockG += SIM_G_PER_TICK;
        if (rockG >= SIM_MAX_G) {
          rockG = SIM_MAX_G;
          simOn = false;
        }
      }

      // Build JSON telemetry string to match the original contract
      char buffer[180];
      int len = snprintf(buffer, sizeof(buffer),
        "{\"type\":\"tel\",\"seq\":%lu,\"ms\":%lu,\"data\":{\"rock_g\":%.1f,\"lid_rock\":\"%s\",\"sim\":%d,\"led\":\"%s\"}}",
        seq++, now, rockG, lidClosed ? "CLOSED" : "OPEN", simOn ? 1 : 0,
        ledMode == LED_ON ? "ON" : (ledMode == LED_OFF ? "OFF" : "AUTO"));

      msg_pub.data.data = buffer;
      msg_pub.data.size = len;
      msg_pub.data.capacity = sizeof(buffer);

      rcl_publish(&publisher, &msg_pub, NULL);
    }
  }
}

// ---------------------------------------------------------------------------
// [E] COMMAND PARSER & EXECUTOR
// ---------------------------------------------------------------------------
void executeCommand(const char* cmd) {
  // Simple case-insensitive comparisons for commands
  if (strstr(cmd, "CMD:LED=ON")) {
    ledMode = LED_ON;
  } else if (strstr(cmd, "CMD:LED=OFF")) {
    ledMode = LED_OFF;
  } else if (strstr(cmd, "CMD:LED=AUTO")) {
    ledMode = LED_AUTO;
  } else if (strstr(cmd, "CMD:SIM=ON")) {
    if (!lidClosed) {
      simOn = true;
      rockG = 0.0f;
    }
  } else if (strstr(cmd, "CMD:SIM=OFF")) {
    simOn = false;
  } else if (strstr(cmd, "CMD:LID=ROCK,CLOSE")) {
    lidClosed = true;
    simOn = false;
  } else if (strstr(cmd, "CMD:LID=ROCK,OPEN")) {
    lidClosed = false;
    rockG = 0.0f;
  } else if (strstr(cmd, "CMD:RESET")) {
    ledMode = LED_AUTO;
    simOn = false;
    lidClosed = false;
    rockG = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// [F] LED BLINK LOGIC
// ---------------------------------------------------------------------------
void updateLed() {
  uint32_t now = millis();
  bool on;
  if (lidClosed)               on = true;
  else if (ledMode == LED_ON)  on = true;
  else if (ledMode == LED_OFF) on = false;
  else {
    uint32_t period = simOn ? 250 : 1000;
    on = (now % period) < 60;
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// [G] SETUP & LOOP
// ---------------------------------------------------------------------------
void setup() {
  // Configure micro-ROS serial transport (using ST-LINK VCP pins PA2/PA3)
#if defined(ARDUINO_ARCH_STM32)
  Serial.setRx(PA3);
  Serial.setTx(PA2);
#endif
  set_microros_transports();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "tauver_nucleo_node", "", &support);

  // Create publisher (/tauver/telemetry)
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "tauver/telemetry");

  // Create subscriber (/tauver/command)
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "tauver/command");

  // Allocate message buffers
  String_init(&msg_pub.data);
  String_init(&msg_sub.data);

  // Create timer for telemetry loop (runs every 100ms to check intervals)
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    publishTelemetry);

  // Create executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  lastTel = millis();
}

void loop() {
  // Spin micro-ROS executor to handle callbacks and timers
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  updateLed();
}