#pragma once
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace esphome_hotcirc {

// Structure for storing learning matrix in flash
struct LearnMatrixData {
  uint8_t learn[7][48];  // Changed to 48 half-hourly slots
  uint32_t checksum;  // Simple validation
};

class HotWaterController : public Component {
 public:
  // Pump trigger types - defines what caused the pump to start
  enum class PumpTrigger {
    NONE,              // Pump not running
    MANUAL_BUTTON,     // Started by hardware button
    MANUAL_WEBUI,      // Started by Web UI button
    WATER_DRAW,        // Started by detected water usage
    SCHEDULED,         // Started by learning/schedule
    DISINFECTION,      // Started by disinfection cycle detection
    ANTI_STAGNATION    // Started by anti-stagnation (prevents pump seizure)
  };

  // Sensors & actuators
  sensor::Sensor *outlet_{nullptr};
  sensor::Sensor *ret_{nullptr};
  switch_::Switch *pump_{nullptr};
  time::RealTimeClock *clock_{nullptr};
  binary_sensor::BinarySensor *button_{nullptr};
  output::BinaryOutput *led_green_{nullptr};
  output::BinaryOutput *led_yellow_{nullptr};

  // Configuration setters
  void set_outlet_sensor(sensor::Sensor *s) { outlet_ = s; }
  void set_return_sensor(sensor::Sensor *s) { ret_ = s; }
  void set_pump_switch(switch_::Switch *s) { pump_ = s; }
  void set_time_source(time::RealTimeClock *t) { clock_ = t; }
  void set_led_green(output::BinaryOutput *led) { led_green_ = led; }
  void set_led_yellow(output::BinaryOutput *led) { led_yellow_ = led; }
  void set_button(binary_sensor::BinarySensor *btn) { button_ = btn; }

  void set_thresholds(float outlet_rise_deg, float return_rise_deg, float disinfection_temp_rise, float min_return_temp) {
    this->temp_rise_threshold_ = outlet_rise_deg;
    this->return_rise_threshold_ = return_rise_deg;
    this->disinfection_temp_threshold_ = disinfection_temp_rise;
    this->min_return_temp_ = min_return_temp;
  }

  void set_pump_flow_rate(float flow_rate_lpm) {
    this->pump_flow_rate_ = flow_rate_lpm;
  }

  void set_anti_stagnation_interval(uint32_t interval_seconds) {
    this->ANTI_STAGNATION_INTERVAL = interval_seconds;
  }

  void set_anti_stagnation_runtime(uint32_t runtime_seconds) {
    this->ANTI_STAGNATION_RUNTIME = runtime_seconds;
  }

  float get_last_cycle_energy() const {
    return last_cycle_energy_;
  }

  uint32_t get_last_cycle_duration() const {
    return last_cycle_duration_;
  }

  bool is_vacation_mode() const {
    return vacation_mode_;
  }

  PumpTrigger get_pump_trigger() const {
    return pump_trigger_;
  }

  // Parameters (configurable)
  float temp_rise_threshold_{1.5f};      // Minimum temperature rise to detect draw
  float return_rise_threshold_{5.0f};    // Target return temperature rise
  float disinfection_temp_threshold_{10.0f};  // Temp rise above baseline indicating disinfection cycle
  float min_return_temp_{30.0f};         // Minimum return temp to start pump (water already hot enough)
  float pump_flow_rate_{20.0f};          // Pump flow rate in liters per minute (L/min)
  uint32_t MIN_RUN_TIME = 30;            // Minimum pump run time (seconds)
  uint32_t MAX_RUN_TIME = 480;           // Maximum pump run time (seconds)
  uint32_t MINIMUM_DRAW_DURATION = 15000; // 15 seconds in milliseconds
  uint32_t USER_REQUEST_MAX_AGE = 1800;  // 30 minutes
  uint32_t DISINFECTION_COOLDOWN = 3600; // 1 hour cooldown before re-detecting disinfection (seconds)
  uint32_t ANTI_STAGNATION_INTERVAL = 172800; // 48 hours (2 days) in seconds
  uint32_t ANTI_STAGNATION_RUNTIME = 15;     // Anti-stagnation run time (seconds)
  uint8_t LEARN_INC = 40;                // Learning increment per detection
  uint8_t SCHEDULE_THRESHOLD = 120;      // Threshold to trigger scheduled run
  float DECAY = 0.98f;                   // Daily decay factor for learning matrix

  // Learning matrix [day_of_week][half_hour_slot]
  // Slots: 0=00:00-00:29, 1=00:30-00:59, 2=01:00-01:29, ..., 47=23:30-23:59
  uint8_t learn_[7][48] = {0};
  uint32_t last_decay_day_ = 0;

  // Water draw detection state (tracks sustained temperature RISE)
  float last_outlet_value_{NAN};
  uint32_t last_outlet_check_{0};
  uint32_t draw_detection_started_{0};   // Timestamp when potential draw started
  bool draw_detected_{false};            // Flag to prevent multiple triggers
  float initial_draw_temp_{NAN};         // Temperature when draw detection started
  float baseline_outlet_{NAN};           // Baseline outlet temp for disinfection detection
  time_t last_water_draw_time_{0};       // Timestamp of last detected water draw
  bool vacation_mode_{false};            // True when no water draw for 24h

  // Pump control state
  bool pump_running_{false};
  bool disinfection_mode_{false};        // Flag when disinfection cycle detected
  PumpTrigger pump_trigger_{PumpTrigger::NONE};  // What triggered the current pump run
  time_t last_disinfection_start_{0};    // Timestamp of last disinfection cycle start (prevents re-trigger)
  time_t last_anti_stagnation_run_{0};   // Timestamp of last anti-stagnation run
  float baseline_return_{NAN};
  uint32_t pump_start_{0};
  time_t last_run_epoch_ = 0;

  // Energy tracking for current/last pump cycle
  float energy_sum_{0.0f};               // Accumulated energy during current cycle (Wh)
  uint32_t energy_samples_{0};           // Number of samples taken
  uint32_t last_energy_calc_time_{0};    // Last time energy was calculated (millis)
  float last_cycle_energy_{0.0f};        // Energy consumed in last completed cycle (kWh)
  uint32_t last_cycle_duration_{0};      // Duration of last cycle (seconds)

  // UI state
  uint32_t yellow_led_on_until_{0};
  bool learning_enabled_{true};
  bool pump_enabled_{true};  // Master enable/disable for pump operation
  bool button_last_{false};
  uint32_t button_pressed_since_{0};

  // Scheduled trigger tracking (prevents re-triggering same 30-min slot)
  int last_scheduled_day_{-1};   // Last day when scheduled trigger fired
  int last_scheduled_slot_{-1};  // Last 30-min slot when scheduled trigger fired

  // Flash storage
  ESPPreferenceObject pref_;

  void setup() override;
  void loop() override;

  // Public control methods (callable from YAML)
  void manual_pump_on() {
    run_pump(PumpTrigger::MANUAL_WEBUI);
  }

  void manual_pump_off() {
    stop_pump("Manual stop (Web UI)");
  }

  void enable_pump();

  void disable_pump();

  // Save learning matrix to flash (callable from YAML button)
  void save_learning_matrix();

  void run_pump(PumpTrigger trigger = PumpTrigger::MANUAL_BUTTON);
  void stop_pump(const char *reason);

 protected:
  void detect_water_draw();              // Detects temperature RISE (correct for this system)
                                         // IMPORTANT: Suspended while pump_running_ to prevent false learning
  void reset_water_draw_detection_();    // Resets draw detection state
  void detect_disinfection_cycle_();     // Detects boiler disinfection by monitoring outlet temp
  void check_vacation_mode_();           // Check if entering/exiting vacation mode
  void check_anti_stagnation_();         // Check if anti-stagnation run is needed
  void handle_user_request();
  void learn_now();
  void decay_table();
  void propagate_first_day_();           // Copy first learned day to all other days
  void save_learning_matrix_();          // Save learning matrix to flash
  void load_learning_matrix_();          // Load learning matrix from flash
  uint32_t calculate_checksum_();        // Calculate checksum for validation
  void reset_learning_matrix_();         // Reset learning matrix to zero (10+ sec button press)
  void check_schedule();
  void pump_control();
  void handle_button();
  void toggle_learning();
  void update_leds();
  void log_learning_matrix_();

};

}  // namespace esphome_hotcirc
}  // namespace esphome
