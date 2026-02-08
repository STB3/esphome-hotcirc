#include "esphome_hotcirc.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath> 

// provide millis() wrapper for ESP-IDF
static inline uint32_t millis() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// provide delay() wrapper for ESP-IDF
static inline void delay(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

namespace esphome {
namespace esphome_hotcirc {

static const char *const TAG = "hotwater";

void HotWaterController::setup() {
  // Initialize flash storage preferences
  pref_ = global_preferences->make_preference<LearnMatrixData>(fnv1_hash("hwc_learn"));
  
  // Try to load learning matrix from flash
  load_learning_matrix_();
  
  ESP_LOGI(TAG, "Setup complete (dT outlet=%.1f°C, dT return=%.1f°C)",
           temp_rise_threshold_, return_rise_threshold_);
  
  if (led_green_) {
    led_green_->set_state(false);
    ESP_LOGI(TAG, "Green LED initialized to OFF");
  } else {
    ESP_LOGW(TAG, "Green LED not configured!");
  }
  
  if (led_yellow_) {
    led_yellow_->set_state(false);
    ESP_LOGI(TAG, "Yellow LED initialized to OFF");
  } else {
    ESP_LOGW(TAG, "Yellow LED not configured!");
  }
}

void HotWaterController::loop() {
  if (!clock_ || !clock_->now().is_valid()) {
    pump_control();
    handle_button();
    update_leds();
    return;
  }

  // Initialize last_decay_day_ on first valid clock reading to prevent immediate decay on boot
  static bool decay_day_initialized = false;
  if (!decay_day_initialized) {
    last_decay_day_ = clock_->now().day_of_year;
    decay_day_initialized = true;
    ESP_LOGI(TAG, "Initialized last_decay_day_ to current day: %d (prevents decay on boot)", last_decay_day_);
  }

  // Check for vacation mode (24h with no water draw)
  check_vacation_mode_();
  
  // ALWAYS check anti-stagnation (runs even when pump disabled or in vacation mode)
  check_anti_stagnation_();
  
  // ALWAYS detect water draws (even in vacation mode - this is how we exit vacation mode)
  detect_water_draw();
  
  // Skip learning, decay, and automatic pump operations in vacation mode
  if (!vacation_mode_) {
    // Normal mode: run learning and decay
    decay_table();
    
    // Only run automatic pump operations if enabled
    if (pump_enabled_) {
      detect_disinfection_cycle_();
      check_schedule();
    }
  }
  
  pump_control();
  handle_button();
  update_leds();
  
  static uint32_t last_matrix_log = 0;
  uint32_t now_s = millis() / 1000;
  if (now_s - last_matrix_log >= 60) {
    log_learning_matrix_();
    last_matrix_log = now_s;
  }
}

void HotWaterController::log_learning_matrix_() {
  ESP_LOGI("hotwater", "Learning matrix (D0=Mon, D1=Tue, D2=Wed, D3=Thu, D4=Fri, D5=Sat, D6=Sun)");
  ESP_LOGI("hotwater", "30-min slots: AM (0-23 = 00:00-11:59), PM (24-47 = 12:00-23:59)");
  char line[250];
  const char* day_names[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
  
  for (int d = 0; d < 7; d++) {
    // AM slots (0-23 = 00:00-11:59)
    int pos = snprintf(line, sizeof(line), "%s-AM:", day_names[d]);
    for (int slot = 0; slot < 24; slot++) {
      pos += snprintf(line + pos, sizeof(line) - pos, " %3d", learn_[d][slot]);
    }
    ESP_LOGI("hotwater", "%s", line);
    
    // PM slots (24-47 = 12:00-23:59)
    pos = snprintf(line, sizeof(line), "%s-PM:", day_names[d]);
    for (int slot = 24; slot < 48; slot++) {
      pos += snprintf(line + pos, sizeof(line) - pos, " %3d", learn_[d][slot]);
    }
    ESP_LOGI("hotwater", "%s", line);
  }
}

/**
 * Detects water draw by monitoring RISING outlet temperature
 * Physics:
 * 1. 40cm pipe from tank top to sensor cools down when idle
 * 2. When tap opens, fresh hot water from stratified tank top flows through
 * 3. Sensor sees temperature RISE as hot water arrives
 * 4. Expected rate: 0.015 to 0.04 °C/s (measured by user)
 * 5. Must persist for 15+ seconds with accumulated rise >= threshold (1.5°C default)
 * 6. Logic: Detect sustained rate >= 0.015°C/s, accumulate over 15s to reach threshold
 */
void HotWaterController::detect_water_draw() {
  if (!outlet_) return;

  // CRITICAL: Don't detect draws while pump is running!
  // Pump operation causes temperature rise that would be falsely learned
  if (pump_running_) {
    reset_water_draw_detection_();
    return;
  }
  
  // ANTI-STAGNATION LOCKOUT: After anti-stagnation completes, enforce 30-minute lockout
  // This prevents false water draw detection during temperature settling
  // and gives clear separation between maintenance and normal operation
  if (last_anti_stagnation_run_ != 0 && clock_ && clock_->now().is_valid()) {
    time_t now = clock_->now().timestamp;
    time_t time_since_anti_stag = now - last_anti_stagnation_run_;
    if (time_since_anti_stag < 1800) {  // 30 minutes = 1800 seconds
      // Still in lockout period - suppress water draw detection
      reset_water_draw_detection_();
      return;
    }
  }

  uint32_t now_ms = millis();
  float t_now = outlet_->state;

  if (std::isnan(t_now)) {
    ESP_LOGW("hotwater", "Outlet temperature invalid (NaN)");
    reset_water_draw_detection_();
    return;
  }

  // Initialize first-time reference
  if (std::isnan(this->last_outlet_value_)) {
    this->last_outlet_value_ = t_now;
    this->last_outlet_check_ = now_ms;
    this->draw_detection_started_ = 0;
    this->draw_detected_ = false;
    ESP_LOGD("hotwater", "Initialized outlet tracking: %.2f°C", t_now);
    return;
  }

  // Check every second
  if (now_ms - this->last_outlet_check_ < 1000) return;

  float delta = t_now - this->last_outlet_value_;  // Change since last check
  uint32_t elapsed_ms = now_ms - this->last_outlet_check_;
  float rate = delta / (elapsed_ms / 1000.0f);  // °C/s

  ESP_LOGV("hotwater", "Outlet: %.2f°C, delta=%.3f°C, elapsed=%.1fs, rate=%.3f°C/s", 
           t_now, delta, elapsed_ms / 1000.0f, rate);

  // Detect temperature RISE indicating water draw
  // Requirements (tuned to avoid false triggers on sensor noise):
  // 1. Rate >= 0.010°C/s (minimum realistic rise rate for this system)
  // 2. Delta > 0.03°C (minimum change to filter sensor noise)
  // Note: Sensor resolution is ~0.0625°C, so 0.03°C provides good noise margin
  if (rate >= 0.010f && delta > 0.03f) {  // Rising at minimum rate with meaningful delta
    if (this->draw_detection_started_ == 0) {
      // Start tracking potential draw - ONLY LOG THIS ONCE
      this->draw_detection_started_ = now_ms;
      this->initial_draw_temp_ = t_now;  // Use current temp as baseline
      ESP_LOGI("hotwater", "Potential water draw started (T=%.2f°C, delta=%.3f°C, rate=%.3f°C/s)", 
               t_now, delta, rate);
    }
    
    // Calculate total accumulated rise since detection started
    float total_rise = t_now - this->initial_draw_temp_;
    uint32_t draw_duration_ms = now_ms - this->draw_detection_started_;
    
    // Check if draw has persisted for 15 seconds AND accumulated enough rise
    if (draw_duration_ms >= MINIMUM_DRAW_DURATION && !this->draw_detected_) {
      if (total_rise >= temp_rise_threshold_) {
        float avg_rate = total_rise / (draw_duration_ms / 1000.0f);
        ESP_LOGI("hotwater",
                 "[WATER DRAW] Water draw CONFIRMED! Duration=%.1fs, Total rise=%.2f°C, avg rate=%.3f°C/s",
                 draw_duration_ms / 1000.0f, total_rise, avg_rate);
        this->draw_detected_ = true;
        this->handle_user_request();
      } else {
        ESP_LOGD("hotwater", "Duration OK (%.1fs) but total rise insufficient (%.2f°C < %.2f°C threshold)", 
                 draw_duration_ms / 1000.0f, total_rise, temp_rise_threshold_);
      }
    } else if (draw_duration_ms < MINIMUM_DRAW_DURATION) {
      ESP_LOGV("hotwater", "Draw in progress... %.1fs elapsed, rise so far: %.2f°C", 
               draw_duration_ms / 1000.0f, total_rise);
    }
  } else {
    // Temperature not rising fast enough or falling
    // Only reset if we've been tracking AND the temperature is actually FALLING (not just paused)
    if (this->draw_detection_started_ != 0 && !this->draw_detected_) {
      uint32_t duration_ms = now_ms - this->draw_detection_started_;
      float total_rise = t_now - this->initial_draw_temp_;
      
      // Reset only if:
      // 1. Temperature is FALLING (negative rate), OR
      // 2. Total accumulated rise is negative (cooled below start point), OR  
      // 3. Been stuck without progress for 30+ seconds
      if (rate < -0.01f || total_rise < -0.1f || duration_ms > 30000) {
        ESP_LOGD("hotwater", "Draw detection reset (rate=%.3f°C/s, delta=%.3f°C, duration: %.1fs, total_rise: %.2f°C)", 
                 rate, delta, duration_ms / 1000.0f, total_rise);
        reset_water_draw_detection_();
      }
      // Otherwise, keep tracking - allow brief pauses in temperature rise
    } else if (this->draw_detected_ && rate < -0.01f) {
      // Draw ended - temperature dropping back down as pipe cools
      ESP_LOGD("hotwater", "Water draw ended (temp falling, rate=%.3f°C/s)", rate);
      reset_water_draw_detection_();
    }
  }

  // Update tracking values
  this->last_outlet_value_ = t_now;
  this->last_outlet_check_ = now_ms;
}

void HotWaterController::reset_water_draw_detection_() {
  this->draw_detection_started_ = 0;
  this->draw_detected_ = false;
  this->initial_draw_temp_ = NAN;
}

/**
 * Check for vacation mode: enters when no water draw detected for 24 hours
 * In vacation mode, all activities except water draw detection are suspended:
 * - No learning
 * - No decay
 * - No scheduled pump runs
 * - No disinfection detection
 * Returns to normal mode on first water draw detection
 */
void HotWaterController::check_vacation_mode_() {
  if (!clock_ || !clock_->now().is_valid()) return;
  
  time_t now = clock_->now().timestamp;
  
  // If we've never detected a water draw, initialize timestamp to now
  if (last_water_draw_time_ == 0) {
    last_water_draw_time_ = now;
    return;
  }
  
  // Calculate time since last water draw
  time_t time_since_draw = now - last_water_draw_time_;
  
  // Enter vacation mode if no water draw for 24 hours (86400 seconds)
  if (!vacation_mode_ && time_since_draw >= 86400) {
    vacation_mode_ = true;
    ESP_LOGW(TAG, "===========================================");
    ESP_LOGW(TAG, "ENTERING VACATION MODE");
    ESP_LOGW(TAG, "No water draw detected for 24 hours");
    ESP_LOGW(TAG, "All learning, decay, and automatic pump operations suspended");
    ESP_LOGW(TAG, "Will resume on first water draw");
    ESP_LOGW(TAG, "===========================================");
  }
  
  // Log periodic status when in vacation mode
  static time_t last_vacation_log = 0;
  if (vacation_mode_ && (now - last_vacation_log >= 3600)) {  // Log every hour
    int hours_since_draw = time_since_draw / 3600;
    ESP_LOGI(TAG, "[VACATION MODE] %d hours since last water draw", hours_since_draw);
    last_vacation_log = now;
  }
}

/**
 * Anti-stagnation check: Prevents pump from seizing due to prolonged inactivity
 * 
 * IMPROVED FIXED SCHEDULE APPROACH:
 * - Runs every Sunday at 3:00 AM (configurable)
 * - Only when pump is disabled OR in vacation mode
 * - Locks out ALL other pump operations for 30 minutes after completion
 * - More predictable than interval-based approach
 * - Avoids conflicts with scheduled runs or water draw detection
 * 
 * This maintenance cycle runs REGARDLESS of pump enabled state to protect hardware
 */
void HotWaterController::check_anti_stagnation_() {
  if (!clock_ || !clock_->now().is_valid()) return;
  if (pump_running_) return;  // Don't interrupt an already running pump
  
  time_t now = clock_->now().timestamp;
  auto n = clock_->now();
  
  // Anti-stagnation configuration (can be made configurable via YAML)
  const int ANTI_STAG_DAY_OF_WEEK = 6;  // 6 = Sunday (0=Mon, 1=Tue, ..., 6=Sun)
  const int ANTI_STAG_HOUR = 3;          // 3 AM
  const int ANTI_STAG_MINUTE_START = 0;  // Start at exactly 3:00 AM
  const int ANTI_STAG_MINUTE_END = 5;    // Stop checking after 3:05 AM (5-minute window)
  
  // Convert day of week to our internal representation
  int raw_dow = n.day_of_week;  // 1=Sunday, 2=Monday, ..., 7=Saturday
  int wd = (raw_dow == 1) ? 6 : (raw_dow - 2);  // Convert to 0=Mon...6=Sun
  if (wd < 0 || wd > 6) wd = 0;
  
  // Check if anti-stagnation is needed (pump disabled OR vacation mode)
  bool needs_anti_stagnation = !pump_enabled_ || vacation_mode_;
  
  // Check if we're in the anti-stagnation time window
  bool in_time_window = (wd == ANTI_STAG_DAY_OF_WEEK && 
                         n.hour == ANTI_STAG_HOUR && 
                         n.minute >= ANTI_STAG_MINUTE_START && 
                         n.minute < ANTI_STAG_MINUTE_END);
  
  // Initialize tracking on first run
  if (last_anti_stagnation_run_ == 0) {
    last_anti_stagnation_run_ = now;
    ESP_LOGI(TAG, "[ANTI-STAGNATION] Initialized - will run every Sunday at 03:00 AM when needed");
    return;
  }
  
  // Reset tracking flag if it's NOT the scheduled day/time
  // This allows anti-stagnation to run again next week
  static bool anti_stag_ran_this_week = false;
  if (wd != ANTI_STAG_DAY_OF_WEEK || n.hour != ANTI_STAG_HOUR) {
    anti_stag_ran_this_week = false;
  }
  
  // If system doesn't need anti-stagnation, do nothing
  if (!needs_anti_stagnation) {
    return;
  }
  
  // Check if we should run anti-stagnation
  if (in_time_window && !anti_stag_ran_this_week) {
    ESP_LOGW(TAG, "===========================================");
    ESP_LOGW(TAG, "[ANTI-STAGNATION] Running weekly maintenance");
    ESP_LOGW(TAG, "Scheduled: Sunday 03:00 AM");
    ESP_LOGW(TAG, "Reason: %s", !pump_enabled_ ? "Pump disabled" : "Vacation mode");
    ESP_LOGW(TAG, "Duration: %u seconds", ANTI_STAGNATION_RUNTIME);
    ESP_LOGW(TAG, "Lockout: 30 minutes after completion");
    ESP_LOGW(TAG, "===========================================");
    
    // Mark as completed for this week
    anti_stag_ran_this_week = true;
    last_anti_stagnation_run_ = now;
    
    // CRITICAL: Mark current time slot to prevent check_schedule() from triggering
    // Also set the lockout timestamp (30 minutes from now)
    int slot = n.hour * 2 + (n.minute >= 30 ? 1 : 0);
    if (slot < 0) slot = 0;
    if (slot > 47) slot = 47;
    last_scheduled_day_ = wd;
    last_scheduled_slot_ = slot;
    
    // The lockout is enforced in detect_water_draw() and check_schedule()
    // by checking (now - last_anti_stagnation_run_ < 1800)
    
    ESP_LOGI(TAG, "[ANTI-STAGNATION] Slot d=%d s=%d marked, lockout until %02d:%02d", 
             wd, slot, (n.hour + (n.minute + 30) / 60) % 24, (n.minute + 30) % 60);
    
    // Start pump in anti-stagnation mode (bypasses pump_enabled_ check)
    run_pump(PumpTrigger::ANTI_STAGNATION);
  } else if (needs_anti_stagnation && wd == ANTI_STAG_DAY_OF_WEEK) {
    // Log status on the scheduled day
    static int last_log_hour = -1;
    if (n.hour != last_log_hour && n.hour >= 0 && n.hour <= 6) {
      if (anti_stag_ran_this_week) {
        ESP_LOGI(TAG, "[ANTI-STAGNATION] Already completed this week");
      } else {
        int hours_until = ANTI_STAG_HOUR - n.hour;
        if (hours_until < 0) hours_until += 24;
        ESP_LOGI(TAG, "[ANTI-STAGNATION] Scheduled in %d hours (%s)", 
                 hours_until, !pump_enabled_ ? "pump disabled" : "vacation mode");
      }
      last_log_hour = n.hour;
    }
  }
}

/**
 * Detects boiler disinfection cycle by monitoring outlet temperature
 * Disinfection raises tank temp by ~10°C above normal operating temperature
 * When detected, triggers pump to run maximum time to disinfect entire circulation system
 * 
 * BASELINE STRATEGY: Capture outlet temperature when pump STOPS
 * - At pump stop, outlet sensor shows actual tank temperature (fresh hot water just arrived)
 * - During idle, 40cm pipe cools down, giving falsely low readings
 * - This ensures accurate baseline for disinfection detection
 */
void HotWaterController::detect_disinfection_cycle_() {
  if (!outlet_ || !clock_) return;
  
  float t_now = outlet_->state;
  if (std::isnan(t_now)) return;
  
  // Check if outlet temperature is significantly elevated (disinfection cycle)
  // Only check when pump is not running and we have a valid baseline
  if (!std::isnan(baseline_outlet_) && !pump_running_) {
    float temp_elevation = t_now - baseline_outlet_;
    
    if (temp_elevation >= disinfection_temp_threshold_) {
      // Check cooldown period to prevent re-triggering same disinfection cycle
      time_t now_epoch = clock_->now().timestamp;
      time_t time_since_last_disinfection = now_epoch - last_disinfection_start_;
      
      if (last_disinfection_start_ == 0 || time_since_last_disinfection >= DISINFECTION_COOLDOWN) {
        ESP_LOGI(TAG, "[DISINFECTION] DISINFECTION CYCLE DETECTED! Outlet=%.1f°C, Baseline=%.1f°C, Elevation=%.1f°C",
                 t_now, baseline_outlet_, temp_elevation);
        
        // Record timestamp to prevent re-detection
        last_disinfection_start_ = now_epoch;
        
        // Start pump in disinfection mode
        disinfection_mode_ = true;
        run_pump(PumpTrigger::DISINFECTION);
      } else {
        // Still in cooldown period, skip this detection
        ESP_LOGD(TAG, "[DISINFECTION] High temp detected (%.1f°C) but in cooldown period (%ld seconds since last, need %ld)",
                 t_now, time_since_last_disinfection, (long)DISINFECTION_COOLDOWN);
      }
    }
  }
}

void HotWaterController::handle_user_request() {
  if (learning_enabled_)
    learn_now();

  yellow_led_on_until_ = millis() + 5000;

  time_t now_epoch = clock_->now().timestamp;
  if (!pump_running_ &&
      (last_run_epoch_ == 0 || (now_epoch - last_run_epoch_) > USER_REQUEST_MAX_AGE)) {
    run_pump(PumpTrigger::WATER_DRAW);
  } else if (pump_running_) {
    ESP_LOGD(TAG, "Pump already running, request acknowledged");
  } else {
    ESP_LOGD(TAG, "Recent pump run detected, skipping (age=%lds)", 
             (long)(now_epoch - last_run_epoch_));
  }
}

void HotWaterController::learn_now() {
  auto n = clock_->now();
  
  // Update last water draw timestamp
  last_water_draw_time_ = n.timestamp;
  
  // Exit vacation mode if we were in it
  if (vacation_mode_) {
    vacation_mode_ = false;
    ESP_LOGW(TAG, "===========================================");
    ESP_LOGW(TAG, "EXITING VACATION MODE");
    ESP_LOGW(TAG, "Water draw detected - resuming normal operation");
    ESP_LOGW(TAG, "===========================================");
  }
  
  // Skip learning if disabled
  if (!learning_enabled_) {
    return;
  }
  
  int raw_dow = n.day_of_week;  // 1=Sunday, 2=Monday, ..., 7=Saturday (standard C library)
  
  // Convert to array index: 0=Mon, 1=Tue, ..., 5=Sat, 6=Sun
  int wd;
  if (raw_dow == 1) {
    wd = 6;  // Sunday -> index 6
  } else {
    wd = raw_dow - 2;  // Monday(2)->0, Tuesday(3)->1, ..., Saturday(7)->5
  }
  
  // Safety bounds check
  if (wd < 0 || wd > 6) wd = 0;
  
  // Calculate 30-minute slot
  int hr = n.hour;
  int min = n.minute;
  int slot = hr * 2 + (min >= 30 ? 1 : 0);  // 0-47
  
  if (slot < 0) slot = 0;
  if (slot > 47) slot = 47;

  uint16_t val = (uint16_t) learn_[wd][slot] + (uint16_t) LEARN_INC;
  if (val > 255) val = 255;
  learn_[wd][slot] = (uint8_t) val;

  const char* day_names[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
  ESP_LOGI(TAG, "Learned: %s (raw_dow=%d, idx=%d) slot=%d (time %02d:%02d) -> val=%u", 
           day_names[wd], raw_dow, wd, slot, hr, min, learn_[wd][slot]);
}

void HotWaterController::decay_table() {
  auto n = clock_->now();
  if (n.day_of_year == last_decay_day_) return;
  
  int prev_day_of_year = last_decay_day_;
  last_decay_day_ = n.day_of_year;

  // Apply daily decay to all cells
  for (int d = 0; d < 7; d++)
    for (int slot = 0; slot < 48; slot++)
      learn_[d][slot] = (uint8_t) std::round(learn_[d][slot] * DECAY);

  ESP_LOGI(TAG, "Learning matrix decayed");
  
  // Save updated matrix to flash at end of each day
  save_learning_matrix_();
}

void HotWaterController::check_schedule() {
  // ANTI-STAGNATION LOCKOUT: Don't run scheduled operations for 30 minutes after anti-stagnation
  // This ensures clean separation between maintenance and normal operation
  if (last_anti_stagnation_run_ != 0 && clock_ && clock_->now().is_valid()) {
    time_t now_epoch = clock_->now().timestamp;
    time_t time_since_anti_stag = now_epoch - last_anti_stagnation_run_;
    if (time_since_anti_stag < 1800) {  // 30 minutes = 1800 seconds
      // Still in lockout period - no scheduled runs
      return;
    }
  }
  
  static uint32_t last_check_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_check_ms < 30000)  // Check every 30 seconds
    return;
  last_check_ms = now_ms;

  auto n = clock_->now();
  int raw_dow = n.day_of_week;  // 1=Sunday, 2=Monday, ..., 7=Saturday
  
  // Convert to array index: 0=Mon, 1=Tue, ..., 5=Sat, 6=Sun
  int wd;
  if (raw_dow == 1) {
    wd = 6;  // Sunday -> index 6
  } else {
    wd = raw_dow - 2;  // Monday(2)->0, Tuesday(3)->1, ..., Saturday(7)->5
  }
  
  if (wd < 0 || wd > 6) wd = 0;
  
  // Calculate 30-minute slot
  int hr = n.hour;
  int min = n.minute;
  int slot = hr * 2 + (min >= 30 ? 1 : 0);  // 0-47
  
  if (slot < 0) slot = 0;
  if (slot > 47) slot = 47;

  if (learn_[wd][slot] >= SCHEDULE_THRESHOLD) {
    // Prevent re-triggering during the same 30-min slot
    // Only trigger if this is a different day/slot combination than last time
    if (last_scheduled_day_ != wd || last_scheduled_slot_ != slot) {
      ESP_LOGI(TAG, "Scheduled preheat triggered for d=%d slot=%d (time %02d:%02d, val=%u)", 
               wd, slot, hr, min, learn_[wd][slot]);
      
      // Record this day/slot to prevent re-trigger (do this BEFORE checking pump state)
      last_scheduled_day_ = wd;
      last_scheduled_slot_ = slot;
      
      if (!pump_running_) {
        run_pump(PumpTrigger::SCHEDULED);
      } else {
        ESP_LOGD(TAG, "Pump already running, scheduled trigger recorded but not started");
      }
    } else {
      ESP_LOGD(TAG, "Schedule threshold met for d=%d slot=%d (time %02d:%02d, val=%u) but already triggered this slot",
               wd, slot, hr, min, learn_[wd][slot]);
    }
  }
}

void HotWaterController::enable_pump() {
  pump_enabled_ = true;
  ESP_LOGI(TAG, "Pump ENABLED - automatic operation resumed");
}

void HotWaterController::disable_pump() {
  pump_enabled_ = false;
  ESP_LOGI(TAG, "Pump DISABLED - all automatic operation suspended (learning preserved)");
  // If pump is currently running, stop it
  if (pump_running_) {
    stop_pump("Pump disabled");
  }
}

void HotWaterController::save_learning_matrix() {
  ESP_LOGI(TAG, "Manual save requested via Web UI");
  save_learning_matrix_();
}

void HotWaterController::run_pump(PumpTrigger trigger) {
  if (!pump_) return;
  
  // Check if pump is globally disabled (except for anti-stagnation which bypasses this)
  if (!pump_enabled_ && trigger != PumpTrigger::ANTI_STAGNATION) {
    ESP_LOGD(TAG, "Pump start blocked: pump is disabled");
    return;
  }
  
  if (!ret_ || std::isnan(ret_->state)) {
    ESP_LOGW(TAG, "Cannot start pump: return sensor invalid");
    return;
  }

  // Check if return temperature is already hot enough (water at taps is warm)
  // Skip this check for disinfection mode and anti-stagnation (must run regardless of temperature)
  if (trigger != PumpTrigger::ANTI_STAGNATION && 
      !disinfection_mode_ && 
      ret_->state >= min_return_temp_) {
    ESP_LOGI(TAG, "Pump start skipped: return temperature already hot enough (%.1f°C >= %.1f°C threshold)",
             ret_->state, min_return_temp_);
    return;
  }

  // Store trigger reason
  pump_trigger_ = trigger;

  baseline_return_ = ret_->state;
  pump_->turn_on();
  pump_running_ = true;
  pump_start_ = millis() / 1000;
  
  // Initialize energy tracking
  energy_sum_ = 0.0f;
  energy_samples_ = 0;
  last_energy_calc_time_ = millis();  // Initialize to current time
  
  if (led_green_) {
    ESP_LOGI(TAG, "Setting Green LED to ON");
    led_green_->set_state(true);
  } else {
    ESP_LOGW(TAG, "led_green_ is NULL - cannot control LED!");
  }
  
  // Log pump start with trigger reason
  const char* trigger_str = "Unknown";
  switch (trigger) {
    case PumpTrigger::MANUAL_BUTTON: trigger_str = "Manual Button"; break;
    case PumpTrigger::MANUAL_WEBUI: trigger_str = "Web UI"; break;
    case PumpTrigger::WATER_DRAW: trigger_str = "Water Draw"; break;
    case PumpTrigger::SCHEDULED: trigger_str = "Schedule"; break;
    case PumpTrigger::DISINFECTION: trigger_str = "Disinfection"; break;
    case PumpTrigger::ANTI_STAGNATION: trigger_str = "Anti-Stagnation"; break;
    default: break;
  }
  
  if (disinfection_mode_) {
    ESP_LOGI(TAG, "Pump ON - DISINFECTION MODE (trigger: %s, baseline return=%.2f°C, will run max time)", 
             trigger_str, baseline_return_);
  } else {
    ESP_LOGI(TAG, "Pump ON (trigger: %s, baseline return=%.2f°C)", trigger_str, baseline_return_);
  }
  
  // Reset draw detection when pump starts to avoid false triggers
  reset_water_draw_detection_();
}

void HotWaterController::pump_control() {
  if (!pump_running_) return;
  
  uint32_t elapsed = (millis() / 1000) - pump_start_;
  
  // Calculate energy transferred using actual time deltas
  if (outlet_ && ret_ && !std::isnan(outlet_->state) && !std::isnan(ret_->state)) {
    uint32_t now_ms = millis();
    uint32_t dt_ms = now_ms - last_energy_calc_time_;
    
    // Only calculate if at least 50ms has passed (avoid division by very small numbers)
    if (dt_ms >= 50) {
      // Temperature difference (outlet should be hotter than return)
      float delta_t = outlet_->state - ret_->state;
      
      if (delta_t > 0) {  // Only count positive temperature difference
        // Energy calculation:
        // Power (W) = Flow rate (L/s) × ΔT (°C) × Specific heat capacity (J/(L·°C))
        // Energy (Wh) = Power (W) × time (h)
        //
        // Flow rate in L/s = pump_flow_rate_ / 60
        // Specific heat of water = 4186 J/(L·°C)
        // Time in hours = dt_ms / 3600000
        
        float flow_rate_ls = pump_flow_rate_ / 60.0f;  // Convert L/min to L/s
        float power_w = flow_rate_ls * delta_t * 4186.0f;  // Power in Watts
        float dt_hours = dt_ms / 3600000.0f;  // Time interval in hours
        float energy_wh = power_w * dt_hours;  // Energy in Wh for this time interval
        
        energy_sum_ += energy_wh;
        energy_samples_++;
      }
      
      last_energy_calc_time_ = now_ms;  // Update last calculation time
    }
  }
  
  // CRITICAL SAFETY: Always enforce maximum runtime regardless of sensor state
  if (elapsed >= MAX_RUN_TIME) {
    stop_pump("Safety timeout");
    return;
  }
  
  // Anti-stagnation mode: run for fixed short duration (15 seconds)
  if (pump_trigger_ == PumpTrigger::ANTI_STAGNATION) {
    if (elapsed >= ANTI_STAGNATION_RUNTIME) {
      stop_pump("Anti-stagnation complete");
    }
    return;  // Skip temperature checks for anti-stagnation
  }
  
  // In disinfection mode, always run for maximum time to ensure full system disinfection
  if (disinfection_mode_) {
    // Just wait for MAX_RUN_TIME, skip temperature checks
    return;
  }
  
  // Normal temperature-based control (only if sensor valid)
  if (!ret_) return;
  float now_ret = ret_->state;
  if (std::isnan(now_ret)) return;

  // Check if target temperature reached (with 0.2°C tolerance)
  if (elapsed >= MIN_RUN_TIME &&
      now_ret >= baseline_return_ + return_rise_threshold_ - 0.2f) {
    stop_pump("Target reached");
    return;
  }
}

void HotWaterController::stop_pump(const char *reason) {
  if (!pump_) return;
  
  // Calculate and store energy for this cycle
  uint32_t elapsed = (millis() / 1000) - pump_start_;
  last_cycle_duration_ = elapsed;
  last_cycle_energy_ = energy_sum_ / 1000.0f;  // Convert Wh to kWh
  
  ESP_LOGI(TAG, "Pump cycle complete: duration=%us, energy=%.4f kWh (%u samples)", 
           last_cycle_duration_, last_cycle_energy_, energy_samples_);
  
  // CRITICAL: Update baseline outlet temperature using slow-moving average
  // Captured NOW while fresh hot water from tank is at sensor (before 40cm pipe cools)
  // This represents actual tank temperature and adapts gradually to boiler setpoint changes
  
  // Debug: Log the conditions
  ESP_LOGI(TAG, "stop_pump conditions: outlet_=%s, outlet_state=%.2f, isnan=%d, disinfection=%d",
           outlet_ ? "valid" : "NULL",
           outlet_ ? outlet_->state : -999.0f,
           outlet_ ? std::isnan(outlet_->state) : -1,
           disinfection_mode_);
  
  if (outlet_ && !std::isnan(outlet_->state) && !disinfection_mode_) {
    float current_outlet = outlet_->state;
    
    if (std::isnan(baseline_outlet_)) {
      // First time initialization
      baseline_outlet_ = current_outlet;
      ESP_LOGI(TAG, "Baseline outlet temperature initialized: %.1f°C", baseline_outlet_);
    } else {
      // Slow-moving average: 90% old baseline + 10% new reading
      float old_baseline = baseline_outlet_;
      baseline_outlet_ = baseline_outlet_ * 0.9f + current_outlet * 0.1f;
      ESP_LOGI(TAG, "Baseline outlet temperature updated: %.1f°C -> %.1f°C (reading: %.1f°C)", 
               old_baseline, baseline_outlet_, current_outlet);
    }
  } else {
    ESP_LOGW(TAG, "Baseline update SKIPPED - check conditions above");
  }
  
  pump_->turn_off();
  pump_running_ = false;
  pump_trigger_ = PumpTrigger::NONE;  // Reset trigger
  last_run_epoch_ = clock_ && clock_->now().is_valid() ? clock_->now().timestamp : 0;
  
  if (led_green_) {
    ESP_LOGI(TAG, "Setting Green LED to OFF");
    led_green_->set_state(false);
  } else {
    ESP_LOGW(TAG, "led_green_ is NULL - cannot control LED!");
  }
  
  if (disinfection_mode_) {
    ESP_LOGI(TAG, "Pump OFF - Disinfection cycle complete (%s)", reason);
    disinfection_mode_ = false;  // Clear disinfection mode
  } else {
    ESP_LOGI(TAG, "Pump OFF (%s)", reason);
  }
  
  // Reset draw detection after pump stops
  reset_water_draw_detection_();
}

void HotWaterController::handle_button() {
  if (!button_) return;
  bool pressed = button_->state;
  uint32_t now = millis();

  if (pressed && !button_last_) {
    button_pressed_since_ = now;
  } else if (!pressed && button_last_) {
    uint32_t dur = now - button_pressed_since_;
    
    if (dur > 10000) {
      // VERY LONG PRESS (>10 seconds) - Reset learning matrix
      ESP_LOGW(TAG, "Button held for %u ms - RESETTING LEARNING MATRIX", dur);
      reset_learning_matrix_();
      
      // Visual feedback: flash yellow LED 6 times
      for (int i = 0; i < 6; i++) {
        if (led_yellow_) led_yellow_->set_state(i % 2 == 0);
        delay(200);
      }
    } else if (dur > 3000) {
      // LONG PRESS (>3 seconds) - Toggle learning
      toggle_learning();
    } else {
      // SHORT PRESS - Toggle pump
      if (pump_) {
        if (pump_running_)
          stop_pump("Manual stop");
        else
          run_pump(PumpTrigger::MANUAL_BUTTON);
      }
    }
  }

  button_last_ = pressed;
}

void HotWaterController::toggle_learning() {
  learning_enabled_ = !learning_enabled_;
  if (learning_enabled_) {
    ESP_LOGI(TAG, "Learning ENABLED");
    yellow_led_on_until_ = millis() + 2000;
  } else {
    ESP_LOGI(TAG, "Learning DISABLED");
    if (led_yellow_) led_yellow_->set_state(true);
    yellow_led_on_until_ = 0;
  }
}

void HotWaterController::save_learning_matrix_() {
  LearnMatrixData data;
  
  // Copy learning matrix
  for (int d = 0; d < 7; d++) {
    for (int slot = 0; slot < 48; slot++) {
      data.learn[d][slot] = learn_[d][slot];
    }
  }
  
  data.checksum = calculate_checksum_();
  
  if (pref_.save(&data)) {
    ESP_LOGI(TAG, "Learning matrix saved to flash (checksum: 0x%08X)", data.checksum);
  } else {
    ESP_LOGW(TAG, "Failed to save learning matrix to flash!");
  }
}

void HotWaterController::load_learning_matrix_() {
  LearnMatrixData data;
  
  if (!pref_.load(&data)) {
    ESP_LOGI(TAG, "No saved learning matrix found - initializing with typical daily pattern");
    // Initialize with typical daily usage pattern
    // Weekday pattern (Mon-Fri)
    for (int d = 0; d < 5; d++) {
      // Morning shower: 6:00-8:00 AM (slots 12-15)
      learn_[d][12] = 80;  // 06:00-06:29
      learn_[d][13] = 120; // 06:30-06:59
      learn_[d][14] = 120; // 07:00-07:29
      learn_[d][15] = 100; // 07:30-07:59
      learn_[d][16] = 80;  // 08:00-08:29
      
      // Lunch cooking: 11:30-13:00 (slots 23-25)
      learn_[d][23] = 80;  // 11:30-11:59
      learn_[d][24] = 100; // 12:00-12:29
      learn_[d][25] = 80;  // 12:30-12:59
      
      // Coming home/dinner: 18:00-19:00 (slots 36-37)
      learn_[d][36] = 100; // 18:00-18:29
      learn_[d][37] = 100; // 18:30-18:59
      
      // Evening bath/shower: 21:00-22:00 (slots 42-43)
      learn_[d][42] = 100; // 21:00-21:29
      learn_[d][43] = 80;  // 21:30-21:59
    }
    
    // Weekend pattern (Sat-Sun) - slightly different timing
    for (int d = 5; d < 7; d++) {
      // Later morning: 8:00-10:00 AM (slots 16-19)
      learn_[d][16] = 80;  // 08:00-08:29
      learn_[d][17] = 100; // 08:30-08:59
      learn_[d][18] = 100; // 09:00-09:29
      learn_[d][19] = 80;  // 09:30-09:59
      
      // Lunch: 12:00-13:00 (slots 24-25)
      learn_[d][24] = 100; // 12:00-12:29
      learn_[d][25] = 80;  // 12:30-12:59
      
      // Dinner: 18:30-19:30 (slots 37-38)
      learn_[d][37] = 100; // 18:30-18:59
      learn_[d][38] = 80;  // 19:00-19:29
      
      // Evening: 21:00-22:00 (slots 42-43)
      learn_[d][42] = 100; // 21:00-21:29
      learn_[d][43] = 80;  // 21:30-21:59
    }
    
    ESP_LOGI(TAG, "Initialized learning matrix with typical daily pattern");
    log_learning_matrix_();
    return;
  }
  
  // Validate checksum
  uint32_t expected_checksum = 0;
  for (int d = 0; d < 7; d++) {
    for (int slot = 0; slot < 48; slot++) {
      expected_checksum += data.learn[d][slot];
    }
  }
  
  if (data.checksum != expected_checksum) {
    ESP_LOGW(TAG, "Learning matrix checksum mismatch (expected 0x%08X, got 0x%08X)",
             expected_checksum, data.checksum);
    ESP_LOGW(TAG, "This may indicate old 24-slot format - resetting to new 48-slot format with typical pattern");
    
    // Reset and initialize with typical pattern (same code as above)
    for (int d = 0; d < 5; d++) {
      learn_[d][12] = 80; learn_[d][13] = 120; learn_[d][14] = 120; learn_[d][15] = 100; learn_[d][16] = 80;
      learn_[d][23] = 80; learn_[d][24] = 100; learn_[d][25] = 80;
      learn_[d][36] = 100; learn_[d][37] = 100;
      learn_[d][42] = 100; learn_[d][43] = 80;
    }
    for (int d = 5; d < 7; d++) {
      learn_[d][16] = 80; learn_[d][17] = 100; learn_[d][18] = 100; learn_[d][19] = 80;
      learn_[d][24] = 100; learn_[d][25] = 80;
      learn_[d][37] = 100; learn_[d][38] = 80;
      learn_[d][42] = 100; learn_[d][43] = 80;
    }
    ESP_LOGI(TAG, "Reset complete - initialized with typical daily pattern");
    log_learning_matrix_();
    return;
  }
  
  // Restore learning matrix
  for (int d = 0; d < 7; d++) {
    for (int slot = 0; slot < 48; slot++) {
      learn_[d][slot] = data.learn[d][slot];
    }
  }
  
  ESP_LOGI(TAG, "Learning matrix loaded from flash (checksum: 0x%08X)", data.checksum);
  log_learning_matrix_();
}

uint32_t HotWaterController::calculate_checksum_() {
  uint32_t checksum = 0;
  for (int d = 0; d < 7; d++) {
    for (int slot = 0; slot < 48; slot++) {
      checksum += learn_[d][slot];
    }
  }
  return checksum;
}

void HotWaterController::reset_learning_matrix_() {
  ESP_LOGW(TAG, "===========================================");
  ESP_LOGW(TAG, "RESETTING LEARNING MATRIX");
  ESP_LOGW(TAG, "===========================================");
  
  // Clear the entire learning matrix first
  for (int d = 0; d < 7; d++) {
    for (int slot = 0; slot < 48; slot++) {
      learn_[d][slot] = 0;
    }
  }
  
  // Re-initialize with typical daily usage pattern
  // Weekday pattern (Mon-Fri)
  for (int d = 0; d < 5; d++) {
    // Morning shower: 6:00-8:00 AM (slots 12-15)
    learn_[d][12] = 80;  // 06:00-06:29
    learn_[d][13] = 120; // 06:30-06:59
    learn_[d][14] = 120; // 07:00-07:29
    learn_[d][15] = 100; // 07:30-07:59
    learn_[d][16] = 80;  // 08:00-08:29
    
    // Lunch cooking: 11:30-13:00 (slots 23-25)
    learn_[d][23] = 80;  // 11:30-11:59
    learn_[d][24] = 100; // 12:00-12:29
    learn_[d][25] = 80;  // 12:30-12:59
    
    // Coming home/dinner: 18:00-19:00 (slots 36-37)
    learn_[d][36] = 100; // 18:00-18:29
    learn_[d][37] = 100; // 18:30-18:59
    
    // Evening bath/shower: 21:00-22:00 (slots 42-43)
    learn_[d][42] = 100; // 21:00-21:29
    learn_[d][43] = 80;  // 21:30-21:59
  }
  
  // Weekend pattern (Sat-Sun) - slightly different timing
  for (int d = 5; d < 7; d++) {
    // Later morning: 8:00-10:00 AM (slots 16-19)
    learn_[d][16] = 80;  // 08:00-08:29
    learn_[d][17] = 100; // 08:30-08:59
    learn_[d][18] = 100; // 09:00-09:29
    learn_[d][19] = 80;  // 09:30-09:59
    
    // Lunch: 12:00-13:00 (slots 24-25)
    learn_[d][24] = 100; // 12:00-12:29
    learn_[d][25] = 80;  // 12:30-12:59
    
    // Dinner: 18:30-19:30 (slots 37-38)
    learn_[d][37] = 100; // 18:30-18:59
    learn_[d][38] = 80;  // 19:00-19:29
    
    // Evening: 21:00-22:00 (slots 42-43)
    learn_[d][42] = 100; // 21:00-21:29
    learn_[d][43] = 80;  // 21:30-21:59
  }
  
  last_decay_day_ = 0;
  
  // Save the reinitialized matrix to flash
  save_learning_matrix_();
  
  ESP_LOGI(TAG, "Learning matrix reset and reinitialized with typical daily pattern");
  ESP_LOGI(TAG, "System will adapt to actual user behavior over time");
  
  // Log the reinitialized matrix
  log_learning_matrix_();
}

void HotWaterController::update_leds() {
  if (!learning_enabled_) {
    if (led_yellow_) led_yellow_->set_state(true);
    return;
  }

  if (led_yellow_) {
    if (millis() < yellow_led_on_until_)
      led_yellow_->set_state(true);
    else
      led_yellow_->set_state(false);
  }
}

}  // namespace esphome_hotcirc
}  // namespace esphome
