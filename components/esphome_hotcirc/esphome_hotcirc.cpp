#include "esphome_hotcirc.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
// FIX #5: ESPHome already provides millis()/delay() for ESP-IDF via hal.h.
// The previous file-local static wrappers around esp_timer/vTaskDelay were
// shadowed by esphome::millis() anyway (unqualified lookup inside the
// esphome namespace) and have been removed.
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
namespace esphome_hotcirc {

// FIX #20: one component tag ("hotcirc") for all controller logs. The
// separate "learning" tag is kept ON PURPOSE so the verbose matrix dump can
// be raised to DEBUG independently in the YAML logger config.
static const char *const TAG = "hotcirc";

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

  // Drive water-draw detection from the outlet sensor's publish callback instead
  // of polling it in loop(). On this display node loop() is starved by LVGL /
  // heatmap / WiFi-roam stalls (200-420 ms gaps), which made the poll-based
  // first-difference detector alias against the sensor's own 1 s publish clock
  // and miss draws. The callback fires exactly once per published value, at its
  // true timestamp, so loop jitter no longer affects detection.
  if (outlet_) {
    outlet_->add_on_state_callback([this](float v) { this->on_outlet_sample_(v); });
    ESP_LOGI(TAG, "Water-draw detection bound to outlet sensor callback");
  } else {
    ESP_LOGW(TAG, "Outlet sensor not configured - water-draw detection disabled!");
  }
}

void HotWaterController::loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (last && now - last > 500)
    ESP_LOGW(TAG, "loop gap %u ms", now - last);
  last = now;

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

  // Water-draw detection runs from the outlet sensor publish callback
  // (on_outlet_sample_), registered in setup(). It is intentionally NOT called
  // here so it cannot be starved by loop() stalls on the display node.

  // Skip learning, decay, and automatic pump operations in vacation mode
  if (!vacation_mode_) {
    // Normal mode: run learning and decay
    decay_table();

    // Only run automatic pump operations if enabled
    if (pump_enabled_) {
      detect_disinfection_cycle_();
      check_schedule();
      check_thermal_stagnation_();
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
  ESP_LOGD("learning", "Learning matrix (D0=Mon, D1=Tue, D2=Wed, D3=Thu, D4=Fri, D5=Sat, D6=Sun)");
  ESP_LOGD("learning", "30-min slots: AM (0-23 = 00:00-11:59), PM (24-47 = 12:00-23:59)");
  char line[250];
  const char* day_names[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

  for (int d = 0; d < 7; d++) {
    // AM slots (0-23 = 00:00-11:59)
    int pos = snprintf(line, sizeof(line), "%s-AM:", day_names[d]);
    for (int slot = 0; slot < 24; slot++) {
      pos += snprintf(line + pos, sizeof(line) - pos, " %3d", learn_[d][slot]);
    }
    ESP_LOGD("learning", "%s", line);

    // PM slots (24-47 = 12:00-23:59)
    pos = snprintf(line, sizeof(line), "%s-PM:", day_names[d]);
    for (int slot = 24; slot < 48; slot++) {
      pos += snprintf(line + pos, sizeof(line) - pos, " %3d", learn_[d][slot]);
    }
    ESP_LOGD("learning", "%s", line);
  }
}

/**
 * Callback-driven water-draw detection.
 *
 * Physics:
 *  1. 40 cm pipe from tank top to sensor cools down when idle
 *  2. When a tap opens, fresh hot water from the stratified tank top flows through
 *  3. The sensor sees a temperature RISE as the hot water arrives
 *  4. Expected rate: ~0.015 to 0.04 °C/s (measured); gate is 0.010 °C/s to
 *     tolerate the sliding-window filter and slow draws
 *  5. Must persist for 15+ s with accumulated rise >= temp_rise_threshold_
 *     (1.0 °C default, see YAML)
 *
 * Registered in setup() via outlet_->add_on_state_callback(). It is invoked
 * exactly once per published outlet reading (~1 s, the dallas update_interval),
 * at the moment of publish, regardless of how badly loop() is stalled by the
 * display, the heatmap redraw, or WiFi roam scans. This removes the aliasing
 * between the old poll clock and the sensor's publish clock that caused draws
 * to be missed on the UEDX4646 node.
 *
 * Robustness properties (vs. the removed poll-based detector):
 *   - No internal 1 s throttle (the publish cadence IS the clock).
 *   - A NaN sample (transient one-wire CRC dropout) is ignored rather than
 *     tearing down an in-progress detection.
 *   - A single negative step (quantization dip on a slow, filtered rise) does
 *     not reset tracking; only real cumulative regression (total_rise <
 *     -0.1 °C) or a 30 s stall does. This keeps the 0.03 °C / 0.010 °C/s
 *     gates working on noisy slow rises.
 */
void HotWaterController::on_outlet_sample_(float t_now) {
  if (!outlet_) return;

  // Don't detect draws while the pump is running (pump itself raises outlet temp)
  if (pump_running_) { reset_water_draw_detection_(); return; }

  // Anti-stagnation lockout: suppress detection for 30 min after a stagnation run
  if (last_anti_stagnation_run_ != 0 && clock_ && clock_->now().is_valid()) {
    time_t now = clock_->now().timestamp;
    if (now - last_anti_stagnation_run_ < 1800) {
      reset_water_draw_detection_();
      return;
    }
  }

  uint32_t now_ms = millis();

  // Transient CRC dropout on the bit-banged bus: skip this sample, keep state.
  // Note: last_outlet_check_ is intentionally NOT updated, so the next valid
  // reading simply spans a longer (correctly normalized) interval.
  if (std::isnan(t_now)) return;

  // First valid reading: arm the reference
  if (std::isnan(this->last_outlet_value_)) {
    this->last_outlet_value_ = t_now;
    this->last_outlet_check_ = now_ms;
    this->draw_detection_started_ = 0;
    this->draw_detected_ = false;
    ESP_LOGD(TAG, "Initialized outlet tracking: %.2f°C", t_now);
    return;
  }

  uint32_t elapsed_ms = now_ms - this->last_outlet_check_;
  if (elapsed_ms < 250) return;  // guard against pathological bursts; publishes are ~1 s

  float delta = t_now - this->last_outlet_value_;
  float rate  = delta / (elapsed_ms / 1000.0f);  // °C/s

  ESP_LOGV(TAG, "Outlet: %.2f°C, delta=%.3f°C, elapsed=%.1fs, rate=%.3f°C/s",
           t_now, delta, elapsed_ms / 1000.0f, rate);

  if (rate >= 0.010f && delta > 0.03f) {
    if (this->draw_detection_started_ == 0) {
      this->draw_detection_started_ = now_ms;
      this->initial_draw_temp_      = t_now;
      this->draw_pending_           = true;
      ESP_LOGI(TAG, "Potential water draw started (T=%.2f°C, delta=%.3f°C, rate=%.3f°C/s)",
               t_now, delta, rate);
    }

    float total_rise = t_now - this->initial_draw_temp_;
    uint32_t draw_duration_ms = now_ms - this->draw_detection_started_;

    if (draw_duration_ms >= MINIMUM_DRAW_DURATION && !this->draw_detected_) {
      if (total_rise >= temp_rise_threshold_) {
        float avg_rate = total_rise / (draw_duration_ms / 1000.0f);
        ESP_LOGI(TAG,
                 "[WATER DRAW] Water draw CONFIRMED! Duration=%.1fs, Total rise=%.2f°C, avg rate=%.3f°C/s",
                 draw_duration_ms / 1000.0f, total_rise, avg_rate);
        this->draw_detected_ = true;
        this->handle_user_request();
      } else {
        ESP_LOGD(TAG, "Duration OK (%.1fs) but total rise insufficient (%.2f°C < %.2f°C threshold)",
                 draw_duration_ms / 1000.0f, total_rise, temp_rise_threshold_);
      }
    } else if (draw_duration_ms < MINIMUM_DRAW_DURATION) {
      ESP_LOGV(TAG, "Draw in progress... %.1fs elapsed, rise so far: %.2f°C",
               draw_duration_ms / 1000.0f, total_rise);
    }
  } else {
    // Rise paused or reversed.
    if (this->draw_detection_started_ != 0 && !this->draw_detected_) {
      uint32_t duration_ms = now_ms - this->draw_detection_started_;
      float total_rise = t_now - this->initial_draw_temp_;
      // ROBUSTNESS: do NOT reset on a single negative step. Only give up on real
      // cumulative regression or a long stall without confirmation.
      if (total_rise < -0.1f || duration_ms > 30000) {
        ESP_LOGD(TAG, "Draw detection reset (rate=%.3f°C/s, delta=%.3f°C, duration: %.1fs, total_rise: %.2f°C)",
                 rate, delta, duration_ms / 1000.0f, total_rise);
        reset_water_draw_detection_();
      }
      // Otherwise keep tracking through brief pauses in the rise.
    } else if (this->draw_detected_ && rate < -0.01f) {
      ESP_LOGD(TAG, "Water draw ended (temp falling, rate=%.3f°C/s)", rate);
      reset_water_draw_detection_();
    }
  }

  this->last_outlet_value_ = t_now;
  this->last_outlet_check_ = now_ms;
}

void HotWaterController::reset_water_draw_detection_() {
  this->draw_detection_started_ = 0;
  this->draw_detected_ = false;
  this->initial_draw_temp_ = NAN;
  this->draw_pending_ = false;
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
 * FIXED WEEKLY SCHEDULE:
 * - Runs every Sunday at 3:00 AM (hardcoded below; the former
 *   anti_stagnation_interval YAML option is deprecated and unused)
 * - Only when pump is disabled OR in vacation mode
 * - Locks out ALL other pump operations for 30 minutes after completion
 * - More predictable than an interval-based approach
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
    // Backdate the reference so the 30-minute post-run lockout is NOT active at
    // boot. This is only a scheduler reference point, not an actual anti-stagnation
    // run - setting it to `now` previously suppressed water-draw detection AND
    // scheduled runs for 30 minutes after every boot (see the < 1800 lockout
    // checks in on_outlet_sample_() and check_schedule()).
    last_anti_stagnation_run_ = (now > 3600) ? (now - 3600) : 1;
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

    // Mark the current time slot so check_schedule() cannot fire in the same
    // slot right after the 30-minute lockout expires.
    int slot = n.hour * 2 + (n.minute >= 30 ? 1 : 0);
    if (slot < 0) slot = 0;
    if (slot > 47) slot = 47;
    last_scheduled_day_ = wd;
    last_scheduled_slot_ = slot;

    // The lockout is enforced in on_outlet_sample_() and check_schedule()
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
 * Thermal stagnation flush: detects summer heat-soak in return pipe
 *
 * Problem: In summer, when the boiler only heats DHW, the return pipe picks up
 * heat conductively from the boiler. The outlet-return delta shrinks or goes
 * negative, so pump_control() never sees the expected return temperature RISE
 * and the pump never starts for a normal request.
 *
 * Solution: If (outlet - return) < thermal_stagnation_delta_ for 10 consecutive
 * seconds, run the pump for THERMAL_STAGNATION_RUNTIME seconds to flush the
 * stagnant hot water out of the return pipe.
 *
 * Guards:
 *  - Pump must not already be running
 *  - pump_enabled_ must be true (respected, unlike anti-stagnation)
 *  - 30-minute cooldown after each flush run
 *  - Condition must persist for 10 s to ignore transient sensor noise
 */
void HotWaterController::check_thermal_stagnation_() {
  if (pump_running_) {
    thermal_stagnation_started_ = 0;  // Reset pending state if pump starts for other reason
    return;
  }

  if (!clock_ || !clock_->now().is_valid()) return;

  // Cooldown: don't re-trigger within THERMAL_STAGNATION_COOLDOWN seconds
  if (last_thermal_stagnation_run_ != 0) {
    time_t now_epoch = clock_->now().timestamp;
    if ((now_epoch - last_thermal_stagnation_run_) < (time_t)THERMAL_STAGNATION_COOLDOWN)
      return;
  }

  float outlet = outlet_ ? outlet_->state : NAN;
  float ret    = ret_    ? ret_->state    : NAN;
  if (std::isnan(outlet) || std::isnan(ret)) return;

  float delta = outlet - ret;  // Positive = normal, near-zero/negative = heat soak

  // Only act when return pipe is actually hot – filters out the normal overnight
  // cool-down where both sensors converge at low temperatures
  if (ret < thermal_stagnation_min_return_) {
    if (thermal_stagnation_started_ != 0) {
      ESP_LOGD(TAG, "[THERMAL-STAGNATION] Return too cold (%.1f°C < %.1f°C min), timer reset",
               ret, thermal_stagnation_min_return_);
      thermal_stagnation_started_ = 0;
    }
    return;
  }

  if (delta < thermal_stagnation_delta_) {
    uint32_t ms_now = millis();
    if (thermal_stagnation_started_ == 0) {
      // Condition just appeared – start timing
      thermal_stagnation_started_ = ms_now;
      ESP_LOGD(TAG, "[THERMAL-STAGNATION] Condition started: outlet=%.1f°C ret=%.1f°C delta=%.1f°C (threshold=%.1f°C)",
               outlet, ret, delta, thermal_stagnation_delta_);
      return;
    }
    // Condition is persisting – check if 10 seconds have elapsed
    if ((ms_now - thermal_stagnation_started_) >= 10000) {
      ESP_LOGW(TAG, "===========================================");
      ESP_LOGW(TAG, "[THERMAL-STAGNATION] Return pipe heat-soak detected!");
      ESP_LOGW(TAG, "  Outlet=%.1f°C  Return=%.1f°C  Delta=%.1f°C (< %.1f°C threshold)",
               outlet, ret, delta, thermal_stagnation_delta_);
      ESP_LOGW(TAG, "  Running pump for %u s to flush stagnant return water", THERMAL_STAGNATION_RUNTIME);
      ESP_LOGW(TAG, "===========================================");
      thermal_stagnation_started_ = 0;
      last_thermal_stagnation_run_ = clock_->now().timestamp;
      run_pump(PumpTrigger::THERMAL_STAGNATION);
    }
  } else {
    // Condition no longer met – cancel pending timer
    if (thermal_stagnation_started_ != 0) {
      ESP_LOGD(TAG, "[THERMAL-STAGNATION] Condition cleared (delta=%.1f°C), timer reset", delta);
      thermal_stagnation_started_ = 0;
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
  // FIX #2: this runs in the outlet sensor's publish-callback path
  // (on_outlet_sample_), which fires even before SNTP has synced - unlike
  // loop(), which guards on clock validity. Never dereference clock_ here
  // without checking it, and never trust an invalid timestamp.
  const bool clock_valid = clock_ && clock_->now().is_valid();

  if (learning_enabled_ && clock_valid)
    learn_now();  // needs a valid time to pick the correct slot

  yellow_led_on_until_ = millis() + 5000;

  if (pump_running_) {
    ESP_LOGD(TAG, "Pump already running, request acknowledged");
    return;
  }

  // Without a valid clock the "recent run" age cannot be evaluated - a real
  // water draw still deserves a pump run, so default to running.
  bool recent_run = false;
  if (clock_valid && last_run_epoch_ != 0) {
    time_t now_epoch = clock_->now().timestamp;
    recent_run = (now_epoch - last_run_epoch_) <= (time_t)USER_REQUEST_MAX_AGE;
    if (recent_run) {
      ESP_LOGD(TAG, "Recent pump run detected, skipping (age=%lds)",
               (long)(now_epoch - last_run_epoch_));
    }
  }

  if (!recent_run) {
    run_pump(PumpTrigger::WATER_DRAW);
  }
}

void HotWaterController::learn_now() {
  // FIX #2: defensive guard - callers must ensure clock validity, but a
  // learning event into a wrong slot is worse than a skipped one.
  if (!clock_ || !clock_->now().is_valid()) {
    ESP_LOGW(TAG, "learn_now() skipped - clock not valid");
    return;
  }
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

  last_decay_day_ = n.day_of_year;

  // FIX #16: floor() instead of round(). With round(), values <= 25 (at
  // DECAY=0.98) never reach 0 because round(x*0.98)==x for small x - once
  // learned slots would leave ghost entries forever. floor() guarantees
  // strict monotonic convergence to 0 for every value.
  for (int d = 0; d < 7; d++)
    for (int slot = 0; slot < 48; slot++)
      learn_[d][slot] = (uint8_t) std::floor(learn_[d][slot] * DECAY);

  ESP_LOGI(TAG, "Learning matrix decayed");

  // Save updated matrix to flash at end of each day
  ESP_LOGI(TAG, "Learning matrix save by daily trigger.");
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

// FIX #7: single source of truth for trigger names (also used by the YAML
// "Pump Status" text sensor via get_pump_trigger_str()).
const char *HotWaterController::trigger_to_str_(PumpTrigger t) {
  switch (t) {
    case PumpTrigger::MANUAL_BUTTON:      return "Manual Button";
    case PumpTrigger::MANUAL_WEBUI:       return "Web UI";
    case PumpTrigger::WATER_DRAW:         return "Water Draw";
    case PumpTrigger::SCHEDULED:          return "Scheduled";
    case PumpTrigger::DISINFECTION:       return "Disinfection";
    case PumpTrigger::ANTI_STAGNATION:    return "Anti-Stagnation";
    case PumpTrigger::THERMAL_STAGNATION: return "Thermal-Stagnation Flush";
    case PumpTrigger::NONE:               return "None";
    default:                              return "Unknown";
  }
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
  // Skip for: disinfection, anti-stagnation, thermal-stagnation
  // (thermal stagnation is triggered precisely BECAUSE return is too warm)
  if (trigger != PumpTrigger::ANTI_STAGNATION &&
      trigger != PumpTrigger::THERMAL_STAGNATION &&
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
  // FIX (millis-Rollover): keep a millisecond reference whose unsigned
  // difference is wrap-safe; pump_start_ (seconds) stays populated for the
  // GUI package.
  pump_start_ms_ = millis();
  pump_start_ = pump_start_ms_ / 1000;

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

  // Log pump start with trigger reason (FIX #7: shared mapping)
  const char *trigger_str = trigger_to_str_(trigger);

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

  // Wrap-safe elapsed seconds (unsigned ms difference survives millis rollover)
  uint32_t elapsed = (millis() - pump_start_ms_) / 1000;

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

  // Thermal stagnation flush: run for fixed short duration, then let temperatures settle
  if (pump_trigger_ == PumpTrigger::THERMAL_STAGNATION) {
    if (elapsed >= THERMAL_STAGNATION_RUNTIME) {
      stop_pump("Thermal-stagnation flush complete");
    }
    return;  // Skip temperature checks – return pipe IS hot, normal logic would stop immediately
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

  // Calculate and store energy for this cycle (wrap-safe ms difference)
  uint32_t elapsed = (millis() - pump_start_ms_) / 1000;
  last_cycle_duration_ = elapsed;
  last_cycle_energy_ = energy_sum_ / 1000.0f;  // Convert Wh to kWh

  ESP_LOGI(TAG, "Pump cycle complete: duration=%us, energy=%.4f kWh (%u samples)",
           last_cycle_duration_, last_cycle_energy_, energy_samples_);

  // CRITICAL: Update baseline outlet temperature using slow-moving average
  // Captured NOW while fresh hot water from tank is at sensor (before 40cm pipe cools)
  // This represents actual tank temperature and adapts gradually to boiler setpoint changes
  //
  // FIX #15: skip the baseline update after anti-stagnation (15 s) and
  // thermal-stagnation (10 s) runs. Those runs are too short to bring fresh
  // tank water to the sensor, so blending their readings in would drag the
  // baseline down and make disinfection detection over-sensitive.
  // (pump_trigger_ is still valid here - it is reset further below.)
  bool baseline_eligible = !disinfection_mode_ &&
                           pump_trigger_ != PumpTrigger::ANTI_STAGNATION &&
                           pump_trigger_ != PumpTrigger::THERMAL_STAGNATION;

  if (baseline_eligible && outlet_ && !std::isnan(outlet_->state)) {
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
    ESP_LOGD(TAG, "Baseline update skipped (trigger=%s, disinfection=%d, outlet %s)",
             trigger_to_str_(pump_trigger_), disinfection_mode_,
             (outlet_ && !std::isnan(outlet_->state)) ? "valid" : "invalid");
  }

  pump_->turn_off();
  pump_running_ = false;
  pump_trigger_ = PumpTrigger::NONE;  // Reset trigger
  // 0 = "unknown"; handle_user_request() then treats it as "no recent run"
  // and allows an immediate start, which is the safe direction.
  last_run_epoch_ = (clock_ && clock_->now().is_valid()) ? clock_->now().timestamp : 0;

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

      // FIX #4: visual feedback WITHOUT blocking. The old 6x delay(200) loop
      // stalled loop() for 1.2 s - on this display node exactly the kind of
      // stall the callback-based draw detection was introduced to survive.
      // The sequence is now driven from update_leds().
      led_flash_remaining_ = 6;      // on/off/on/off/on/off, 200 ms each
      led_flash_next_ms_ = now;      // start immediately
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
  LearnMatrixData data{};

  data.magic = MATRIX_MAGIC;
  data.version = MATRIX_VERSION;

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
    init_default_pattern_();
    return;
  }

  // FIX #11: deterministic format detection via magic/version instead of
  // hoping an old layout produces a different additive checksum.
  if (data.magic != MATRIX_MAGIC || data.version != MATRIX_VERSION) {
    ESP_LOGW(TAG, "Learning matrix format mismatch (magic=0x%08X, version=%u) - resetting to typical pattern",
             data.magic, data.version);
    init_default_pattern_();
    return;
  }

  // Validate checksum (guards against flash corruption)
  uint32_t expected_checksum = calculate_checksum_(data.learn);
  if (data.checksum != expected_checksum) {
    ESP_LOGW(TAG, "Learning matrix checksum mismatch (expected 0x%08X, got 0x%08X) - resetting to typical pattern",
             expected_checksum, data.checksum);
    init_default_pattern_();
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

uint32_t HotWaterController::calculate_checksum_() const {
  return calculate_checksum_(learn_);
}

uint32_t HotWaterController::calculate_checksum_(const uint8_t (&m)[7][48]) {
  uint32_t checksum = 0;
  for (int d = 0; d < 7; d++) {
    for (int slot = 0; slot < 48; slot++) {
      checksum += m[d][slot];
    }
  }
  return checksum;
}

// FIX #11: the typical daily pattern used to be written out three times
// (load failure, checksum mismatch, manual reset) with subtle differences -
// the mismatch path did not clear the matrix first. This helper is now the
// single source: it always clears, then seeds the pattern.
void HotWaterController::init_default_pattern_() {
  // Clear the entire learning matrix first
  for (int d = 0; d < 7; d++) {
    for (int slot = 0; slot < 48; slot++) {
      learn_[d][slot] = 0;
    }
  }

  // Weekday pattern (Mon-Fri)
  for (int d = 0; d < 5; d++) {
    // Morning shower: 6:00-8:00 AM (slots 12-16)
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
}

void HotWaterController::reset_learning_matrix_() {
  ESP_LOGW(TAG, "===========================================");
  ESP_LOGW(TAG, "RESETTING LEARNING MATRIX");
  ESP_LOGW(TAG, "===========================================");

  init_default_pattern_();

  last_decay_day_ = 0;

  // Save the reinitialized matrix to flash
  save_learning_matrix_();

  ESP_LOGI(TAG, "Learning matrix reset and reinitialized with typical daily pattern");
  ESP_LOGI(TAG, "System will adapt to actual user behavior over time");
}

void HotWaterController::update_leds() {
  // FIX #4: non-blocking flash sequence after matrix reset takes priority
  if (led_flash_remaining_ > 0) {
    if ((int32_t)(millis() - led_flash_next_ms_) >= 0) {
      if (led_yellow_) led_yellow_->set_state(led_flash_remaining_ % 2 == 0);
      led_flash_remaining_--;
      led_flash_next_ms_ = millis() + 200;
      if (led_flash_remaining_ == 0 && led_yellow_)
        led_yellow_->set_state(false);  // ensure defined end state
    }
    return;
  }

  if (!learning_enabled_) {
    if (led_yellow_) led_yellow_->set_state(true);
    return;
  }

  if (led_yellow_) {
    // FIX (millis-Rollover): signed difference instead of a raw '<' compare,
    // which misbehaves when millis()+5000 wraps past zero.
    if ((int32_t)(yellow_led_on_until_ - millis()) > 0)
      led_yellow_->set_state(true);
    else
      led_yellow_->set_state(false);
  }
}

}  // namespace esphome_hotcirc
}  // namespace esphome
