// xsns_91_user.ino

struct Config {
  bool pwm_control = false;
  float set_temp = 40.0; // In C++ nutzen wir echte Celsius (z.B. 40.0 statt 400)
  float min_pwm = 10.0;
  float max_pwm = 100.0;
  float hysteresis = 0.5;
  float pid_kp = 1.0;
  float pid_ki = 0.1;
  float pid_kd = 0.01;
  float pid_dt = 0.1;
  float integral_limit = 0.0;
  float last_pwm = 0.0;
  float learned_ks = 0.0;
  float learned_t = 0.0;
  float learned_tot = 0.0;
};

struct Learn {
  float pwm_jump_val = 0.0;
  uint32_t pwm_jump_time = 0;
  float temp_jump_val = 0.0;
  float tot_time = 0.0;
  bool has_responded = false;
  float max_gradient = 0.0;
  float last_learn_temp = 0.0;
  uint32_t last_learn_time = 0;
  float filtered_gradient = 0.0;
};

struct BoilerSystem {
  Config config;
  Learn learn;
  float pwm = 0.0;
  float temp_out = 0.0;
  float prev_error = 0.0;
  float integral = 0.0;
};

static BoilerSystem self;

float clamp(float val, float min, float max) {
  if (val > max) return max;
  if (val < min) return min;
  return val;
}

void tunePIDParameters() {
  // In a real C++ implementation, ensure these variables exist in your struct
  const float ks = self.config.learned_ks;
  const float t = self.config.learned_t;
  const float tt = self.config.learned_tot;

  // Guard against division by zero or unrealistic physical parameters using Yoda style
  if (!ks || !t || !tt || (0.0 >= ks) || (0.0 >= t) || (0.0 >= tt)) return;

  // 1. Calculate proportional gain (Kp)
  const float kp = (0.95 * t) / (ks * tt);

  // 2. Calculate integral time (Ti) and convert to gain (Ki)
  const float ti = 1.4 * t;
  const float ki = kp / ti;

  // 3. Calculate derivative time (Td) and convert to gain (Kd)
  const float td = 0.42 * tt;
  const float kd = kp * td;

  // 4. Apply tuned parameters to the running system configuration with rounding
  self.config.pid_kp = round(kp * 100.0) / 100.0;
  self.config.pid_ki = round(ki * 1000.0) / 1000.0;
  self.config.pid_kd = round(kd * 100.0) / 100.0;
}

void estimateSystemParameters(float currentTemp, float currentPwm) {
  const uint32_t now = millis();
  const float tempThreshold = 0.2;
  const float minPwmJump = 20.0;
  Learn& l = self.learn;

  // 1. Trigger learning phase on a significant PWM jump
  if ((fabs(currentPwm - l.pwm_jump_val) > minPwmJump) && (0 == l.pwm_jump_time)) {
    l.pwm_jump_time = now;
    l.pwm_jump_val = currentPwm;
    l.temp_jump_val = currentTemp;
    l.has_responded = false;
    l.max_gradient = 0.0;
    l.last_learn_temp = currentTemp;
    l.last_learn_time = now;
    return;
  }

  // If no learning cycle is active, exit early
  if (0 == l.pwm_jump_time) return;

  // Calculate time delta since the last sensor sample
  const float dt = (float)(now - l.last_learn_time) / 1000.0;
  if (0.0 >= dt) return;

  // 2. Measure dead-time (sensor latency) until the temperature reacts
  if (!l.has_responded) {
    if (currentTemp > (l.temp_jump_val + tempThreshold)) {
      l.tot_time = (float)(now - l.pwm_jump_time) / 1000.0;
      l.has_responded = true;
    }
    l.last_learn_temp = currentTemp;
    l.last_learn_time = now;
    return;
  }

  // 3. Track and filter the gradient (heating rate in °C/second)
  const float rawGradient = (currentTemp - l.last_learn_temp) / dt;
  
  // Smoothing factor alpha (0.2 means: 20% new value, 80% history)
  const float alpha = 0.2;
  l.filtered_gradient = (alpha * rawGradient) + ((1.0 - alpha) * l.filtered_gradient);
  
  if (l.filtered_gradient > l.max_gradient) l.max_gradient = l.filtered_gradient;

  // Update tracking references for the next sample
  l.last_learn_temp = currentTemp;
  l.last_learn_time = now;

  // 4. Check for steady state (curve flattens out, gradient drops near 0)
  const bool isSteadyState = (l.filtered_gradient < 0.05) && (((float)(now - l.pwm_jump_time) / 1000.0) > (l.tot_time + 5.0));

  if (!isSteadyState) return;

  const float deltaPwm = l.pwm_jump_val - (self.config.last_pwm);
  const float deltaTemp = currentTemp - l.temp_jump_val;

  if ((0.0 < fabs(deltaPwm)) && (0.0 < fabs(l.max_gradient))) {
    // Ks = deltaTemp / deltaPwm
    const float ks = deltaTemp / deltaPwm;
    // T = deltaTemp / max_gradient (minus sensor dead-time)
    const float t = (deltaTemp / l.max_gradient) - l.tot_time;

    // Store learned parameters in config if valid
    if ((0.0 < ks) && (0.0 < t)) {
      self.config.pid_kp = ks; // Temporary storage or direct mapping depending on choice
      // Store raw learned markers if you want to keep them structured
      // self.config.learned_ks = ks; 
      // self.config.learned_t = t;
      // self.config.learned_tot = l.tot_time;

      tunePIDParameters();
    }
  }

  // Reset learning flags for the next opportunity
  l.pwm_jump_time = 0;
}

void onBoilerTempIn(float processVariable) {
  Config& config = self.config;
  if (!config.pwm_control) return;
  if (isnan(processVariable)) return;

  const float diff = config.set_temp - processVariable;
  
  // Local references to mirror your validated variables
  const float kp = config.pid_kp;
  const float ki = config.pid_ki;
  const float kd = config.pid_kd;
  const float minPwm = config.min_pwm;
  const maxPwm = config.max_pwm;
  const float hyst = config.hysteresis;

  const float proportional = kp * diff;

  // Standby & Gradient calculation
  const float dt = (config.pid_dt > 0.0) ? config.pid_dt : 1e-6;
  const float currentGradient = (processVariable - self.temp_out) / dt;
  self.temp_out = processVariable;

  const bool isHeaterHardwareOff = (processVariable < (config.set_temp - 5.0)) && (currentGradient <= 0.0);

  if (isHeaterHardwareOff) {
    self.integral = 0.0;
    self.pwm = config.min_pwm;
    // Tasmota core function to inject the PWM directly to the H801 pins
    TasmotaSetPwm(Math.round(self.pwm)); 
    return;
  }

  // Pre-calculate derivative part using physical gradient
  const float derivative = -kd * currentGradient;

  // 1. Calculate potential integral and its term
  const float potentialIntegral = self.integral + (diff * config.pid_dt);
  const float potentialIntegralTerm = ki * potentialIntegral;

  // 2. Calculate potential total PWM value
  const float potentialPwm = proportional + potentialIntegralTerm + derivative;

  // 3. Determine actuator state (Simulation clamp)
  const float clippedPwm = clamp(potentialPwm, minPwm, maxPwm);

  // 4. Check for saturation (using sign bit/Yoda style)
  const bool isSaturated = potentialPwm != clippedPwm;
  const bool isDrivingDeeper = (0.0 != ki) && ((diff > 0) == (ki > 0));

  // 5. Anti-windup decision
  const bool nearLimit = fabs(potentialPwm - clippedPwm) < hyst;

  if (!isSaturated || !isDrivingDeeper || nearLimit) self.integral = potentialIntegral;

  // 6. Secondary hard-clamp protection
  float integralLimit = config.integral_limit;
  if (0.0 == integralLimit) {
    integralLimit = fabs(maxPwm / ((0.0 != ki) ? fabs(ki) : 1e-6));
  }

  if (self.integral > integralLimit) self.integral = integralLimit;
  else if (self.integral < -integralLimit) self.integral = -integralLimit;

  // Final PWM calculation
  const float integralTerm = ki * self.integral;
  self.pwm = proportional + integralTerm + derivative;

  self.prev_error = diff;
  self.pwm = clamp(self.pwm, minPwm, maxPwm);
  config.last_pwm = self.pwm;

  // Trigger self-learning module
  estimateSystemParameters(processVariable, self.pwm);

  // Execute the physical hardware PWM change in Tasmota
  TasmotaChannelSet(0, (int)round(self.pwm));
}

void User_Read(void) {
  // Read the current temperature of the DS18B20 from Tasmota's internal sensor array
  float dsbTemp = SensorValue(0); 

  // Static variable to track the last value and detect change events
  static float last_triggered_temp = -99.0;

  // Trigger only if the temperature has actually changed (Event-Driven)
  if (dsbTemp != last_triggered_temp) {
    last_triggered_temp = dsbTemp;
    onBoilerTempIn(dsbTemp);
  }
}
