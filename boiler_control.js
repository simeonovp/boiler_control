const self = {
  config: {
    pwm_control: false, //automatic is off
    set_temp : 400, //set temperature
    last_pwm : 0, //last pwm value
    min_pwm : 10, //min pwm value
    max_pwm : 100, //max pwm value
    def_pwm : 0, //default pwm value
    pwm_step : 5, //pwm step value
    pid_kp : 1.0, // propotional weight
    pid_ki : 0.1, // integral weight
    pid_kd : 0.01, // differential weight
    pid_dt : 0.1, // time interval
    time_in: 0,
    time_out: 0,
    last_integral: 0,
    learned_ks: 0,
    learned_t: 0,
    learned_tot: 0,
  },

  pwm: 0,
  temp_in: 0,
  temp_out: 0,
  prevError: 0, // error from last iteration
  integral: 0, // accumulated error (integral)

  // Additional internal states for learning
  learn: {
    pwm_jump_val: 0,     // PWM value before the jump
    pwm_jump_time: 0,    // Timestamp when PWM changed
    temp_jump_val: 0,    // Temperature before the jump
    tot_time: 0,         // Detected sensor dead-time (Tt)
    has_responded: false, // Flag to check if sensor started reacting
    max_gradient: 0,
    last_learn_temp: 0, // for gradient calculation
    last_learn_time: 0,  // for gradient calculation
    filtered_gradient: 0,
  }

}

function tunePIDParameters(send) {
  const ks = self.config.learned_ks
  const t = self.config.learned_t
  const tt = self.config.learned_tot

  // Guard against division by zero or unrealistic physical parameters
  if (!ks || !t || !tt || (0 >= ks) || (0 >= t) || (0 >= tt)) return

  // 1. Calculate proportional gain (Kp)
  const kp = (0.95 * t) / (ks * tt)

  // 2. Calculate integral time (Ti) and convert to gain (Ki)
  const ti = 1.4 * t
  const ki = kp / ti

  // 3. Calculate derivative time (Td) and convert to gain (Kd)
  const td = 0.42 * tt
  const kd = kp * td

  // 4. Apply tuned parameters to the running system configuration
  self.config.pid_kp = Math.round(kp * 100) / 100
  self.config.pid_ki = Math.round(ki * 1000) / 1000
  self.config.pid_kd = Math.round(kd * 100) / 100

  // Emit a specific tuning log entry to Output 2
  // Convert the diagnostic object into a clean single-line JSON string
  const diagnosticMsg = {
      event: "TUNING_SUCCESS",
      timestamp: Date.now(),
      time_string: new Date().toISOString(),
      calculated_ks: ks,
      calculated_t: t,
      calculated_deadtime: tt,
      new_kp: self.config.pid_kp,
      new_ki: self.config.pid_ki,
      new_kd: self.config.pid_kd
    }
  send([null, { payload: JSON.stringify(diagnosticMsg) }])
}

function estimateSystemParameters(currentTemp, currentPwm, send) {
  const now = Date.now()
  const tempThreshold = 0.2
  const minPwmJump = 20
  const l = self.learn

  // 1. Trigger learning phase on a significant PWM jump
  if (Math.abs(currentPwm - l.pwm_jump_val) > minPwmJump && !l.pwm_jump_time) {
    l.pwm_jump_time = now
    l.pwm_jump_val = currentPwm
    l.temp_jump_val = currentTemp
    l.has_responded = false
    l.max_gradient = 0
    l.last_learn_temp = currentTemp
    l.last_learn_time = now
    return
  }

  // If no learning cycle is active, exit early
  if (!l.pwm_jump_time) return

  // Calculate time delta since the last sensor sample
  const dt = (now - l.last_learn_time) / 1000
  if (dt <= 0) return

  // 2. Measure dead-time (sensor latency) until the temperature reacts
  if (!l.has_responded) {
    if (currentTemp > l.temp_jump_val + tempThreshold) {
      l.tot_time = (now - l.pwm_jump_time) / 1000
      l.has_responded = true
    }
    l.last_learn_temp = currentTemp
    l.last_learn_time = now
    return
  }

  // 3. Track and filter the gradient (heating rate in °C/second)
  const rawGradient = (currentTemp - l.last_learn_temp) / dt
  
  // Smoothing factor alpha (0.2 means: 20% new value, 80% history)
  const alpha = 0.2
  l.filtered_gradient = (alpha * rawGradient) + ((1 - alpha) * l.filtered_gradient)
  
  if (l.filtered_gradient > l.max_gradient) l.max_gradient = l.filtered_gradient

  // Update tracking references for the next sample
  l.last_learn_temp = currentTemp
  l.last_learn_time = now

  // 4. Check for steady state (curve flattens out, gradient drops near 0)
  const isSteadyState = l.filtered_gradient < 0.05 && (now - l.pwm_jump_time) / 1000 > l.tot_time + 5

  if (!isSteadyState) return

  const deltaPwm = l.pwm_jump_val - (self.config.last_pwm || 0)
  const deltaTemp = currentTemp - l.temp_jump_val

  if ((0 < Math.abs(deltaPwm)) && (0 < Math.abs(l.max_gradient))) {
    // Ks = deltaTemp / deltaPwm
    const ks = deltaTemp / deltaPwm
    // T = deltaTemp / max_gradient (minus sensor dead-time)
    const t = (deltaTemp / l.max_gradient) - l.tot_time

    // Store learned parameters in config if valid
    if ((0 < ks) && (0 < t)) {
      self.config.learned_ks = ks
      self.config.learned_t = t
      self.config.learned_tot = l.tot_time

      // Trigger automatic tuning
      tunePIDParameters(send)
    }
  }

  // Reset learning flags for the next opportunity
  l.pwm_jump_time = 0

}

function onBoilerTempIn(msg, send) {
  const config = self.config
  if (!config.pwm_control) return

  // Input data
  const processVariable =  msg.payload // current temperature
  if (!Number.isFinite(processVariable)) return

  // config is validated on update via `onConfig`; avoid redundant checks here for performance

  // Calculation
  const diff = (config.set_temp - processVariable)
  // Ensure critical configuration values are numeric before calculation
  const kp = Number.isFinite(config.pid_kp) ? config.pid_kp : 0
  const ki = Number.isFinite(config.pid_ki) ? config.pid_ki : 0
  const kd = Number.isFinite(config.pid_kd) ? config.pid_kd : 0
  const minPwm = Number.isFinite(config.min_pwm) ? config.min_pwm : 0
  const maxPwm = Number.isFinite(config.max_pwm) ? config.max_pwm : 100
  const hyst = Number.isFinite(config.hysteresis) ? config.hysteresis : 0.5

  // Helper function for clipping values
  const clamp = (val, min, max) => {
    if (val > max) return max
    if (val < min) return min
    return val
  }

  // Pre-calculate proportional and derivative parts
  const proportional = kp * diff

  // Detect if the hardware is heating (temperature is rising or already high)
  // If the temperature is cold and not rising, the flow is likely stopped
  const dt = (config.pid_dt && config.pid_dt > 0) ? config.pid_dt : 1e-6
  const currentGradient = (processVariable - self.temp_out) / dt
  self.temp_out = processVariable // store for next iteration

  // Standby management: If temperature is low and not rising, freeze controller
  const isHeaterHardwareOff = (processVariable < (config.set_temp - 5)) && (currentGradient <= 0)

  if (isHeaterHardwareOff) {
    self.integral = 0 // reset integral to prevent wind-up during standby
    self.pwm = config.min_pwm || 10 // keep at minimum or default standby
    msg.payload = Math.round(self.pwm)
    msg.status = 'HARDWARE_STANDBY'
    send(msg)
    return
  }

  const derivative = -kd * currentGradient

  // 1. Calculate potential integral and its term
  const potentialIntegral = self.integral + (diff * config.pid_dt)
  const potentialIntegralTerm = ki * potentialIntegral

  // 2. Calculate potential total PWM value
  const potentialPwm = proportional + potentialIntegralTerm + derivative

  // 3. Determine actuator state (use real feedback if available, fallback to simulation)
  const realFeedback = msg.actuator_feedback
  const clippedPwm = Number.isFinite(realFeedback)
    ? clamp(realFeedback, minPwm, maxPwm)
    : clamp(potentialPwm, minPwm, maxPwm)

  // 4. Check for saturation
  const isSaturated = potentialPwm !== clippedPwm
  const isDrivingDeeper = ki !== 0 && Math.sign(diff) === Math.sign(ki)

  // 5. Anti-windup decision with configurable hysteresis tolerance
  const nearLimit = Math.abs(potentialPwm - clippedPwm) < hyst

  if (!isSaturated || !isDrivingDeeper || nearLimit) self.integral = potentialIntegral

  // 6. Secondary hard-clamp protection for the integral state
  const integralLimit = Number.isFinite(config.integral_limit)
    ? Math.abs(config.integral_limit)
    : Math.abs(maxPwm / Math.max(Math.abs(ki), 1e-6))

  if (self.integral > integralLimit) self.integral = integralLimit
  else if (self.integral < -integralLimit) self.integral = -integralLimit

  // Final PWM value using the verified integral state
  const integralTerm = ki * self.integral
  self.pwm = proportional + integralTerm + derivative

  // persist error for the next iteration
  self.prevError = diff

  // Clamp output PWM
  self.pwm = clamp(self.pwm, minPwm, maxPwm)
  self.config.last_pwm = self.pwm

  estimateSystemParameters(processVariable, self.pwm, send)

  // Debug
  msg.error = diff
  msg.proportional = Math.round(proportional * 100) / 100
  msg.integral = Math.round(integralTerm * 100) / 100
  msg.derivative = Math.round(derivative * 100) / 100

  // Output data
  msg.payload = Math.round(self.pwm)
  msg.d_temp = diff

  // Prepare comprehensive diagnostic message for Output 2 (File Node)
  const l = self.learn
  const diagnosticMsg = {
    timestamp: Date.now(),
    time_string: new Date().toISOString(),
    process_variable: processVariable,
    control_error: diff,
    calculated_pwm: self.pwm,
    proportional_term: Math.round(proportional * 100) / 100,
    integral_term: Math.round(integralTerm * 100) / 100,
    derivative_term: Math.round(derivative * 100) / 100,
    raw_gradient: Math.round((currentGradient || 0) * 10000) / 10000,
    filtered_gradient: Math.round((l.filtered_gradient || 0) * 10000) / 10000,
    max_gradient: Math.round((l.max_gradient || 0) * 10000) / 10000,
    has_responded: l.has_responded ? 1 : 0,
    dead_time: l.tot_time,
    config_kp: config.pid_kp,
    config_ki: config.pid_ki,
    config_kd: config.pid_kd,
    learned_ks: config.learned_ks,
    learned_t: config.learned_t,
    learned_tot: config.learned_tot
  }
  send([msg, {payload: JSON.stringify(diagnosticMsg)}])
}

function onConfig(config) {
  if (!config || ('object' !== typeof config)) return
  
  // whitelist config keys to avoid prototype pollution and unexpected fields
  const allowed = ['pwm_control','set_temp','last_pwm','min_pwm','max_pwm','def_pwm','pwm_step','pid_kp','pid_ki','pid_kd','pid_dt','time_in','time_out','last_integral','integral_limit','learned_ks','learned_t','learned_tot']
  for (const k of allowed) {
    if (Object.prototype.hasOwnProperty.call(config, k)) {
      if ('pwm_control' === k) self.config[k] = !!config[k]
      else if (Number.isFinite(Number(config[k]))) self.config[k] = Number(config[k])
      else self.config[k] = config[k]
    }
  }

  // ensure sensible min/max
  if (self.config.min_pwm > self.config.max_pwm) {
    const tmp = self.config.min_pwm
    self.config.min_pwm = self.config.max_pwm
    self.config.max_pwm = tmp
  }

  const configLog = {
    event: "CONFIG_CHANGED",
    timestamp: Date.now(),
    time_string: new Date().toISOString(),
    updated_fields: config, // shows exactly what was sent in the inject payload
    current_full_config: self.config // shows the state of the entire config matrix after the merge
  }
  send([null, { payload: JSON.stringify(configLog) }])
}

function send(msg) {
  node.send(msg)
}

function onInput(msg, send, done = () => {}) {
  Object.assign(self, context.get('boiler') || {})
  switch (msg.topic) {
    case 'config':
      onConfig(msg.payload)
      break
    case 'boiler_temp_out':
      onBoilerTempIn(msg, send)
      break
    default:
      return
  }
  context.set('boiler', self)
}

function onClose(done = () => {}) {
  send({topic: 'config', payload: self.config})
}

if (msg) onInput(msg, send)