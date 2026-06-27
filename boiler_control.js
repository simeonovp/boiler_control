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
  },

  pwm: 0,
  temp_in: 0,
  temp_out: 0,
  prevError: 0, // error from last iteration
  integral: 0, // accumulated error (integral)
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
    if (val > max) {
      return max
    }
    if (val < min) {
      return min
    }
    return val
  }

  // Pre-calculate proportional and derivative parts
  const proportional = kp * diff
  const dt = (config.pid_dt && config.pid_dt > 0) ? config.pid_dt : 1e-6
  const derivative = kd * (diff - self.prevError) / dt

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

  if (!isSaturated || !isDrivingDeeper || nearLimit) {
    self.integral = potentialIntegral
  }

  // 6. Secondary hard-clamp protection for the integral state
  const integralLimit = Number.isFinite(config.integral_limit)
    ? Math.abs(config.integral_limit)
    : Math.abs(maxPwm / Math.max(Math.abs(ki), 1e-6))

  if (self.integral > integralLimit) {
    self.integral = integralLimit
  }
  else if (self.integral < -integralLimit) {
    self.integral = -integralLimit
  }

  // Final PWM value using the verified integral state
  const integralTerm = ki * self.integral
  self.pwm = proportional + integralTerm + derivative

  // persist error for the next iteration
  self.prevError = diff

  // Clamp output PWM
  self.pwm = clamp(self.pwm, minPwm, maxPwm)

  // Debug
  msg.error = diff
  msg.proportional = Math.round(proportional * 100) / 100
  msg.integral = Math.round(integralTerm * 100) / 100
  msg.derivative = Math.round(derivative * 100) / 100

  // Output data
  msg.payload = Math.round(self.pwm)
  msg.d_temp = diff
  send(msg)
}

function onConfig(config) {
  if (!config || typeof config !== 'object') return
  // whitelist config keys to avoid prototype pollution and unexpected fields
  const allowed = ['pwm_control','set_temp','last_pwm','min_pwm','max_pwm','def_pwm','pwm_step','pid_kp','pid_ki','pid_kd','pid_dt','time_in','time_out','last_integral','integral_limit']
  for (const k of allowed) {
    if (Object.prototype.hasOwnProperty.call(config, k)) {
      if (k === 'pwm_control') self.config[k] = !!config[k]
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
}

function send(msg) {
  node.send(msg)
}

function onInput (msg, send, done = () => {}) {
  Object.assign(self, context.get('boiler') || {})
  switch (msg.topic) {
    case 'config':
      onConfig(msg.payload)
      break
    case 'boiler_temp_in':
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