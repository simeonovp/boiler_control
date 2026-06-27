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

  // ensure numeric config fields
  const ensureNumber = (k, fallback = 0) => {
    const v = Number(self.config[k])
    self.config[k] = Number.isFinite(v) ? v : fallback
  }
  ensureNumber('set_temp', 0)
  ensureNumber('pid_kp', 0)
  ensureNumber('pid_ki', 0)
  ensureNumber('pid_kd', 0)
  ensureNumber('pid_dt', 0.1)
  ensureNumber('min_pwm', 0)
  ensureNumber('max_pwm', 100)

  // Calculation
  const diff = (config.set_temp - processVariable)
  const proportional = config.pid_kp * diff

  // Integral part (Error sum over the time) with anti-windup
  self.integral += diff * config.pid_dt
  // calculate an integral clamp (allow override via config.integral_limit)
  const integralLimit = Number.isFinite(config.integral_limit)
    ? Math.abs(config.integral_limit)
    : Math.abs(config.max_pwm / Math.max(config.pid_ki, 1e-6))
  if (self.integral > integralLimit) self.integral = integralLimit
  if (self.integral < -integralLimit) self.integral = -integralLimit
  const integralTerm = config.pid_ki * self.integral

  // differential part (Rate of teh error change)
  const dt = (config.pid_dt && config.pid_dt > 0) ? config.pid_dt : 1e-6
  const derivative = config.pid_kd * (diff - self.prevError) / dt

  self.pwm = proportional + integralTerm + derivative

  // persist error for the next iteration
  self.prevError = diff

  // Check min/max
  if (self.pwm > config.max_pwm) self.pwm = config.max_pwm
  if (self.pwm < config.min_pwm) self.pwm = config.min_pwm

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

if (msg.payload) onInput(msg, send)