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
    is_learned: false,
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
  if (!ks || !t || !tt || (0 >= ks) || (0 >= t) || (0 >= tt)) return send([null, { payload: {event: "TUNING_FAILED"} }])

  // 1. Calculate proportional gain (Kp) tailored for 0% overshoot setpoint tracking
  const kp = (0.6 * t) / (ks * tt)

  // 2. Calculate integral time (Ti) and convert to gain (Ki)
  const ti = 4.0 * tt
  const ki = kp / ti

  // 3. Calculate derivative time (Td) and convert to gain (Kd)
  const td = 0.5 * tt
  const kd = kp * td

  // 4. Apply tuned parameters to the running system configuration
  self.config.pid_kp = Math.round(kp * 100) / 100
  self.config.pid_ki = Math.round(ki * 1000) / 1000
  self.config.pid_kd = Math.round(kd * 100) / 100
  self.config.is_learned = true

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
  send([null, { payload: diagnosticMsg }])
}

function estimateSystemParameters(currentTemp, send) {
  const now = Date.now()
  const tempThreshold = 0.2
  const minPwmJump = 20
  const l = self.learn

  // 1. Trigger learning phase on a significant PWM jump
  if ((Math.abs(self.pwm - l.pwm_jump_val) > minPwmJump) && !l.pwm_jump_time) {
    l.pwm_jump_time = now
    l.pwm_jump_val = self.pwm
    l.temp_jump_val = currentTemp
    l.has_responded = false
    l.max_gradient = 0
    l.last_learn_temp = currentTemp
    l.last_learn_time = now
    return send([null, { payload: {event: "LEARN_RETURN 1"} }])
  }

  // If no learning cycle is active, exit early
  if (!l.pwm_jump_time) return send([null, { payload: {event: "LEARN_RETURN 2", currentPwm:self.pwm, pwm_jump_val: l.pwm_jump_val, pwm_jump_time: l.pwm_jump_time} }])

  // Calculate time delta since the last sensor sample
  const dt = (now - l.last_learn_time) / 1000
  if (dt <= 0) return  send([null, { payload: {event: "LEARN_RETURN 3"} }])

  // 2. Measure dead-time (sensor latency) until the temperature reacts
  if (false === l.has_responded) {
    if (currentTemp > l.temp_jump_val + tempThreshold) {
      l.tot_time = (now - l.pwm_jump_time) / 1000
      l.has_responded = true
    }
    else {
      l.last_learn_temp = currentTemp
      l.last_learn_time = now
      return  send([null, { payload: {event: "LEARN_RETURN 4"} }]) // Safe exit only if no physical response has occurred yet
    }
  }

  // 3. Track and filter the gradient (heating rate in °C/second)
  const rawGradient = (currentTemp - l.last_learn_temp) / dt

  // Catch unitialized context state to prevent fatal NaN propagation
  if ('undefined' === typeof l.filtered_gradient) l.filtered_gradient = rawGradient

  // Smoothing factor alpha (0.2 means: 20% new value, 80% history)
  const alpha = 0.2

  // Directly enforce updates on the global memory context to bypass reference drift
  self.learn.filtered_gradient = ((alpha * rawGradient) + ((1 - alpha) * (self.learn.filtered_gradient || 0)))

  if (self.learn.filtered_gradient > self.learn.max_gradient) {
    self.learn.max_gradient = self.learn.filtered_gradient
  }

  // Update tracking references for the next sample
  l.last_learn_temp = currentTemp
  l.last_learn_time = now

  // 4. Check for steady state (curve flattens out, gradient drops near 0)
  const isSteadyState = ((self.learn.filtered_gradient < 0.15) && ((now - self.learn.pwm_jump_time) / 1000 > (self.learn.tot_time + 5)))

  if (!isSteadyState) return  send([null, { payload: {
    event: "LEARN_RETURN 5", 
    filtered_gradient: self.learn.filtered_gradient,
    tot_time: self.learn.tot_time,
    dt_since_jump: (now - self.learn.pwm_jump_time) / 1000,
  } }])

  // Fix the deltaPwm calculation: If the system was armed in standby at max power,
  // the true physical step response jump was from 0% to the current active PWM value.
  const deltaPwm = (l.pwm_jump_val > 0) ? l.pwm_jump_val : 100
  const deltaTemp = currentTemp - l.temp_jump_val

  if ((0 < Math.abs(deltaPwm)) && (0 < Math.abs(l.max_gradient))) {
    const ks = deltaTemp / deltaPwm
    const t = (deltaTemp / l.max_gradient) - l.tot_time

    // Only allow parameter storage if the physical measurements are within realistic bounds
    if ((0.0 < ks) && (0.0 < t) && (t < 1200.0) && (l.tot_time < 60.0)) {
      self.config.learned_ks = ks
      self.config.learned_t = t
      self.config.learned_tot = l.tot_time

      // Trigger automatic tuning parameters
      tunePIDParameters(send)
    }
  }
}

function resetLearn() {
  if (self.learn.pwm_jump_time) return

  self.config.pid_kp = 1.0
  self.config.pid_ki = 0.05
  self.config.pid_kd = 0.5
  self.config.learned_ks = 0
  self.config.learned_t = 0
  self.config.learned_tot = 0
  self.integral = 0
  
  if (self.learn) {
    self.learn.pwm_jump_time = 0
    self.learn.pwm_jump_val = 0
    self.learn.temp_jump_val = 0
    self.learn.tot_time = 0
    self.learn.has_responded = false
    self.learn.max_gradient = 0
    self.learn.last_learn_temp = 0
    self.learn.last_learn_time = 0
    self.learn.filtered_gradient = 0
  }
}

function onBoilerTempIn(msg, send) {
  const config = self.config
  if (!config.pwm_control) return

  if ((false === self.config.is_learned) && !self.learn.pwm_jump_time) resetLearn()

  // Input data
  const processVariable = msg.payload // current temperature in 0.1 °C
  if (!Number.isFinite(processVariable)) return

  // Helper function for clipping values
  const clamp = (val, min, max) => {
    if (val > max) return max
    if (val < min) return min
    return val
  }

  // Determine tuning mode: force robust hardcoded defaults until system is fully learned
  let kp = 1.0
  let ki = 0.05
  let kd = 0.5

  if (true === config.is_learned) {
    kp = Number.isFinite(config.pid_kp) ? config.pid_kp : 1.0
    ki = Number.isFinite(config.pid_ki) ? config.pid_ki : 0.05
    kd = Number.isFinite(config.pid_kd) ? config.pid_kd : 0.5
  }

  // 1. HARD TUNING WALL: Force absolute 100% PWM during the ENTIRE learning phase
  // The PID controller is strictly forbidden from throttling down until is_learned becomes true
  const minPwm = (false === config.is_learned) ? 100 : (Number.isFinite(config.min_pwm) ? config.min_pwm : 0)
  const maxPwm = Number.isFinite(config.max_pwm) ? config.max_pwm : 100
  const hyst = Number.isFinite(config.hysteresis) ? config.hysteresis : 0.5

  const diff = (config.set_temp - processVariable)
  const tempSpread = processVariable - (self.temp_in || processVariable)
  const dt = (config.pid_dt && config.pid_dt > 0) ? config.pid_dt : 1e-6
  
  const proportional = (kp * diff)

  // 2. Dual Gradient Engine: Use raw data for physical identification, but filtered data for PID execution
  const rawGradient = (0 === self.temp_out) ? 0 : (processVariable - self.temp_out) / dt

  if ('undefined' === typeof self.filtered_gradient_mem) {
    self.filtered_gradient_mem = rawGradient
  }
  self.filtered_gradient_mem = (0.2 * rawGradient) + (0.8 * self.filtered_gradient_mem)

  // Identification module receives the true, unfiltered physics to capture the real maximum slope
  const identificationGradient = (false === config.is_learned) ? rawGradient : self.filtered_gradient_mem
  // Active PID controller receives the smooth, filtered gradient to prevent contactor chattering
  const currentGradient = self.filtered_gradient_mem

  // Environmental drift filtering: flow is only verified if temperature changes faster than 0.1°C/s
  // A gradient magnitude above 1.0 (tenth-degrees/second) clearly separates mechanical flow from static ambient cooling
  const isDynamicFlowDetected = (Math.abs(currentGradient) > 1.0)

  // Standby management: 5.0°C equals 50 in tenth-degrees
  // The system enters standby ONLY if it is cold AND static (no heavy dynamic gradient detected)
  const isHeaterHardwareOff = (processVariable < (config.set_temp - 50)) && (!isDynamicFlowDetected)

  // Overwrite history reference for the next iteration step
  self.temp_out = processVariable 

  if (isHeaterHardwareOff) {
    // Inject a massive integral charge to keep the PWM locked at 100% until water is hot
    self.integral = Number.isFinite(config.integral_limit) ? config.integral_limit : 100
    // Force 100% PWM during standby so the heater instantly fires the moment the mechanical contactor closes
    if ('undefined' === typeof self.pwm) self.pwm = config.max_pwm || 100 
    self.was_in_standby = true 
    msg.self_pwm = self.pwm
    msg.payload = Math.round(self.pwm * 10.23)
    msg.status = 'HARDWARE_STANDBY'
    send([msg, msg])
    return
  }

  // Hardware Start Boost: Force 100% PWM immediately on the very first event after standby
  if (true === self.was_in_standby) {
    self.was_in_standby = false 
    // Inject a massive integral charge to keep the PWM locked at 100% until water is hot
    self.integral = Number.isFinite(config.integral_limit) ? config.integral_limit : 100
    if ('undefined' === typeof self.pwm) self.pwm = config.max_pwm || 100 

    msg.self_pwm = self.pwm
    msg.payload = Math.round(self.pwm * 10.23)
    msg.status = 'HARDWARE_WAKEUP_BOOST'

    self.prevError = diff
    send([msg, msg])
    return
  }

  // Derivative term outputs direct % PWM corrected for the 0.1 scale and phase alignment
  // Derivative term acts as a dampening brake against rapid temperature changes
  const derivative = -kd * (currentGradient / 10)

  // 2. Integral accumulation in direct % PWM scale
  const potentialIntegral = self.integral + (diff * config.pid_dt)
  const potentialIntegralTerm = ki * potentialIntegral

  // 3. Total ideal PWM output in direct % scale (Value range 0 to 100)
  const potentialPwm = proportional + potentialIntegralTerm + derivative

  const clippedPwm = clamp(potentialPwm, minPwm, maxPwm)

  // 4. Saturation and safety checks using direct % scale
  const isSaturated = potentialPwm !== clippedPwm
  const isDrivingDeeper = (0 !== ki) && (Math.sign(diff) === Math.sign(ki))
  const nearLimit = Math.abs(potentialPwm - clippedPwm) < hyst
  const isHeaterFailing = (potentialPwm >= maxPwm) && (0 >= tempSpread)
  
  const antiWindupBlock = (isSaturated && isDrivingDeeper && !nearLimit) || isHeaterFailing

  if (!antiWindupBlock) {
    self.integral = potentialIntegral
  }

  // 5. Secondary hard-clamp protection for the integral state
  const integralLimit = Number.isFinite(config.integral_limit)
    ? Math.abs(config.integral_limit)
    : Math.abs(maxPwm / Math.max(Math.abs(ki), 1e-6))

  if (self.integral > integralLimit) self.integral = integralLimit
  else if (self.integral < -integralLimit) self.integral = -integralLimit

  // 6. Compute final PWM and clamp to hardware boundaries
  const integralTerm = ki * self.integral
  const rawPwm = proportional + integralTerm + derivative
  
  self.pwm = clamp(rawPwm, minPwm, maxPwm)
  self.config.last_pwm = self.pwm

  // persist error for the next iteration
  self.prevError = diff

  // Execute parameter identification during the entire tuning run
  if (false === self.config.is_learned) {
    estimateSystemParameters(processVariable, send)
  }

  // Debug
  msg.error = diff
  msg.proportional = Math.round(proportional * 100) / 100
  msg.integral = Math.round(integralTerm * 100) / 100
  msg.derivative = Math.round(derivative * 100) / 100

  // Output data
  msg.self_pwm = self.pwm
  msg.payload = Math.round(self.pwm * 10.23)
  msg.d_temp = diff

  // Prepare comprehensive diagnostic message for Output 2 (File Node)
  const l = self.learn
  const diagnosticMsg = {
    event: "ITERATION_LOG",
    timestamp: Date.now(),
    time_string: new Date().toISOString(),
    process_variable: processVariable,
    inlet_temp: self.temp_in || 0,
    temp_spread: tempSpread,
    control_error: diff,
    calculated_pwm: self.pwm,
    proportional_term: Math.round(proportional * 100) / 100,
    integral_term: Math.round(integralTerm * 100) / 100,
    derivative_term: Math.round(derivative * 100) / 100,
    raw_gradient: Math.round((rawGradient  || 0) * 10000) / 10000,
    filtered_gradient: Math.round((l.filtered_gradient || 0) * 10000) / 10000,
    max_gradient: Math.round((l.max_gradient || 0) * 10000) / 10000,
    has_responded: l.has_responded ? 1 : 0,
    dead_time: l.tot_time,
    config_kp: config.pid_kp,
    config_ki: config.pid_ki,
    config_kd: config.pid_kd,
    config_dt: config.pid_dt,
    learned_ks: config.learned_ks,
    learned_t: config.learned_t,
    learned_tot: config.learned_tot,
    is_learned: config.is_learned,
  }
  send([msg, {payload: diagnosticMsg}])
}

function onConfig(config, send) {
  if (!config || ('object' !== typeof config)) return
  
  // Hard Reset Trigger: If system is not learned, instantly wipe out constants 
  // and restore secure hardware default values directly inside the registry
  if (false === self.config.is_learned) resetLearn()

  // Process the rest of your standard incoming configuration fields...
  if (Number.isFinite(config.set_temp)) self.config.set_temp = config.set_temp

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
  send([null, { payload: configLog }])
}

function send(msg) {
  node.send(msg)
}

function onInput(msg, send, done = () => {}) {
  Object.assign(self, context.get('boiler') || {})
  switch (msg.topic) {
    case 'config':
      onConfig(msg.payload, send)
      break
    case 'learn':
      self.config.is_learned = msg.payload
      self.learn.pwm_jump_time = 0
      onConfig({}, send)
      break
    case 'boiler_temp_in':
      self.temp_in = msg.payload // Store the inlet value safely in context memory
      
      // Protect PID calculus: overwrite payload with last known outlet temperature
      // before forcing the execution loop to update dynamic flow spreads instantly
      msg.payload = (self.temp_out || self.config.set_temp)
      onBoilerTempIn(msg, send)
      break
    case 'boiler_temp_out':
      // Adaptive Jitter Filter: Apply deadband ONLY when system is fully learned
      // This protects the mechanical contactor during daily operation, but keeps events flowing during tuning
      if (true === self.config.is_learned) {
        const lastOutlet = self.temp_out || 0
        const jitterThreshold = 1 // 1 equals 0.1 °C
        
        if ((0 !== lastOutlet) && (Math.abs(msg.payload - lastOutlet) <= jitterThreshold)) {
          break // Suppress message execution to protect hardware from jitter
        }
      }
      onBoilerTempIn(msg, send)
      break
    case 'boiler_update':
      // 1. Thermodynamic Wakeup Trigger via Inlet Temperature Drop
      if (Number.isFinite(msg.payload.temp_in)) {
        const lastInlet = (self.temp_in || msg.payload.temp_in)
        self.temp_in = msg.payload.temp_in
        
        // If the inlet drops by more than 0.2°C, water is flowing! Break out of standby instantly.
        if (((lastInlet - msg.payload.temp_in) > 2) && (true === self.was_in_standby)) {
          self.was_in_standby = false
          self.integral = 0
          self.pwm = (self.config.max_pwm || 100)
          
          const wakeupMsg = {
            topic: "boiler_temp_out",
            payload: Math.round(self.pwm * 10.23),
            status: "HARDWARE_WAKEUP_INLET_BOOST"
          }
          send([wakeupMsg, null])
          break // Exit early, the immediate 100% boost is fired to lock the contactor
        }
      }
      
      // 2. Primary PID Loop Execution on Outlet Changes
      // If temp_out is missing in this frame, load the last known value from memory to prevent math crashes
      if (Number.isFinite(msg.payload.temp_out)) {
        self.temp_out_buffer = msg.payload.temp_out
      }
      
      if (Number.isFinite(self.temp_out_buffer)) {
        msg.payload = self.temp_out_buffer
        onBoilerTempIn(msg, send)
      }
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