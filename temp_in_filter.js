if (!msg.payload || ('object' !== typeof msg.payload)) return null

function send_temp(name, temp, time) {
  const temp_val = Math.round(temp * 10)
  if (!temp_val) return

  const temp_key = `temp_${name}`
  const last_temp = context.get(temp_key) || 0
  context.set(temp_key, temp_val)

  const time_key = `time_${name}`
  const last_time = context.get(time_key) || 0
  context.set(time_key, time)

  node.send({
    topic: `boiler_temp_${name}`,
    payload: temp_val, // in 0.1 °C
    d_temp: last_temp ? (temp_val - last_temp) : 0, // in 0.1 °C
    d_time: last_time ? (new Date(time) - new Date(last_time)) / 1000 : 0
  })
}

for (const key of Object.keys(msg.payload)) {
  if (!key.startsWith('DS18B20')) continue

  const data = msg.payload[key]
  const temp = data.Temperature
  const time = msg.payload.Time
  if (!Number.isFinite(temp)) continue

  if ('0316A2794CB7' === data.Id) send_temp('in', temp, time)
  else if ('0316A2790967' === data.Id) send_temp('out', temp, time)
}
