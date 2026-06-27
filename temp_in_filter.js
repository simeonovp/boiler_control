if (!msg.payload || ('object' !== typeof msg.payload)) return null

for (const key of Object.keys(msg.payload)) {
  if (!key.startsWith('DS18B20')) continue

  const data = msg.payload[key]
  const temp = data.Temperature
  if (!Number.isFinite(temp)) continue

  if ('0316A2794CB7' === data.Id) {
    const temp_in = Math.round(temp * 10)
    if (!temp_in) continue

    const last_temp_in = context.get('temp_in') || 0
    context.set('temp_in', temp_in)

    const msg_in = {
      topic: 'boiler_temp_in',
      payload: temp_in, // in 0.1 °C
      d_temp: last_temp_in ? (temp_in - last_temp_in) : 0 // in 0.1 °C
    }
    node.send(msg_in)
  }
  else if ('0316A2790967' === data.Id) {
    const temp_out = Math.round(temp * 10)
    if (!temp_out) continue

    const last_temp_out = context.get('temp_out') || 0
    context.set('temp_out', temp_out)

    const time = msg.payload.Time
    const last_time_out = context.get('time_out') || 0
    context.set('time_out', time)

    const msg_out = {
      topic: 'boiler_temp_out',
      payload: temp_out, // in 0.1 °C
      d_temp: last_temp_out ? (temp_out - last_temp_out) : 0, // in 0.1 °C
      d_time: last_time_out ? (new Date(time) - new Date(last_time_out)) / 1000 : 0
    }
    node.send(msg_out)
  }
}
