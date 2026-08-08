<script setup lang="ts">
import { onMounted, ref } from 'vue'

const uptime = ref(0)
const voltage = ref(0)

onMounted(() => {
    const eventSource = new EventSource('/api/stream')
    eventSource.onmessage = (event) => {
        const frame = JSON.parse(event.data)
        uptime.value = frame.uptime
        voltage.value = frame.minimum_voltage
    }
})
</script>

<template>
    <h1>Uptime: {{ uptime }}</h1>
    <h1>LVS Voltage: {{ voltage }}</h1>
</template>
