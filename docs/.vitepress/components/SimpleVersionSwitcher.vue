<template>
  <div class="version-switcher">
    <select v-model="currentVersion" @change="changeVersion">
      <option value="latest">Latest</option>
      <option v-for="version in availableVersions" :key="version" :value="version">
        v{{ version }}
      </option>
    </select>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'

const currentVersion = ref('latest')
const availableVersions = ref([])

async function loadVersions() {
  try {
    // Try to load versions from a JSON file or API
    const response = await fetch('/versions.json')
    if (response.ok) {
      const data = await response.json()
      availableVersions.value = data.versions || []
    } else {
      // Fallback to hardcoded versions
      availableVersions.value = ['1.0', '1.3', '2.0']
    }
  } catch (error) {
    // Fallback to hardcoded versions
    availableVersions.value = ['1.0', '1.3', '2.0']
  }
}

function changeVersion() {
  if (currentVersion.value === 'latest') {
    window.location.href = '/'
  } else {
    window.location.href = `/v${currentVersion.value}/`
  }
}

onMounted(() => {
  loadVersions()
  
  // Detect current version from URL
  const path = window.location.pathname
  const match = path.match(/\/v(\d+\.\d+)/)
  if (match) {
    currentVersion.value = match[1]
  }
})
</script>

<style scoped>
.version-switcher {
  display: inline-block;
  margin-left: 1rem;
}

.version-switcher select {
  padding: 4px 8px;
  border: 1px solid var(--vp-c-divider);
  border-radius: 4px;
  background: var(--vp-c-bg);
  color: var(--vp-c-text-1);
  font-size: 14px;
  cursor: pointer;
}

.version-switcher select:hover {
  border-color: var(--vp-c-brand);
}
</style> 