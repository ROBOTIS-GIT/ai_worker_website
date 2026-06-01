<script setup>
import { onMounted, ref, watch } from 'vue'
import { useRoute } from 'vitepress'

const isVisible = ref(false)
const dailyDismissKey = 'robotis-docs-migration-notice-dismissed-date'
const docsUrl = 'https://docs.robotis.com/'
const route = useRoute()

onMounted(() => {
  showNotice()
})

watch(
  () => route.path,
  () => {
    showNotice()
  }
)

function getTodayKey() {
  return new Date().toISOString().slice(0, 10)
}

function closeNotice() {
  isVisible.value = false
}

function showNotice() {
  isVisible.value = localStorage.getItem(dailyDismissKey) !== getTodayKey()
}

function dismissForToday() {
  localStorage.setItem(dailyDismissKey, getTodayKey())
  closeNotice()
}
</script>

<template>
  <Teleport to="body">
    <div
      v-if="isVisible"
      class="docs-migration-notice"
      role="dialog"
      aria-modal="false"
      aria-labelledby="docs-migration-notice-title"
    >
      <div class="docs-migration-notice__panel">
        <button class="docs-migration-notice__close" type="button" aria-label="Close notice" @click="closeNotice">
          x
        </button>
        <p class="docs-migration-notice__eyebrow">Notice</p>
        <h2 id="docs-migration-notice-title">AI Worker documentation has moved</h2>
        <p>
          The AI Worker website and e-Manual have been integrated into ROBOTIS Docs.
          Please use the new Docs site for the latest AI Worker documentation.
        </p>
        <p class="docs-migration-notice__warning">
          This AI Worker website will be removed in July.
        </p>
        <div class="docs-migration-notice__actions">
          <a class="docs-migration-notice__primary" :href="docsUrl">Go to ROBOTIS Docs</a>
          <button class="docs-migration-notice__secondary" type="button" @click="closeNotice">
            Continue here
          </button>
          <button class="docs-migration-notice__tertiary" type="button" @click="dismissForToday">
            Don't show again today
          </button>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<style scoped>
.docs-migration-notice {
  position: fixed;
  top: 24px;
  left: 24px;
  z-index: 1000;
  width: min(520px, calc(100vw - 48px));
}

.docs-migration-notice__panel {
  position: relative;
  padding: 32px;
  border: 1px solid var(--vp-c-divider);
  border-radius: 18px;
  background: var(--vp-c-bg);
  color: var(--vp-c-text-1);
  box-shadow: 0 18px 60px rgba(0, 0, 0, 0.24);
}

.docs-migration-notice__close {
  position: absolute;
  top: 14px;
  right: 14px;
  width: 32px;
  height: 32px;
  border: 0;
  border-radius: 50%;
  background: var(--vp-c-bg-soft);
  color: var(--vp-c-text-2);
  cursor: pointer;
  font-size: 18px;
  line-height: 1;
}

.docs-migration-notice__close:hover {
  color: var(--vp-c-text-1);
  background: var(--vp-c-bg-mute);
}

.docs-migration-notice__eyebrow {
  margin: 0 0 8px;
  color: var(--vp-c-brand-1);
  font-size: 13px;
  font-weight: 700;
  letter-spacing: 0.08em;
  text-transform: uppercase;
}

.docs-migration-notice h2 {
  margin: 0 24px 14px 0;
  font-size: 28px;
  line-height: 1.2;
}

.docs-migration-notice p {
  margin: 0 0 14px;
  color: var(--vp-c-text-2);
  line-height: 1.65;
}

.docs-migration-notice__warning {
  font-weight: 700;
  color: var(--vp-c-text-1) !important;
}

.docs-migration-notice__actions {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  margin-top: 24px;
}

.docs-migration-notice__primary,
.docs-migration-notice__secondary,
.docs-migration-notice__tertiary {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  min-height: 42px;
  padding: 0 18px;
  border-radius: 999px;
  font-weight: 700;
  text-decoration: none;
  cursor: pointer;
}

.docs-migration-notice__primary {
  border: 1px solid var(--vp-c-brand-1);
  background: var(--vp-c-brand-1);
  color: #fff;
}

.docs-migration-notice__primary:hover {
  background: var(--vp-c-brand-2);
  text-decoration: none;
}

.docs-migration-notice__secondary {
  border: 1px solid var(--vp-c-divider);
  background: var(--vp-c-bg-soft);
  color: var(--vp-c-text-1);
}

.docs-migration-notice__secondary:hover {
  background: var(--vp-c-bg-mute);
}

.docs-migration-notice__tertiary {
  border: 0;
  background: transparent;
  color: var(--vp-c-text-2);
  padding-inline: 6px;
}

.docs-migration-notice__tertiary:hover {
  color: var(--vp-c-text-1);
  text-decoration: underline;
}

@media (max-width: 640px) {
  .docs-migration-notice {
    top: 16px;
    left: 16px;
    width: calc(100vw - 32px);
  }

  .docs-migration-notice__panel {
    padding: 26px 22px;
  }

  .docs-migration-notice h2 {
    font-size: 23px;
  }
}
</style>
