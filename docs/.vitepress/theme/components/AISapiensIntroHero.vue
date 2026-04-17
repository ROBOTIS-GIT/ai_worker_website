<script setup>
const bannerImageSrc = '/ai_sapiens/ai_sapiens_banner.png'

</script>

<template>
  <section class="ai-sapiens-intro-hero" aria-labelledby="ai-sapiens-hero-heading">
    <div class="container">
      <div class="main">
        <p class="eyebrow">ROBOTIS · Humanoid</p>
        <div id="ai-sapiens-hero-heading" class="heading" role="heading" aria-level="2">
          <span class="name">Humanoid</span>
          <span class="text">AI Sapiens</span>
        </div>
        <p class="tagline">
          Open-Source Humanoid Platform for Physical AI powered by DYNAMIXEL-Q.
        </p>
        <ul class="stat-chips" aria-label="K0 highlights">
          <li v-for="s in stats" :key="s.label" class="stat-chip">
            <span class="stat-chip-value">{{ s.value }}</span>
            <span class="stat-chip-label">{{ s.label }}</span>
          </li>
        </ul>
      </div>
      <div class="image">
        <div class="image-container">
          <img
            :src="bannerImageSrc"
            alt="ROBOTIS AI Sapiens"
            class="image-src"
          />
        </div>
      </div>
    </div>
  </section>
</template>

<style scoped>
/* Bleed to doc content edges (VPDoc default horizontal padding ~24px) */
.ai-sapiens-intro-hero {
  margin: 0 -24px 24px;
  padding: 20px 24px 0;
  background: linear-gradient(
    180deg,
    var(--vp-c-bg-soft) 0%,
    color-mix(in srgb, var(--vp-c-bg-soft) 55%, var(--vp-c-bg) 45%) 100%
  );
  border: none;
  border-bottom: 1px solid var(--vp-c-divider);
  box-sizing: border-box;
}

@media (prefers-reduced-motion: no-preference) {
  .ai-sapiens-intro-hero {
    animation: aiSapiensHeroIn 0.55s ease-out both;
  }

  @keyframes aiSapiensHeroIn {
    from {
      opacity: 0.92;
    }
    to {
      opacity: 1;
    }
  }
}

@media (min-width: 640px) {
  .ai-sapiens-intro-hero {
    margin-bottom: 32px;
    padding-top: 24px;
    padding-bottom: 0;
  }
}

@media (min-width: 960px) {
  .ai-sapiens-intro-hero {
    margin: 0 -32px 40px;
    padding: 28px 32px 0;
  }
}

.container {
  display: flex;
  flex-direction: column;
  margin: 0 auto;
  max-width: 1152px;
  align-items: flex-start;
  text-align: center;
}

@media (min-width: 960px) {
  .container {
    flex-direction: row;
    text-align: left;
    align-items: flex-start;
  }
}

.main {
  position: relative;
  z-index: 10;
  order: 1;
  flex-grow: 1;
  flex-shrink: 1;
  min-width: 0;
  max-width: 100%;
  width: 100%;
  box-sizing: border-box;
  overflow-wrap: break-word;
  word-wrap: break-word;
}

@media (min-width: 960px) {
  .main {
    width: calc((100% / 3) * 2);
    max-width: 592px;
  }
}

.eyebrow {
  margin: 0 auto 4px;
  padding: 0;
  font-size: 12px;
  font-weight: 600;
  letter-spacing: 0.12em;
  text-transform: uppercase;
  color: var(--vp-c-text-3);
  line-height: 1.4;
}

@media (min-width: 960px) {
  .eyebrow {
    margin-left: 0;
    margin-right: 0;
  }
}

.heading {
  display: flex;
  flex-direction: column;
  align-items: center;
  max-width: 100%;
  margin: 0;
  padding: 0;
  border: none;
}

@media (min-width: 960px) {
  .heading {
    align-items: flex-start;
  }
}

.name,
.text {
  box-sizing: border-box;
  letter-spacing: -0.4px;
  line-height: 40px;
  font-size: 32px;
  font-weight: 700;
}

/* 한 줄 유지 (VPDoc h2 스타일 미적용: div.heading 사용) */
.name,
.text {
  white-space: nowrap;
  width: max-content;
  max-width: 100%;
  word-break: normal;
  overflow-wrap: normal;
}

.name {
  color: var(--vp-home-hero-name-color);
  background: var(--vp-home-hero-name-background, inherit);
  -webkit-background-clip: text;
  background-clip: text;
  -webkit-text-fill-color: var(--vp-home-hero-name-color);
}

.name,
.text {
  margin-left: auto;
  margin-right: auto;
}

@media (min-width: 640px) {
  .name,
  .text {
    line-height: 56px;
    font-size: 48px;
  }
}

@media (min-width: 960px) {
  .name,
  .text {
    margin-left: 0;
    margin-right: 0;
    line-height: 56px;
    font-size: 48px;
  }
}

.tagline {
  padding-top: 10px;
  max-width: 100%;
  line-height: 26px;
  font-size: 17px;
  font-weight: 500;
  white-space: normal;
  overflow-wrap: break-word;
  word-break: break-word;
  color: var(--vp-c-text-2);
  margin: 0 auto;
  box-sizing: border-box;
}

.subline {
  margin: 6px auto 0;
  padding: 0;
  font-size: 15px;
  line-height: 24px;
  font-weight: 500;
  color: var(--vp-c-text-2);
}

@media (min-width: 640px) {
  .tagline {
    padding-top: 12px;
    line-height: 30px;
    font-size: 19px;
  }

  .subline {
    font-size: 16px;
    line-height: 26px;
  }
}

@media (min-width: 960px) {
  .tagline {
    line-height: 32px;
    font-size: 20px;
    margin: 0;
  }

  .subline {
    margin-left: 0;
    margin-right: 0;
  }
}

.stat-chips {
  display: flex;
  flex-wrap: wrap;
  gap: 8px 10px;
  justify-content: center;
  list-style: none;
  margin: 14px auto 0;
  padding: 0;
  max-width: 100%;
}

@media (min-width: 960px) {
  .stat-chips {
    justify-content: flex-start;
    margin-left: 0;
    margin-right: 0;
  }
}

.stat-chip {
  display: inline-flex;
  flex-direction: column;
  align-items: flex-start;
  margin: 0;
  padding: 8px 12px;
  border-radius: 8px;
  background: color-mix(in srgb, var(--vp-c-bg) 70%, var(--vp-c-bg-soft));
  border: 1px solid var(--vp-c-divider);
  text-align: left;
  min-width: 0;
}

.stat-chip-value {
  font-size: 14px;
  font-weight: 700;
  letter-spacing: -0.02em;
  color: var(--vp-c-text-1);
  line-height: 1.25;
}

.stat-chip-label {
  font-size: 10px;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.06em;
  color: var(--vp-c-text-3);
  margin-top: 2px;
  line-height: 1.2;
}

@media (min-width: 640px) {
  .stat-chip {
    padding: 9px 14px;
  }

  .stat-chip-value {
    font-size: 15px;
  }

  .stat-chip-label {
    font-size: 11px;
  }
}

.image {
  order: 2;
  margin: 20px 0 0;
  flex-shrink: 0;
  align-self: stretch;
  width: 100%;
}

@media (min-width: 960px) {
  .image {
    flex-grow: 1;
    order: 2;
    margin: 0;
    min-height: 100%;
    align-self: stretch;
    width: auto;
  }
}

.image-container {
  position: relative;
  margin: 0;
  padding: 0;
  width: 100%;
  max-width: none;
  height: auto;
  display: flex;
  align-items: flex-end;
  justify-content: flex-end;
  line-height: 0;
}

@media (min-width: 960px) {
  .image-container {
    width: 100%;
    max-width: none;
    height: 100%;
    align-items: flex-end;
    justify-content: flex-end;
  }
}

.image-src {
  max-width: 220px;
  max-height: 220px;
  width: auto;
  height: auto;
  margin: 0;
  padding: 0;
  object-fit: contain;
  display: block;
}

@media (min-width: 640px) {
  .image-src {
    max-width: 300px;
    max-height: 300px;
  }
}

@media (min-width: 960px) {
  .image-src {
    max-width: 400px;
    max-height: 400px;
  }
}
</style>
