<template>
  <DefaultTheme.Layout>
    <template #nav-bar-content-after>
      <SimpleVersionSwitcher />
    </template>
  </DefaultTheme.Layout>
</template>

<script setup>
import DefaultTheme from 'vitepress/theme'
import SimpleVersionSwitcher from '../components/SimpleVersionSwitcher.vue'
</script> 