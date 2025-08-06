#!/usr/bin/env node

const { execSync } = require('child_process')
const fs = require('fs')
const path = require('path')

function getGitTags() {
  try {
    const tags = execSync('git tag -l "v*"', { encoding: 'utf8' })
    return tags.split('\n')
      .filter(tag => tag.trim())
      .map(tag => tag.replace('v', ''))
      .sort((a, b) => {
        const aParts = a.split('.').map(Number)
        const bParts = b.split('.').map(Number)
        for (let i = 0; i < Math.max(aParts.length, bParts.length); i++) {
          const aVal = aParts[i] || 0
          const bVal = bParts[i] || 0
          if (aVal !== bVal) return aVal - bVal
        }
        return 0
      })
  } catch (error) {
    console.error('❌ Failed to get Git tags:', error.message)
    return []
  }
}

function generateVersionsJson() {
  const versions = getGitTags()
  
  if (versions.length === 0) {
    console.log('⚠️ No Git tags found. Creating empty versions.json')
    const versionsData = {
      versions: [],
      current: null,
      latest: null,
      generated_at: new Date().toISOString(),
      source: "git_tags"
    }
    return versionsData
  }
  
  const latestVersion = versions[versions.length - 1]
  
  const versionsData = {
    versions: versions,
    current: latestVersion,
    latest: latestVersion,
    generated_at: new Date().toISOString(),
    source: "git_tags"
  }
  
  return versionsData
}

// Generate and write versions.json
const versionsData = generateVersionsJson()
const outputPath = path.join('docs', 'public', 'versions.json')

// Ensure directory exists
const dir = path.dirname(outputPath)
if (!fs.existsSync(dir)) {
  fs.mkdirSync(dir, { recursive: true })
}

fs.writeFileSync(outputPath, JSON.stringify(versionsData, null, 2))

console.log('✅ Generated versions.json:')
console.log(`📋 Available versions: ${versionsData.versions.join(', ')}`)
console.log(`📁 Output: ${outputPath}`) 