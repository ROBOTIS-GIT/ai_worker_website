#!/usr/bin/env node

const { execSync } = require('child_process')
const fs = require('fs')
const path = require('path')

// Get version from command line argument or git tag
const version = process.argv[2] || getLatestGitTag()

if (!version) {
  console.error('❌ No version specified and no git tags found')
  console.log('Usage: node scripts/simple-deploy.js [version]')
  console.log('Example: node scripts/simple-deploy.js 1.0')
  process.exit(1)
}

console.log(`🚀 Building version ${version}...`)

try {
  // Set environment variable for build
  process.env.VITE_VERSION_NUMBER = version
  
  // Build the site
  execSync('npm run docs:build', { stdio: 'inherit' })
  
  // Create version directory
  const versionDir = path.join('docs/.vitepress/dist', `v${version}`)
  if (!fs.existsSync(versionDir)) {
    fs.mkdirSync(versionDir, { recursive: true })
  }
  
  // Copy built files to version directory (excluding all version directories)
  const distDir = 'docs/.vitepress/dist'
  const files = fs.readdirSync(distDir)
  
  files.forEach(file => {
    // Skip all version directories
    if (file.startsWith('v')) {
      return
    }
    
    const sourcePath = path.join(distDir, file)
    const destPath = path.join(versionDir, file)
    
    if (fs.statSync(sourcePath).isDirectory()) {
      execSync(`cp -r "${sourcePath}" "${destPath}"`, { stdio: 'inherit' })
    } else {
      execSync(`cp "${sourcePath}" "${destPath}"`, { stdio: 'inherit' })
    }
  })
  
  console.log(`✅ Version ${version} built successfully!`)
  console.log(`📁 Files copied to: ${versionDir}`)
  
  // List all available versions
  const allVersions = fs.readdirSync(distDir)
    .filter(dir => dir.startsWith('v') && fs.statSync(path.join(distDir, dir)).isDirectory())
    .map(dir => dir.substring(1))
    .sort()
  
  console.log(`📋 All available versions: ${allVersions.join(', ')}`)
  
} catch (error) {
  console.error('❌ Build failed:', error.message)
  process.exit(1)
}

function getLatestGitTag() {
  try {
    const tags = execSync('git tag --sort=-version:refname', { encoding: 'utf8' })
    const latestTag = tags.split('\n')[0].trim()
    return latestTag.replace('v', '') // Remove 'v' prefix
  } catch (error) {
    return null
  }
} 