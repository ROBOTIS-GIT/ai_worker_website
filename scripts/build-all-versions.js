#!/usr/bin/env node

const { execSync } = require('child_process')
const fs = require('fs')
const path = require('path')

// Get all Git tags
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

// Generate versions.json
function generateVersionsJson(versions) {
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

const versions = getGitTags()

if (versions.length === 0) {
  console.error('❌ No Git tags found. Please create tags first:')
  console.log('git tag v1.0 -m "Release v1.0"')
  console.log('git tag v1.1 -m "Release v1.1"')
  console.log('git tag v2.0 -m "Release v2.0"')
  process.exit(1)
}

console.log(`🚀 Building versions from Git tags: ${versions.join(', ')}`)

// Create a temporary directory for all builds
const tempDir = path.join('docs/.vitepress', 'temp-builds')
if (!fs.existsSync(tempDir)) {
  fs.mkdirSync(tempDir, { recursive: true })
}

try {
  // Build each version from its Git tag
  for (const version of versions) {
    console.log(`\n📦 Building version ${version} from Git tag v${version}...`)
    
    // Checkout the specific tag
    try {
      execSync(`git checkout v${version}`, { stdio: 'inherit' })
      console.log(`✅ Checked out tag v${version}`)
    } catch (error) {
      console.error(`❌ Failed to checkout tag v${version}:`, error.message)
      continue
    }
    
    // Set environment variable for build
    process.env.VITE_VERSION_NUMBER = version
    
    // Create version-specific config
    const configPath = 'docs/.vitepress/config.mts'
    const originalConfig = fs.readFileSync(configPath, 'utf8')
    
    // Add base path for versioned builds
    const versionConfig = originalConfig.replace(
      'export default defineConfig({',
      `export default defineConfig({
  base: '/v${version}/',`
    )
    
    // Write temporary config
    fs.writeFileSync(configPath, versionConfig)
    
    // Build the site
    execSync('npm run docs:build', { stdio: 'inherit' })
    
    // Restore original config
    fs.writeFileSync(configPath, originalConfig)
    
    // Copy to temp directory
    const tempVersionDir = path.join(tempDir, `v${version}`)
    if (fs.existsSync(tempVersionDir)) {
      execSync(`rm -rf "${tempVersionDir}"`, { stdio: 'inherit' })
    }
    execSync(`cp -r docs/.vitepress/dist "${tempVersionDir}"`, { stdio: 'inherit' })
    
    console.log(`✅ Version ${version} built and saved`)
  }
  
  // Return to main branch
  try {
    execSync('git checkout main', { stdio: 'inherit' })
    console.log('✅ Returned to main branch')
  } catch (error) {
    try {
      execSync('git checkout master', { stdio: 'inherit' })
      console.log('✅ Returned to master branch')
    } catch (error2) {
      console.warn('⚠️ Could not return to main/master branch')
    }
  }
  
  // Create final dist directory
  const finalDistDir = 'docs/.vitepress/dist'
  if (fs.existsSync(finalDistDir)) {
    execSync(`rm -rf "${finalDistDir}"`, { stdio: 'inherit' })
  }
  fs.mkdirSync(finalDistDir, { recursive: true })
  
  // Copy all versions to final dist
  for (const version of versions) {
    const tempVersionDir = path.join(tempDir, `v${version}`)
    const finalVersionDir = path.join(finalDistDir, `v${version}`)
    
    if (fs.existsSync(tempVersionDir)) {
      execSync(`cp -r "${tempVersionDir}" "${finalVersionDir}"`, { stdio: 'inherit' })
      console.log(`📁 Copied v${version} to final dist`)
    }
  }
  
  // Copy latest version (use the last one) to root
  const latestVersion = versions[versions.length - 1]
  const latestTempDir = path.join(tempDir, `v${latestVersion}`)
  execSync(`cp -r "${latestTempDir}"/* "${finalDistDir}/"`, { stdio: 'inherit' })
  console.log(`📁 Copied v${latestVersion} as latest version`)
  
  // Generate versions.json
  const versionsData = generateVersionsJson(versions)
  const versionsJsonPath = path.join(finalDistDir, 'versions.json')
  fs.writeFileSync(versionsJsonPath, JSON.stringify(versionsData, null, 2))
  console.log(`📄 Generated versions.json with ${versions.length} versions`)
  
  // Clean up temp directory
  execSync(`rm -rf "${tempDir}"`, { stdio: 'inherit' })
  
  console.log(`\n🎉 All versions built successfully from Git tags!`)
  console.log(`📋 Available versions: ${versions.join(', ')}`)
  console.log(`📁 Final dist directory: ${finalDistDir}`)
  
} catch (error) {
  console.error('❌ Build failed:', error.message)
  process.exit(1)
} 