#!/usr/bin/env node

const express = require('express')
const path = require('path')
const fs = require('fs')

const app = express()
const PORT = 3000

// Serve static files from the dist directory
app.use(express.static(path.join(__dirname, '../docs/.vitepress/dist')))

// Handle versioned routes
app.get('/v:version', (req, res) => {
  const version = req.params.version
  const versionPath = path.join(__dirname, '../docs/.vitepress/dist', `v${version}`)
  
  if (fs.existsSync(versionPath)) {
    res.sendFile(path.join(versionPath, 'index.html'))
  } else {
    res.status(404).send('Version not found')
  }
})

app.get('/v:version/:path(*)', (req, res) => {
  const version = req.params.version
  const filePath = req.params.path
  const versionPath = path.join(__dirname, '../docs/.vitepress/dist', `v${version}`)
  
  if (fs.existsSync(versionPath)) {
    const fullPath = path.join(versionPath, filePath)
    if (fs.existsSync(fullPath)) {
      res.sendFile(fullPath)
    } else {
      // Fallback to version's index.html for SPA routing
      res.sendFile(path.join(versionPath, 'index.html'))
    }
  } else {
    res.status(404).send('Version not found')
  }
})

// Handle root and other routes
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, '../docs/.vitepress/dist/index.html'))
})

app.listen(PORT, () => {
  console.log(`🚀 Version server running at http://localhost:${PORT}`)
  console.log(`📖 Latest version: http://localhost:${PORT}/`)
  console.log(`📖 Version 1.0: http://localhost:${PORT}/v1.0/`)
  
  // List available versions
  const distDir = path.join(__dirname, '../docs/.vitepress/dist')
  const versions = fs.readdirSync(distDir)
    .filter(dir => dir.startsWith('v') && fs.statSync(path.join(distDir, dir)).isDirectory())
    .map(dir => dir.substring(1))
  
  if (versions.length > 0) {
    console.log(`📋 Available versions: ${versions.join(', ')}`)
  }
}) 