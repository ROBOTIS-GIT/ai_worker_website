#!/usr/bin/env python3

import http.server
import socketserver
import os
import urllib.parse
from pathlib import Path

PORT = 3000
DIST_DIR = Path(__file__).parent.parent / 'docs' / '.vitepress' / 'dist'

class VersionAwareHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIST_DIR), **kwargs)
    
    def do_GET(self):
        # Parse the URL
        parsed_path = urllib.parse.urlparse(self.path)
        path = parsed_path.path
        
        # Check if this is a versioned path
        if path.startswith('/v'):
            # Extract version from path like /v1.0/something
            parts = path.split('/')
            if len(parts) >= 2 and parts[1].startswith('v'):
                version = parts[1][1:]  # Remove 'v' prefix
                version_dir = DIST_DIR / f'v{version}'
                
                if version_dir.exists():
                    # Reconstruct the path relative to version directory
                    relative_path = '/'.join(parts[2:]) if len(parts) > 2 else ''
                    file_path = version_dir / relative_path
                    
                    if file_path.exists() and file_path.is_file():
                        # Serve the file
                        self.send_response(200)
                        self.send_header('Content-type', self.guess_type(str(file_path)))
                        self.end_headers()
                        with open(file_path, 'rb') as f:
                            self.wfile.write(f.read())
                        return
                    else:
                        # Fallback to version's index.html for SPA routing
                        index_path = version_dir / 'index.html'
                        if index_path.exists():
                            self.send_response(200)
                            self.send_header('Content-type', 'text/html')
                            self.end_headers()
                            with open(index_path, 'rb') as f:
                                self.wfile.write(f.read())
                            return
        
        # Default behavior for non-versioned paths
        super().do_GET()

def main():
    os.chdir(DIST_DIR)
    
    with socketserver.TCPServer(("", PORT), VersionAwareHTTPRequestHandler) as httpd:
        print(f"🚀 Version server running at http://localhost:{PORT}")
        print(f"📖 Latest version: http://localhost:{PORT}/")
        
        # List available versions
        versions = []
        for item in DIST_DIR.iterdir():
            if item.is_dir() and item.name.startswith('v'):
                versions.append(item.name[1:])  # Remove 'v' prefix
        
        if versions:
            print(f"📋 Available versions: {', '.join(versions)}")
            for version in versions:
                print(f"📖 Version {version}: http://localhost:{PORT}/v{version}/")
        
        print("\nPress Ctrl+C to stop the server")
        httpd.serve_forever()

if __name__ == "__main__":
    main() 