# Issues

## Known Issues

### Hardware Issues

#### Power Supply
- **Issue**: Intermittent power loss during operation
- **Solution**: Ensure stable 24VDC power supply with adequate current rating
- **Status**: Resolved in firmware version 2.1.0

#### Communication
- **Issue**: Ethernet connection drops occasionally
- **Solution**: Check network cable and router settings
- **Status**: Under investigation

### Software Issues

#### ROS 2 Compatibility
- **Issue**: Some ROS 2 packages may not work with OMX
- **Solution**: Use the provided Docker container with pre-configured environment
- **Status**: Resolved

#### Docker Container
- **Issue**: Container fails to start on some systems
- **Solution**: Update Docker to latest version and ensure sufficient disk space
- **Status**: Resolved

## Troubleshooting

### Common Problems

#### OMX Not Responding
1. Check power connection
2. Verify network connectivity
3. Restart the system
4. Contact support if issue persists

#### SSH Connection Failed
1. Ensure OMX is powered on
2. Check network connection
3. Verify hostname format: `omx-SNPR44B9999.local`
4. Try using IP address directly

#### Joint Movement Issues
1. Check for physical obstructions
2. Verify joint limits
3. Restart the system
4. Contact support if issue persists

## Reporting Issues

To report a new issue:

1. **Check existing issues**: Search the [GitHub issues page](https://github.com/ROBOTIS-GIT/open_manipulator/issues)
2. **Create new issue**: Use the issue template and provide detailed information
3. **Include logs**: Attach relevant log files and error messages
4. **Describe steps**: Provide clear steps to reproduce the issue

### Issue Template

When reporting an issue, please include:

- **Hardware Model**: OMX-3M, OMX-F3M, OMX-L100, or OMX-AI3M
- **Firmware Version**: Current firmware version
- **Software Version**: ROS 2 version and package versions
- **Operating System**: Host OS and container OS versions
- **Description**: Clear description of the issue
- **Steps to Reproduce**: Detailed steps to reproduce the issue
- **Expected Behavior**: What should happen
- **Actual Behavior**: What actually happens
- **Logs**: Relevant log files and error messages
- **Screenshots**: If applicable, include screenshots or videos

## Support

For urgent issues or questions not covered here, please contact:

- **Email**: support@robotis.com
- **Forum**: [https://forum.robotis.com](https://forum.robotis.com)
- **Discord**: [https://discord.gg/robotis](https://discord.gg/robotis) 