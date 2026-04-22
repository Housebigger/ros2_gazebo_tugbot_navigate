# Security Policy

## Reporting a Vulnerability

Please do not open a public issue for suspected security-sensitive problems.

Instead:

1. Prepare a minimal description of the issue.
2. Include affected files, launch paths, and reproduction conditions.
3. Include whether the issue affects:
   - local development only
   - simulation assets only
   - ROS 2 node behavior
   - dependency or packaging behavior
4. Share the report privately with the project maintainer.

## What to Include

A useful report should contain:

- summary of the vulnerability
- environment details
- exact reproduction steps
- expected behavior
- actual behavior
- impact assessment
- any temporary mitigation

## Scope Notes

This repository is primarily a robotics simulation and engineering workspace. Many issues may be safety, reproducibility, or integrity problems rather than remote-exploitation vulnerabilities.

Please still report any issue that could cause:

- unsafe robot behavior in simulation or downstream deployment
- integrity loss of packaged model/world assets
- accidental exposure of credentials or secrets
- unsafe dependency or launch-time execution behavior

## Disclosure Process

After receiving a report, maintainers will aim to:

- acknowledge receipt
- reproduce the issue
- determine severity and scope
- prepare a fix or mitigation
- coordinate any public disclosure after a fix is ready

## Supported Versions

Security support is best-effort for the current active workspace state documented in `README.md`.
