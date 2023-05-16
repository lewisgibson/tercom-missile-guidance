# Security Policy

## Scope

This is a research and simulation library for terrain-referenced navigation.
It performs offline numerical computation and, in its optional hardware
backends, reads from local serial devices. It has no network listeners and
handles no credentials.

## Reporting a vulnerability

If you find a memory-safety issue, a crash on malformed input (for example a
malformed DEM or NMEA stream), or any other defect with security relevance,
please open a private security advisory through GitHub's "Report a
vulnerability" workflow on the repository's Security tab rather than a public
issue.

Please include a minimal reproducer and the affected commit. A response can be
expected within a reasonable time for a personal project.

## Supported versions

The `main` branch is the only supported version.
