---
icon: lucide/home
---

# Welcome to the Nebula documentation

Welcome to the Nebula documentation. Here you will find information about the background of the project, how to install and use with ROS 2, and also how to add new sensors to the Nebula driver.

## About Nebula

Nebula is a sensor driver platform that is designed to provide a unified framework for as wide a variety of devices as possible.
While it primarily targets Ethernet-based LiDAR sensors, it aims to be easily extendable to support new sensors and interfaces.
Nebula works with ROS 2 and is the recommended sensor driver for the [Autoware](https://autoware.org/) project. The project aims to provide:

- A universal sensor driver
  - Supporting an increasing number of LiDAR, radar and camera sensors
- 100% open source, with a no-binaries policy
- ROS 2 wrappers for easy inclusion in robotics and self-driving vehicle projects
- A driver solution to suit current Autoware requirements
  - Interfaces and pointcloud type updates made in unison with Autoware developments

## Getting started

- [Installation](../getting_started/installation.md)
- [Launching with ROS 2](../usage/index.md)

## Nebula architecture

- [Design](design/index.md)
- [Parameters](../parameters/index.md)
- [Point cloud types](../usage/point_types.md)
- [Point filters](../usage/filters.md)

## Supported sensors

- [Supported sensors](../getting_started/supported_sensors.md)

## Development

- [Tutorials](../contributing/tutorials.md)
- [Contributing](../contributing/contributing.md)
