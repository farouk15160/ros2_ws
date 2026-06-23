# UNIVERSAL ROBOTICS STUDIO

# UI/UX DESIGN SPECIFICATION

---

# Vision

Design a **next-generation robotics control interface** that feels futuristic, premium, and highly intuitive.

The interface should combine elements inspired by:

- Apple Vision Pro
- Tesla UI
- Rivian
- Unreal Engine
- Iron Man Jarvis
- NVIDIA Omniverse
- Isaac Sim
- CAD software
- Modern SCADA systems
- Professional industrial HMIs

The goal is to create:

> The Visual Studio Code + Unreal Engine of Robotics.

---

# Design Language

Style:

- Minimal
- Dark mode first
- Glassmorphism
- Frosted translucent panels
- Rounded corners
- Smooth animations
- Premium feel
- Modern typography
- Subtle gradients
- Neon highlights

Avoid:

- Old industrial interfaces
- Windows 95 style
- Clutter
- Too many colors
- Tiny buttons

Everything should feel:

- Elegant
- Fast
- Responsive
- Professional

---

# Theme

Main Colors:

```yaml
background: "#0B0D10"
panel: "#111418"
card: "#1A1F26"

accent_blue: "#00C2FF"
accent_green: "#00FFAE"
accent_red: "#FF4E63"
accent_orange: "#FFA63E"
accent_purple: "#7A5FFF"

text_primary: "#FFFFFF"
text_secondary: "#8A96A3"
```

---

# Layout

```text

┌─────────────────────────────────────────────┐
│ Top Bar                                     │
├──── Sidebar ────┬───────────────────────────┤
│                 │                           │
│                 │        3D Viewport        │
│                 │                           │
│                 │                           │
│                 │                           │
│                 ├──────── Bottom Panels ────┤
│                 │ Console | IO | Diagnostics│
│                 │                           │
└─────────────────┴───────────────────────────┘

```

---

# Main Sections

## Dashboard

Overview page.

Contains:

### Robot Status Card

Shows:

- Connected
- Emergency stop
- Temperature
- Voltage
- CPU load
- Joint states
- Active task

Animated indicators.

---

### Camera Preview

Live RGB stream.

AI detections overlay.

Depth map support.

---

### Robot Health

Live charts:

- Motor temperatures
- Current draw
- Torque
- CPU usage
- Network latency

---

### Notifications

Modern notification center.

Priority levels:

- Info
- Warning
- Critical

---

# 3D Robot Workspace

The center of the application.

Inspired by:

- Isaac Sim
- Unreal Engine
- Fusion360

Contains:

## Interactive Robot

User can:

- Rotate view
- Pan
- Zoom
- Select links
- Select joints

---

## Collision Objects

Display:

- Tables
- Walls
- Obstacles
- Point clouds

---

## Camera Frustums

Visualize:

- RGB cameras
- Depth cameras
- Lidar

---

## TF Frames

Show coordinate systems.

---

## Tool Paths

Visualize trajectories.

---

## Reachability Maps

Heatmap visualization.

---

# Robot Control Page

Contains:

### Cartesian Control

XYZ movement.

Buttons:

+X
-X

+Y
-Y

+Z
-Z

Rotation:

RX
RY
RZ

Continuous jogging.

---

### Joint Control

Dynamic number of joints.

Supports:

3 DOF
4 DOF
6 DOF
7 DOF
12 DOF

Automatically generated sliders.

Display:

- Position
- Velocity
- Current
- Temperature

---

### Tool Control

Depending on end-effector:

Gripper:

Open
Close

Vacuum:

On
Off

Welder:

Enable
Disable

Screwdriver:

RPM

---

# Teach Pendant Mode

Full-screen interface.

Large buttons.

Optimized for touchscreen.

Contains:

- Jog controls
- Speed override
- Coordinate systems
- Emergency stop

Tablet friendly.

---

# IO Panel

Digital Inputs

Digital Outputs

Analog Inputs

Analog Outputs

CAN messages

EtherCAT devices

GPIO

Relays

PWM

Live updates.

---

# Camera Panel

Supports:

RGB

Depth

Stereo

Thermal

Lidar

Multiple cameras simultaneously.

Features:

Snapshot

Recording

Calibration

FPS

Latency

Exposure

AI overlay

---

# Point Cloud Viewer

Supports:

Colored cloud

Voxel maps

Octomap

TSDF

Dynamic obstacles

Selectable objects

Transparency controls

---

# Task Manager

Inspired by VSCode.

Tree structure:

```text
Mission
 ├── Pick Object
 ├── Move To Position
 ├── Open Gripper
 ├── Wait
 └── Return Home
```

Drag and drop editing.

---

# AI Assistant Panel

ChatGPT-style interface.

Can:

Generate motions

Explain errors

Modify configurations

Create tasks

Debug hardware

Generate MoveIt configs

Generate URDF

Calibrate cameras

Repair TF issues

Suggest trajectories

Voice commands optional.

---

# Diagnostics

Charts:

CPU

RAM

CAN traffic

Network

Packet loss

Temperature

Joint currents

Motor load

Update rates

---

# Terminal Panel

Like VSCode.

Tabs:

ROS2

System

Docker

CAN

Logs

AI Agent

Color syntax.

Search support.

---

# Package Manager

Install:

Drivers

Planners

Cameras

Sensors

IK Solvers

Plugins

Themes

Updates

One-click installation.

---

# Motion Planning Page

MoveIt2 integration.

Visualize:

Goal pose

Current pose

Trajectory

Collision checking

Singularities

Manipulability

Execution time

Planner comparisons.

---

# Calibration Wizard

Step-by-step.

Robot calibration

Camera calibration

Tool calibration

TCP calibration

Hand-eye calibration

Joint offsets

Automatic detection.

---

# Digital Twin

Switch between:

Real robot

Simulation

Gazebo

Isaac Sim

MuJoCo

Twin synchronization.

---

# Multi-Robot View

Display:

Robot 1

Robot 2

Mobile robot

AGV

Shared workspace

Collision zones

---

# Scene Builder

Drag and drop:

Tables

Boxes

Walls

Fixtures

Tools

Conveyors

Humans

CAD models

---

# Appearance

Animations should be smooth.

120 FPS preferred.

Subtle glows.

Soft shadows.

Rounded cards.

Blur effects.

No hard edges.

No excessive clutter.

---

# Sidebar

Icons only.

Animated expansion.

Sections:

Dashboard

Workspace

Robot

Motion

IO

Vision

Tasks

Calibration

Diagnostics

Terminal

AI Assistant

Settings

Plugins

---

# Top Bar

Connection status

Battery

Network

CPU

Current mode

Emergency stop

User profile

Notifications

Clock

Search

---

# Settings

Theme

Language

Units

Plugins

ROS settings

MoveIt settings

Cameras

Controllers

Networking

Accounts

---

# Responsive Design

Must support:

Desktop

Touchscreen

Tablet

4K monitors

Ultra-wide monitors

Multi-monitor setups

---

# Technology Suggestions

Frontend:

Qt6 + QML

or

Tauri + React + Typescript

or

Electron + React

Rendering:

OpenGL

Vulkan

Three.js

RViz integration

Backend:

ROS2

FastAPI

Python

C++

WebSockets

Docker

---

# Goal

The final product should feel like:

> Unreal Engine + Tesla + Apple Vision Pro + NVIDIA Omniverse, but built specifically for robotics.

Users should immediately feel that this is a premium professional robotics operating environment rather than a traditional industrial application.
