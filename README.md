# Sleep detection and safety pull over

## Developers

- Michele Ravaioli
- Alessandro Venturini

## Abstract

We propose a vehicle equipped with an adaptive cruise control system that includes highway lane-keeping functionality (which, for safety reasons, requires the driver to remain attentive at all times).

The system we aim to develop will complement the aforementioned functionality by detecting if the driver falls asleep and, in such cases, safely pulling the vehicle over to a stop.

Since the described behavior would require the vehicle to safely change lanes to reach the emergency lane—a task that is inherently complex—we have chosen to adopt the following simplifying assumption:

The vehicle’s cruise control system is designed to operate only in the slowest lane (already adjacent to the emergency lane).

## Setup

### Virtual environment

```sh
conda create -p .venv python=3.7
conda activate .venv
```

### Requirements

```sh
pip install -r requirements.txt
```

### How to update typings

[carla-python-stubs](https://github.com/aasewold/carla-python-stubs/releases/tag/0.9.15)

## Solution

We detect driver drowsiness or more generally inattention by leveraging an artificial vision algorithm based on face detection and analisys.

During pull over preparation we check the following conditions in order to compute if the vehicle can start pulling over:

- there must be an emergency lane
- there must be no vehicle coming from the emergency lane (behind the ego vehicle)
- for the entire space needed for the maneuver:

  - the emergency lane must be clear of obstacles (vehicles or static obstacles)
  - there should be no exits from the highway

During the whole maneuver and after the vehicle has stopped the driver is required to press a button in order to go back to manual driving.
The reason is that we want the driver to explicitly express the will of taking control of the vehicle (it may not be safe to assume that the driver has regained consciousness by just detecting some movements on the driving wheel or on the pedals).
