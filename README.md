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
conda create -p ./.venv python=3.12
conda activate ./.venv
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

## How to detect a safe pull over spot

First we detect a white line aside of the vehicle.
If the line is not detected we assume that there is no emergency lane -> NOT SAFE
If the line is dashed we assume there is a nearby exit -> NOT SAFE

Now that we have a solid white line we ensure there is enough space for the vehicle
to pull over (the emergency lane is wide enough).
We project multiple points on the other side of the white line and then we only
consider points which distance from the line is about the width of the vehicle (a bit more).

Now we consider a ground plane and then measure the height of each considered point from
this plane.
If there are multiple points which distance from the plane is relevantly high, we
then assume that there is some obstacle in the emergency lane -> NOT SAFE.



