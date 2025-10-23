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
- for the entire space needed for the maneuver:

  - the emergency lane must be clear of obstacles (vehicles or static obstacles)
  - there should be no entries or entry or exits from the highway

During the whole maneuver and after the vehicle has stopped the driver is required to press a button in order to go back to manual driving.
The reason is that we want the driver to explicitly express the will of taking control of the vehicle (it may not be safe to assume that the driver has regained consciousness by just detecting some movements on the driving wheel or on the pedals).

## How to detect a safe pull over spot

We basically need to achieve two goals and combine their results to decide if a spot is safe for pulling over.
- Detect the emergency lane and possible highway entries or entry or exits
- Detect if the emergency lane is free of obstacles

Initially we thought about exploiting the guardrail in order to detect the road boundary.
In the end we decided not do use that since guardrails are more likely to be missing with respect to the emergency lane marking.

We explored two different approaches using different sensors which are described below.

In order to simplify the examples and the implentation of this project we decided to assume
a right-hand drive context.

### Camera and radar approach

We exploit a forward looking camera in order to detect a continous (no entry or exit) lane marking (the emergency lane).

We exploit a long range radar pointing forward with a slight tilt on the right in order to detect any obstacles in the emergency lane.

1. The camera need to have a free line of sight on the emergency lane line for X meters ahead of the vehicle (X meters is the minimum distance for the vehicle to be able to stop safely and gently)
2. If the line is not continous throughout all the X meters we assume there is a nearby entry or exit -> NOT SAFE
3. The radar projects multiple points on the ground ahead for X meters (and slightly on the right) of the vehicle.
4. We only consider points which distance from the lane marking is about the width of the vehicle (a bit more). This allows us to allow slimmer vehicles to pull over even if the emergency lane il smaller in width.
5. We measure the height of each considered point from the ground (approximating it with a plane).
6. If there are multiple points which distance from the plane is relevantly high, we then assume that there is some obstacle in the emergency lane -> NOT SAFE.

#### Pros
- Cheaper sensors

#### Cons

- Harder to detect a safe pull over spot with high confidence (may result in missing some safe pull over spots)
- Harder to detect a safe pull over spot in turning roads (it can still be achieved but it requires the vehicle to slow down to a lower speed when searching for the spot)

### Semantic lidar approach

1. The sensor need to have a free line of sight on the emergency lane line for X meters ahead of the vehicle (X meters is the minimum distance for the vehicle to be able to stop safely and gently)
2. If the line is not continous throughout all the X meters we assume there is a nearby entry or exit -> NOT SAFE
3. Now that we have a solid white line we ensure there is enough space for the vehicle to pull over (the emergency lane is wide enough).
4. We consider all the sensed points on the other side of the white line and then we only
consider points which distance from the line (the sensor can give us 3D coorinates of the line) is about the width of the vehicle (a bit more).
5. We measure the height of each considered point from the ground (approximating it with a plane).
6. If there are multiple points which distance from the plane is relevantly high, we then assume that there is some obstacle in the emergency lane -> NOT SAFE.

#### Pros

- Higher capability of detecting a safe pull over spot with confidence
- More resilient on turning roads, even at higher speeds

#### Cons

- Extremely expensive sensor

We finally decided to apply the first approach in order to make our system easier to deploy on cheaper vehicles.

## How we achieve a flexible, smooth and robust pullover

There are three main parameters which then allow to compute most of the thresholds for the algorihm:

- minimum between each of the sensors maximum functional range
- maximum deceleration that the vehicle should reach while pulling over
- minimum pull over speed

Knowing the sensors maximum range and the maximum deceleration we can compute the maximum speed the vehicle can keep while searching for a safe pull over spot.
So the vehicle will not start pulling over if the speed is higher than the computed value.

While the vehicle is searching for a pull over spot it will check if there are any obstacles or entries/exits in the emergency lane and if so compute their distance.
Knowing the current speed and the distance from the next obstacle or entry/exit it is possible to compute what is the deceleration needed to stop before that.

If the computed deceleration is higher than what allowed (by the other primary parameter) then the vehicle will not consider this pull over spot as suitable.
Otherwise the vehicle will start pulling over while also decelerating as computed.

A minimum pull over speed must be maintained by the vehicle for the whole maneuver until it reaches the emergency lane and position itself correctly inside it.

The steering is computed by using Motor Schemas which is a well known control technique in robotics.
It provides flexibility and robustness as the steering is adjusted as a continuous function.

In this way it is possbile to achieve a pull over that:
- is gentle (maximum deceleration)
- is flexible as the space needed to stop depends on the vehicle speed
- robust with respect to how much roads can differ from one another

