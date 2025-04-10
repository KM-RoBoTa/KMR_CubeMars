\page how-to-use How to use
[TOC]

The library lives in the KMR::CBM namespace. 

# Concept of the library

## Threading
This library uses two threads.\n
The Listener thread monitors messages on the CAN bus and saves their information. The main thread can access this information when required. \n

From a user's perspective, every interaction with the library needs to be done through a MotorHandler instance, which is the highest-level class.

## MIT mode

The control equation used by the motors is:
\f[
T = Kp \cdot (p_{goal} - p_{curr}) + Kd \cdot (v_{goal} - v_{curr}) + Tff
\f]

with:
- \f$ T \f$ : input torque in Nm
- \f$ Kp \f$: proportional factor in Nm/rad
- \f$ Kd \f$: derivative factor in Nm/(rad/s)
- \f$ p_{goal} \f$ and \f$ p_{curr} \f$: goal and current positions respectively, in rad
- \f$ v_{goal} \f$ and \f$ v_{curr} \f$: goal and current speeds respectively, in rad/s
- \f$ Tff \f$ : feedforward torque in Nm

The user can control \f$ Kp \f$, \f$ Kd \f$, \f$ p_{goal} \f$, \f$ v_{goal} \f$ and \f$ Tff \f$, which are sent in a CAN packet to a wished motor. The motor's firmware then internally calculates the corresponding input torque \f$ T \f$.


When controlling all 5 parameters, the user controls the motor in impedance. By setting specific parameters to 0, this control can become pure position, speed or torque control. The different types of control are illustrated by the following table:


|                   | Kp | Kd | p_goal | v_goal | Tff |
|-------------------|----|----|--------|--------|-----|
| Impedance control | Kp | Kd | p_goal | v_goal | Tff |
| Position control  | Kp | Kd | p_goal | 0      | 0   |
| Speed control     | 0  | Kd | 0      | v_goal | 0   |
| Torque control    | 0  | 0  | 0      | 0      | Tff |

# Axis definition

The motors' axis is defined as follows:
![Motor axis definition](cubemars_motor.png)

The counterclockwise rotation is therefore defined as positive.

# MotorHandler

##  Zero setting

## ID order

## Function list?

# Examples 
