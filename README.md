# Constellation Asteroid Measurement Project (CAMP)

This repository contains all code, documentation, and resources for the Constellation Asteroid Measurement Capstone Project.

## Overview
The Constellation Asteroid Measurement Project (CAMP) is a Capstone mission to design, simulate, and demonstrate a CubeSat-based probe system for near‐Earth asteroid exploration, sensor data acquisition, and sample return. The mission targets two asteroids—436724 2011 UW158 and 153958 2002 AM31—selected for their similar orbital parameters and scientific value, including potential resources and insights into binary asteroid dynamics.

Key objectives:

- **Design and integrate** three probe types (Visual, Non‐Visual, Lander) in a 6U CubeSat form factor.
- **Develop a robust attitude determination and control system (ADCS)** using an inverse‐tetrahedral reaction wheel configuration and Super‐Twisting Sliding Mode Control.
- **Demonstrate** system performance on a 20 %‐scale prototype with CNC‐machined reaction wheels, Arduino‐based control, and a 3-DOF test stand.

## Project Timeline

| Milestone                   | Date        |
| --------------------------- | ----------- |
| Mission Launch              | July 2034   |
| UW158 Rendezvous            | Mar 2040    |
| UW158 Depart                | Sep 2040    |
| Earth Fly‐by                | Aug 2042    |
| AM31 Rendezvous             | Oct 2045    |

## Structure
- `src/` - Source code (MATLAB, Python, Arduino)
- `simulink/` - Simulink models
- `docs/` - Documentation, reports, and budget
- `tests/` - Test scripts

## Results & Conclusions

- **Design Corrections**: Removed redundant sensors on Lander Probes, expanded sample‐collector volume from 1U to 2U.  
- **ADCS Optimization**: Replaced integrated ADCS unit with discrete ST‐200 star tracker and four RW‐400 wheels; implemented Super‐Twisting SMC for precise, low‐jitter control.  
- **Prototype Validation**: Demonstrated control law implementation on a scale model; identified motor–controller compatibility improvements (integral Hall‐sensor motors).  
- **Engineering Standards**: Conformed to NASA Systems Engineering Handbook (SP-2016-6105), IEEE 15288, ISO/IEEE 29148, and NASA-STD-7009 models and simulations standards.
  
## License
This project is licensed under the [MIT License](LICENSE).
