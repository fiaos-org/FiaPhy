---
title: 'FiaPhy: Differential Temporal Derivative Soft-Sensing for Solar Radiation'
tags:
  - C++
  - Arduino
  - embedded systems
  - solar radiation
  - soft-sensing
  - IoT
  - environmental monitoring
authors:
  - name: Neksha V. DeSilva
    orcid: 0009-0000-1434-4777
    affiliation: 1
    corresponding: true
affiliations:
 - name: FiaOS.org
   index: 1
date: 25 November 2025
bibliography: paper.bib
---

# Summary

**FiaPhy** is a C++ library designed for embedded systems (Arduino, ESP32, Raspberry Pi) that reconstructs Global Horizontal Irradiance (GHI) and convective heat flux using standard environmental sensors. 

Current environmental monitoring relies on static state variables—temperature, pressure, and humidity—while remaining blind to the dynamic energy exchanges that drive them. FiaPhy implements the Differential Temporal Derivative Soft-Sensing (DTDSS) framework, employing a differential topology with Inertial Noise Reduction (INR) to mathematically reconstruct solar radiation without the cost or fragility of thermopile pyranometers.The library transforms commodity hardware (specifically dual BME280 configurations [@Bosch:2016]) into capability-dense radiometers suitable for mass IoT deployment. While currently specialized for solar flux, the underlying DTDSS framework establishes a generalized methodology for deriving complex energy flux parameters from standard state variables, paving the way for a new class of virtual sensors on constrained embedded architectures.

# Statement of need

Solar radiation is a fundamental variable for agricultural evapotranspiration models (such as Penman-Monteith [@Allen:1998]), renewable energy forecasting, and climate monitoring. However, the hardware required to measure it—thermopile pyranometers—is prohibitively expensive ($500+), fragile, and power-hungry, making them unsuitable for dense IoT networks or low-cost agriculture solutions [@ISO9060].

Researchers and engineers currently face a gap: they must either rely on expensive, sparse reference stations or use machine learning models that often fail to generalize across different altitudes and climates due to their reliance on static training data [@Fortuna:2007].

FiaPhy addresses this need by providing a physics-based, hardware-agnostic computational library. Unlike "black box" ML approaches, FiaPhy derives air density and enthalpy dynamically from local pressure and humidity using the Magnus-Tetens formula [@Alduchov:1996] for vapor pressure calculations, making the system altitude-invariant—deployable from sea level to mountaintops without reconfiguration. The convective heat transfer models are based on established engineering principles [@Incropera:2007], with empirical cloud-proxy relationships adapted from temperature-based radiation estimation methods [@Samani:2000]. It allows researchers in hydrology, agronomy, and embedded engineering to access radiative flux data using standard Class-0 IoT devices (8-bit microcontrollers) with minimal memory footprint (<60 bytes RAM).

# Implementation

The software implements a dual-pipeline architecture:
1.  **Reference Path:** Uses the Kasten-Czeplak cloud model [@Kasten:1976] to establish a baseline radiation probability based on humidity and pressure.
2.  **Reactive Path:** Uses the inverted Newton's Law of Cooling combined with a custom Inertial Noise Reduction (INR) filter [@Savitzky:1964] to detect instantaneous heat flux events.

The library is designed for portability, using platform-agnostic C++11, allowing integration into existing Arduino, PlatformIO, or Linux-based environmental monitoring stacks.

# Acknowledgements

We acknowledge the support of FiaOS.org in providing the reference architecture for validation.

# References