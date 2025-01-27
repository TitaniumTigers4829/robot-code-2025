# Formulas

This document contains information and sources for the formulas used in the project.

## 1. Module Chassis Acceleration

x = x position
y = y position
v_x = linear velocity x component
v_y = linear velocity y component
a_x = linear acceleration x component
a_y = linear acceleration y component
t = timestep duration
θ = drivetrain heading
ω = drivetrain angular velocity
α = drivetrain angular acceleration
m = drivetrain mass
J = drivetrain moment of inertia
τ = wheel's torque on the drivetrain
r = 2D distance vector from robot center of mass to wheel
F = 2D wheel force vector

xₖ₊₁ = xₖ + v_xₖt + 1/2a_xₖt²
v_xₖ₊₁ = v_xₖ + a_xₖt

yₖ₊₁ = yₖ + v_yₖt + 1/2a_yₖt²
v_yₖ₊₁ = v_yₖ + a_yₖt

θₖ₊₁ = θₖ + ωₖt
ωₖ₊₁ = ωₖ + αₖt

a_xₖ = (ΣF_xₖ)/m
a_yₖ = (ΣF_yₖ)/m
αₖ = (Στₖ)/J
τₖ = r x F

Inputs are tuple of F (2D vector).
