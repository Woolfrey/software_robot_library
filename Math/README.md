# :heavy_plus_sign: Math
This section of the library contains useful mathematical functions.

## Contents
- [Simple QP Solver](#simple-qp-solver)
- [Skew Symmetric Class](#skew-symmetric-class)

## Simple QP Solver
Many control problems in robotics can be solved using quadratic programming (QP), or convex optimisation of the form:
```math
\begin{align}
	\min_{\mathbf{x}} ~ \frac{1}{2}\mathbf{x^\mathrm{T}Hx + x^\mathrm{T}f} \\
	\text{subject to: } \mathbf{Bx \le z}
\end{align}
```
where:
- $\mathbf{x}\in\mathbb{R}^\mathrm{n}$ is the decision variable,
- $\mathbf{H = H^\mathrm{T}}\in\mathbb{R}^\mathrm{n\times n}$ is a weighting matrix,
- $\mathbf{f}\in\mathbb{R}^\mathrm{n}$ is the linear component of the quadratic equation,
- $\mathbf{B}\in\mathbb{R}^\mathrm{c\times n}$ is a constraint matrix, and
- $\mathbf{z}\in\mathbb{R}^\mathrm{c}$ is a constraint vector.

[SimpleQPSolver](https://github.com/Woolfrey/software_simple_qp) is a single header file that is automatically downloaded in to RobotLibrary. It contains useful functions for solving QP problems, both constrained and uncontrained.

[:arrow_backward: Go Back.](#contents)

## Skew Symmetric Class

[:arrow_backward: Go Back.](#contents)
