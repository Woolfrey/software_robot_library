# Math
This section of the library contains useful mathematical functions.

[:rewind: Back to the foyer.](../README.md)

### Contents:
- [Backward Substitution](#backward-substitution)
- [Forward Substitution](#forward-substitution)
- [QR Decomposition](#qr-decomposition)
- [Simple QP Solver](#simple-qp-solver)
- [Skew Symmetric Class](#skew-symmetric-class)

## Backward Substitution
For a system of equations:
```math
\underbrace{
\begin{bmatrix}
y_1 \\ \vdots \\ y_m
\end{bmatrix}}_{\mathbf{y}} =
\underbrace{
\begin{bmatrix}
u_{11} & \cdots & u_{1m} \\
       & \ddots & \vdots \\
       &        & u_{mm}
\end{bmatrix}
}_{\mathbf{U}}
\begin{bmatrix}
x_1 \\ \vdots \\ x_m
\end{bmatrix}
```
instead of directly inverting $\mathbf{x} = \mathbf{U}^{-1}\mathbf{y}$ we can instead call:
```
Eigen::VectorXd x = backward_subsitution(y,U, tolerance);
```
which is faster and more numerically stable. The `tolerance` argument is the level of precision for handling singularities.

Eigen can [also solve triangular systems](http://eigen.tuxfamily.org/dox/group__QuickRefPage.html#title14), but this function was written for more precise control over singularities.

[:arrow_backward: Go back.](#math)

## Forward Substitution

For a system of equations:
```math
\underbrace{
\begin{bmatrix}
y_1 \\ \vdots \\ y_m
\end{bmatrix}}_{\mathbf{y}} =
\underbrace{
\begin{bmatrix}
l_{11} &        &        \\
\vdots & \ddots &        \\
l_{m1} & \cdots & l_{mm}
\end{bmatrix}
}_{\mathbf{L}}
\begin{bmatrix}
x_1 \\ \vdots \\ x_m
\end{bmatrix}
```
instead of directly inverting $\mathbf{x} = \mathbf{L}^{-1}\mathbf{y}$ we can instead call:
```
Eigen::VectorXf x = forward_subsitution(y,L, tolerance);
```
which is faster and more numerically stable. The `tolerance` argument is the level of precision for handling singularities.

Eigen can [also solve triangular systems](http://eigen.tuxfamily.org/dox/group__QuickRefPage.html#title14), but this function was written for more precise control over singularities.

[:arrow_backward: Go back.](#math)


## QR Decomposition
A matrix $\mathbf{A}\in\mathbb{R}^{m\times n}$ for $m >= n$ can be decomposed in to:
```math
\mathbf{A} = \underbrace{\begin{bmatrix}\mathbf{Q}_r & \mathbf{Q}_n \end{bmatrix}}_{\mathbf{Q}} \begin{bmatrix} \mathbf{R} \\ \mathbf{0} \end{bmatrix}
```
where:
 - $\mathbf{Q}\in\mathbb{O}(m)$ is an orthogonal matrix: $\mathbf{Q^\mathrm{T}Q = I}$, and
 - $\mathbf{R}\in\mathbb{R}^{n\times n}$ is an upper-triangular matrix:
```math
\mathbf{R} =
\begin{bmatrix}
r_{11} & \cdots & r_{1n} \\
       & \ddots & \vdots \\
       &        & r_{nn}
\end{bmatrix}
```
Eigen already has [several QR decomposition aglorithms available](https://eigen.tuxfamily.org/dox/group__TopicLinearAlgebraDecompositions.html). A function utilizing the [Schwarz-Rutishauser](https://towardsdatascience.com/can-qr-decomposition-be-actually-faster-schwarz-rutishauser-algorithm-a32c0cde8b9b) algorithm is provided in this library. It returns only $\mathbf{Q}_r\in\mathbf{R}^{m\times n}$ and $\mathbf{R}$ as a data structure:
```
QRdecomposition decomp<float> = schwarz_rutishauser(A);
Eigen::MatrixXf Q = decomp.Q;
Eigen::MatrixXf R = decomp.R;
```
or alternatively:
```
const auto &[Q, R] = schwarz_rutishauser(A);
```
This was done deliberately for more precise handling of singularities in $\mathbf{R}$ by using [backward substitution](#backward-substitution) and [forward substitution](#forward-substitution) also provided by `Math.h`.

[:arrow_backward: Go back.](#math)

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

[:arrow_backward: Go back.](#math)

## Skew Symmetric Class
The cross-product of two vectors $\mathbf{a,~b}\in\mathbb{R}^3$ can be written as the product of a skew-symmetric matrix and a vector:
```math
\mathbf{a}\times\mathbf{b} =
\underbrace{
	\begin{bmatrix}
		0 & -a_3 & a_2 \\
                a_3 & 0 & -a_1 \\
               -a_2 & a_1 & 0
	\end{bmatrix}
}_{S(\mathbf{a})}
\underbrace{
	\begin{bmatrix}
		b_1 \\
		b_2 \\
		b_3
	\end{bmatrix}
}_{\mathbf{b}}
```
where $S(\mathbf{a}) : \mathbb{R}^3\mapsto\mathbb{R}^{3\times3}$ is the skew-symmetric matrix operator.

`Eigen` has an in-built function for the cross-product of 3D vectors: `a.cross(b)`, but does not have a skew-symmetric matrix representation. A skew-symmetric class is available in this library for convenient multiplication of a skew-symmetric matrix and another matrix $S(\mathbf{a})\mathbf{B}$ where $\mathbf{B}\in\mathbb{R}^{3\times n}$.

For example, if we have a 3D vector we convert it to a skew-symmetric matrix object for use:
```
Eigen::Vector3d a = ...;     // Declaration of a 3d vector.
SkewSymmetric<double> S(a);  // Create skew-symmetric matrix object.
...
Eigen::Matrix<double,3,Eigen::Dynamic> product = S * someOtherMatrix; // Matrix multiplication of skew-symmetric matrix with another.
```
[:arrow_backward: Go back.](#math)
