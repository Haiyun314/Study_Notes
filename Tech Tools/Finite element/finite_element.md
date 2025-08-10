## Questions: 
- In fluid dynamics, directly computing gradients can be impractical. Therefore, we use the concept of weak derivatives, which allows us to reformulate problems into a weak form. This approach not only makes the computations more feasible but also provides a framework to rigorously discuss the existence and uniqueness of solutions to the problem.

## Overall Explanation of the Finite Element Method

The **Finite Element Method (FEM)** is a numerical technique used to approximate solutions of **partial differential equations (PDEs)**. It is particularly effective for complex geometries and boundary conditions.

## Core Idea

Suppose we are solving a PDE of the form:

$$
\mathcal{L}u(x) = f(x), \quad x \in \Omega,
$$

with appropriate boundary conditions on \( \partial\Omega \).

FEM approximates the unknown function \( u(x) \) by a linear combination of known basis functions:

$$
u(x) \approx \tilde{u}(x) = \sum_{i=1}^N c_i \phi_i(x),
$$

where:

- \( \phi_i(x) \) are the **basis functions** (e.g., piecewise linear),
- \( c_i \) are the **unknown coefficients**,
- \( N \) is the number of basis functions.

## Procedure
1. **Discretize the domain** \( \Omega \) into small elements (e.g., line segments in 1D, triangles in 2D).
2. **Choose basis functions** \( \phi_i(x) \) with local support on each element.
3. **Formulate the weak form** of the PDE. For example, turn:

   $$
   \mathcal{L}u = f
   $$

   into:

   $$
   a(u, v) = l(v) \quad \text{for all test functions } v,
   $$

   where \( a(\cdot, \cdot) \) is a bilinear form and \( l(\cdot) \) is a linear functional.

4. **Plug the approximation** \( u(x) \approx \sum c_i \phi_i(x) \) into the weak form.

5. **Test** the equation against each \( \phi_j(x) \), yielding:

   $$
   \sum_{i=1}^N a(\phi_i, \phi_j) c_i = l(\phi_j), \quad j = 1, \dots, N.
   $$

6. **Solve the linear system**:

   $$
   A\mathbf{c} = \mathbf{b},
   $$

   where:

   - \( A_{ij} = a(\phi_i, \phi_j) \) is the **stiffness matrix**,
   - \( b_j = l(\phi_j) \) is the **load vector**,
   - \( \mathbf{c} = [c_1, \dots, c_N]^\top \) are the unknown coefficients.

## Summary

- FEM transforms a continuous PDE problem into a discrete system of equations.
- The choice of elements and basis functions controls the accuracy.
- Once solved, the approximate solution can be evaluated or visualized over the domain.

