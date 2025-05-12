[Convergence Analysis of the Proportional Control Term](#convergence-analysis-of-the-proportional-control-term)


# Classical Control Approaches: PID Control

The classical PID (proportional-integral-derivative) controller is generally defined as:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

where:

- $e(t) = S(t) - S_{\text{target}}(t)$ is the error (reference minus output),
- $u(t)$ is the control input at time $t$,
- $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains, which must be carefully tuned during operation.

---

## Convergence Analysis of the Proportional Control Term

We consider a discrete-time linear system controlled by a proportional term:

$$
S_{k+1} = A S_k + B U_k
$$

$$
U_k = K_p (S_k - S_{\text{target}})
$$

Rewriting the equations:

$$
S_{k+1} = (A - B K_p) S_k + B K_p S_{\text{target}}
$$

### Define the Error

Let the tracking error be:

$$
e_k := S_k - S_{\text{target}} \quad \Rightarrow \quad S_k = e_k + S_{\text{target}}
$$

Substitute into the update equation:

$$
\begin{aligned}
S_{k+1} &= (A - B K_p)(e_k + S_{\text{target}}) + B K_p S_{\text{target}} \\
       &= (A - B K_p)e_k + (A - B K_p)S_{\text{target}} + B K_p S_{\text{target}} \\
       &= (A - B K_p)e_k + A S_{\text{target}}
\end{aligned}
$$


Now compute the next error term:

$$
e_{k+1} = S_{k+1} - S_{\text{target}} = (A - B K_p)e_k + (A - I) S_{\text{target}}
$$

---

### Linear Recurrence

This gives the inhomogeneous recurrence:

$$
e_{k+1} = M e_k + c
$$

where:

$$
M = A - B K_p, \quad c = (A - I) S_{\text{target}}
$$

---

### General Solution

The general solution of the recurrence is:

$$
e_k = M^k e_0 + \sum_{i=0}^{k-1} M^i c
$$

We define the spectral radius:

$$
\rho(A) = \max \left\{ |\lambda| : \lambda \text{ is an eigenvalue of } A \right\}
$$

If $\rho(M) < 1$, then:

$$
\lim_{k \to \infty} e_k = (I - M)^{-1} c
$$

Substitute $M$ and $c$:

$$
\lim_{k \to \infty} e_k = (I - A + B K_p)^{-1}(A - I) S_{\text{target}}
$$

Therefore, the steady-state value of $S_k$ is:

$$
\lim_{k \to \infty} S_k = \lim_{k \to \infty} (e_k + S_{\text{target}}) = \left[ (I - A + B K_p)^{-1}(A - I) + I \right] S_{\text{target}}
$$

---

### Conclusion

- The system converges if $\rho(A - B K_p) < 1$.
- The limit of $S_k$ depends on system dynamics and proportional gain.
- Proper tuning of $K_p$ is necessary to ensure convergence to $S_{\text{target}}$.
