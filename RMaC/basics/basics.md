### Typical robot joints:
revolute, prismatic, helical, cylindrical, universal, spherical

Dof: It’s not just about the number of joints, but about how many independent movements the end-effector can make, given joint constraints and link geometry. (x, y, z, pitch, roll, yaw)

### Concepts of rigid dynamics system:

Moment of inertia: similar terminology of mass, but different descriptive object, it describes the resistance to move around the axis

For a rigid body rotating about an axis with angular velocity $\omega$:

$$
v = r \omega \quad \text{(linear speed of a particle at distance $r$)}
$$

Kinetic energy of a particle:

$$
dT = \tfrac{1}{2} dm \, v^2 = \tfrac{1}{2} dm \, (r \omega)^2 = \tfrac{1}{2} (r^2 dm) \, \omega^2
$$

That’s why the **moment of inertia** is defined as:

$$
I = \int r^2 \, dm
$$
