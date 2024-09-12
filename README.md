# MIA-team-1-task-12.1
## Kinematic Model for Differential Drive Robot

**Equations:**
* Linear velocity: `v = (r1 + r2) / 2 * ω`
* Angular velocity: `ω = (r2 - r1) / L`

**Variables:**
* `v`: Linear velocity of the robot
* `ω`: Angular velocity of the robot
* `r1`: Angular velocity of the left wheel
* `r2`: Angular velocity of the right wheel
* `L`: Wheelbase (distance between the centers of the wheels)

## Kinematic Model for Mecanum Wheel Robot

**Equations:**
* Linear velocity: `vx = (r1 + r2 + r3 + r4) / 4 * ω1`
* Linear velocity: `vy = (r1 - r2 + r3 - r4) / 4 * ω2`
* Angular velocity: `ω = (r1 - r2 - r3 + r4) / 4 * L`

**Variables:**
* `vx`: Linear velocity in the x-direction
* `vy`: Linear velocity in the y-direction
* `ω`: Angular velocity of the robot
* `r1`, `r2`, `r3`, `r4`: Angular velocities of the four wheels
* `L`: Wheelbase (distance between the centers of the wheels)

$$
v = \frac{r_1 + r_2}{2} \omega
$$

$$
\omega = \frac{r_2 - r_1}{L}
$$

$$
v_x = \frac{r_1 + r_2 + r_3 + r_4}{4} \omega_1
$$

$$
v_y = \frac{r_1 - r_2 + r_3 - r_4}{4} \omega_2
$$

$$
\omega = \frac{r_1 - r_2 - r_3 + r_4}{4} L
$$
