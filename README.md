# Double Inverted Pendulum System

## Description
The inverted pendulum system is a complex control system characterized by non-linearity, strong coupling, multivariable, and instability. Studying inverted pendulums has significant practical implications for the research of modern high-tech fields such as rocket flight control and robot control. Hence, controlling inverted pendulums has become an enduring research topic in control theory. In the past 25 years, many classic and modern control theories have been employed to control inverted pendulums, such as PID control, adaptive control, state feedback control, intelligent control, fuzzy control, and artificial neuronal control.

<div style="text-align:center;">
    <img src="inverted pendulum2.png" alt="Inverted Pendulum System" height="500">
    <figcaption>Second Order Inverted Pendulum System.</figcaption>
</div>
<br>

We establish a model of a double pendulum. As shown in the diagram above, the double pendulum system consists of the following parts: a smooth guide rail, a cart that can move back and forth on the rail, with mass of M, a lower pendulum rod 1 hinged to the cart, with mass of $m_1$​, and an upper pendulum rod 2, with mass of $m_2$​ connected to the lower pendulum rod 1 in the same way. Their hinging method determines their motion in the vertical plane.

For ease of analysis, we make the following assumptions: each pendulum rod is rigid, and all forms of friction (Coulomb, dynamic, etc.) are negligible for the modelling process.

We divide the double pendulum into 3 parts to better represent each parameter. 

**1. Car**  The force exerted by pendulum rod 1 on the cart is decomposed into a vertical component $F_{11}$; and a horizontal component $F_{12}$. F is the traction force of the cart, serving as an input. As shown in the diagram below:

<div style="text-align:center;">
    <img src="car.png" alt="Force diagram of the car" height="300">
    <figcaption>Force diagram of the car.</figcaption>
</div>
<br>


**2. Lower Pendulum Rod** The moment of inertia of pendulum rod 1 around its centre of rotation is $I_1$​, the distance from the centre of the lower pendulum to the axis of rotation is $l_1$​, and the distance between the axes of pendulum rods 1 and 2 is $L_1​$.

<div style="text-align:center;">
    <img src="bar1.png" alt="Force diagram of the first rod" height="300">
    <figcaption>Force diagram of the first rod.</figcaption>
</div>
<br>


**3. Upper Pendulum Rod** The moment of inertia of pendulum rod 2 around its centre of rotation is $I_2$, and the distance from the centre of pendulum rod 2 to the axis of rotation is $l_2$​.

<div style="text-align:center;">
    <img src="bar2.png" alt="Force diagram of the upper rod" height="300">
    <figcaption>Force diagram of the upper rod.</figcaption>
</div>
<br>


## Equations
Firstly, we will conduct a step-by-step kinematic analysis of the system

The relationship equation can be obtained by analyzing the centroid coordinates of the upper and lower pendulum rods:

$$ 
\begin{aligned}
	x_{1g}&=x+l_1\sin \theta _1\\
	y_{1g}&=l_1\cos \theta _1\\
	x_{2g}&=x+L\sin \theta _1+l_2\sin \theta _2\\
	y_{2g}&=L\cos \theta _1+l_2\cos \theta _2\\
\end{aligned}
$$


A comprehensive force analysis of the system can deduce the following relationship in the horizontal direction:

$$ F=M\ddot{x}+m_1\frac{d^2}{dt^2}(x_{1g})+m_2\frac{d^2}{dt^2}(x_{2g}) $$


The force analysis of the lower pendulum in the inertial frame is derived from the momentum moment theorem:

$$
f\prime_{2y}L\sin \theta _1+m_1gl_1\sin \theta _1-f\prime_{2x}L\cos \theta _1-m_1\ddot{x}_{1g}l_1\cos \theta _1+m_1\ddot{y}_{1g}l_1\sin \theta _1=I_1\ddot{\theta}_1
$$

Similarly, for the analysis of the upper pendulum rod:

$$
m_2gl_2\sin \theta _2-m_2l_2\cos \theta _2\ddot{x}_{2g}+m_2l_2\sin \theta _2\ddot{y}_{2g}=I_2\ddot{\theta}_2
$$

According to Newton's laws of motion, force analysis is performed on the upper pendulum in both horizontal and vertical directions:

$$ f_{2x}=m_2\ddot{x}_{2g}=m_2(\ddot{x}+L\sin \theta _1+l_2\sin \theta _2)'' $$

$$ f_{2y}-m_2g=m_2\ddot{y}_{2g}=m_2(L\cos \theta _1+l_2\cos \theta _2)'' $$


Based on the above formulations, this gives the following set of differential equations:

$$
(M+m_1+m_2)\ddot{x}+(m_1l_1+m_2L)\cos \theta _1\ddot{\theta}_1+m_2l_2\cos \theta _2\ddot{\theta}_2-(Ml_1+M_2L)\sin \theta _1\dot{\theta}_{1}^{2}-M_2l_2\sin \theta _2\dot{\theta}_{2}^{2}=F
$$

$$
(m_1l_1+m_2L)\cos \theta _1\ddot{x}+(I_1+m_1l_{1}^{2}+m_2L^2)\ddot{\theta}_1+m_2Ll_2\cos\mathrm{(}\theta _2-\theta _1)\ddot{\theta}_2-m_2Ll_2\sin\mathrm{(}\theta _2-\theta _1)\dot{\theta}_{2}^{2}=(m_1l_1+m_2L)g\sin \theta _1
$$

$$
m_2l_2\cos \theta _2\ddot{x}+m_2Ll_2\cos\mathrm{(}\theta _2-\theta _1)\ddot{\theta}_1+m_2Ll_2\sin\mathrm{(}\theta _2-\theta _1)\dot{\theta}_{1}^{2}+(I_2+m_2l_{2}^{2})\ddot{\theta}_2=m_2gl_2\sin \theta _2
$$

## Constants

| $\ M[Kg]$ | $\ m_1[Kg]$ | $\ m_2[Kg]$ | $\ l_1[m]$ | $\ l_2[m]$ | $\ L_1[m]$ | $\ g[m/s^{2}]$ |  $\ F[N]$ |
|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|
| 2 | 0.5 | 0.5 | 0.2 | 0.2 | 0.4 | 9.8 | 2 |


## Analysis
**Q1.** Simulate the ODE equation of the system using the given constant based on the given matrix expression, describe the result you observe and, is this the desired behaviour?
```math
\left[ \begin{matrix}
	M_{11}&		M_{12}&		M_{13}\\
	M_{21}&		M_{22}&		M_{23}\\
	M_{31}&		M_{32}&		M_{33}\\
\end{matrix} \right] \left[ \begin{array}{c}
	\ddot{x}\\
	\ddot{\theta}_1\\
	\ddot{\theta}_2\\
\end{array} \right] +\left[ \begin{matrix}
	C_{11}&		C_{12}&		C_{13}\\
	C_{21}&		C_{22}&		C_{23}\\
	C_{31}&		C_{32}&		C_{33}\\
\end{matrix} \right] \left[ \begin{array}{c}
	\dot{x}\\
	\dot{\theta}_1\\
	\dot{\theta}_2\\
\end{array} \right] +\left[ \begin{array}{c}
	G_1\\
	G_2\\
	G_3\\
\end{array} \right] =\left[ \begin{array}{c}
	u\\
	0\\
	0\\
\end{array} \right] 
```

**Q2.** Setting the state vector and finding the equilibrium point for this system. Linearize the system around a stable equilibrium.


**Q3.** Simulate linearised system with state space method.


**Q4.** Compare the results between the linearized simulation and the numerical integration in Q1, what differences do you observe?


**Q5.** Design a controller that can move a double-inverted pendulum system to an equilibrium point from any initial position.
