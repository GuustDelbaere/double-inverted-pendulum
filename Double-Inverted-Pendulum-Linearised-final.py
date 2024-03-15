import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from scipy.integrate import odeint
import control as ct
import math
from numpy.linalg import inv, eig


def calculate_A_B(M1, m1, m2, l1, l2, L, g):
    #constants
    I1 = (m1*l1**2)/12
    I2 = (m2*l2**2)/12

    #Controller
    M0 = np.array([[M1 + m1 + m2, (m1*l1 + m2*L), m2*l2],
                   [(m1*l1 + m2*L), I1 + m1*l1**2 + m2*L**2, m2*L*l2],
                   [m2*l2, m2*L*l2, I2 + m2*l2**2]])
    M0inv = inv(M0)
    dGdth0 = np.array([[0, 0, 0],
                       [0, -(m1*l1 + m2*L)*g, 0],
                       [0, 0, -m2*g*l2]])
    D = -np.dot(M0inv,dGdth0)
    H = np.array([[1],
                  [0],
                  [0]])
    E = -np.dot(M0inv,H)

    A = np.concatenate([np.concatenate([np.zeros((3,3)),np.eye(3,3)],axis=1),np.concatenate([D,np.zeros((3,3))],axis=1)],axis=0)
    B = np.concatenate([np.zeros((3,1)),E],axis=0)
    return A, B


def ODEFUN(y, t, A, B, K):   

    # xdot, th1dot, th2dot, x, th1, th2 = y
    
    F = -K @ y[:, np.newaxis] 
    
    ydotdot = A @ y + B @ F.flatten()
    return ydotdot


def solve_riccati(A, B, Q, R, maxiter=150, eps=0.01):
    P = Q

    for i in range(maxiter):
        Pn = A.T @ P @ A - A.T @ P @ B @ \
            inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
        if (abs(Pn - P)).max() < eps:
            break
        P = Pn

    return Pn


def dlqr(A, B, Q, R):
    # first, try to solve the ricatti equation
    P = solve_riccati(A, B, Q, R)

    # compute the LQR gain
    K = inv(B.T @ P @ B + R) @ (B.T @ P @ A)

    eigVals, eigVecs = eig(A - B @ K) #inside unit circle

    return K, P, eigVals



#constants
M1 = 2
m1 = 0.5
m2 = 0.5
l1 = 0.2
l2 = 0.2
L = 0.4
g = 9.8

#cost matrices
Q = np.diag([2.0, 10.0, 10.0, 2.0, 25.0, 25.0])  # state cost matrix
R = np.diag([0.01])  # input cost matrix

#initial conditions
y0 = np.array([0, 0, np.pi/18, 0, 0, 0])

#time vector
t = np.linspace(0, 10, 60*10)


A, B = calculate_A_B(M1, m1, m2, l1, l2, L, g)

A_discrete = np.eye(6,6) + A*0.01
B_discrete = B*0.01

K, _, _ = dlqr(A_discrete, B_discrete, Q, R)
sol = odeint(ODEFUN, y0, t, args=(A, B, K))

x = sol[:,3].flatten()
th1 = sol[:,4].flatten()
th2 = sol[:,5].flatten()
print(K)


#plots
fig = plt.figure()

ax1 = fig.add_subplot(2, 3, 1)
ax1.plot(t, th1, 'r')
ax1.set_title('Variabelen')
ax1.set_xlabel('tijd')
ax1.set_ylabel('theta1')

ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(t, th2, 'b')
ax2.set_xlabel('tijd')
ax2.set_ylabel('theta2')

ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(t, x, 'g')
ax3.set_xlabel('tijd')
ax3.set_ylabel('x')


#animation
def pend1pos(x, th1):
    return [x, 0, x + (L*np.sin(th1)), L*np.cos(th1)] #[xBottom, yBottom, xTop, yTop]

def pend2pos(x, th1, th2):
    return [x + (L*np.sin(th1)), L*np.cos(th1), x + (L*np.sin(th1)) + (L*np.sin(th2)), L*np.cos(th1) + (L*np.cos(th2))] #[xBottom, yBottom, xTop, yTop]

ax4 = fig.add_subplot(2, 1, 2, aspect='equal', xlim=(-5, 5), ylim=(-1, 1))
ax4.set_title('Animatie')

bar = ax4.plot([-5, 5], [0, 0], linewidth=2, color='k')
x0Bottom, y0Bottom, x0Top, y0Top = pend1pos(y0[0], y0[1]) #initial position of pendulum 1
x1Bottom, y1Bottom, x1Top, y1Top = pend2pos(y0[0], y0[1], y0[2]) #initial position of pendulum 2
pendulum1, = ax4.plot([x0Bottom, x0Top], [y0Bottom, y0Top], linewidth=2)
pendulum2, = ax4.plot([x1Bottom, x1Top], [y1Bottom, y1Top], linewidth=2)
car = ax4.add_patch(patches.Rectangle((y0[0]-0.2,-0.1), 0.4, 0.2, linewidth=2, edgecolor='k', facecolor='k'))


def animate(i):
    xBottom, yBottom, xTop, yTop = pend1pos(x[i], th1[i])
    pendulum1.set_data([xBottom, xTop], [yBottom, yTop])
    xBottom, yBottom, xTop, yTop = pend2pos(x[i], th1[i], th2[i])
    pendulum2.set_data([xBottom, xTop], [yBottom, yTop])
    car.set_xy([x[i]-0.2,-0.1])

ani = animation.FuncAnimation(fig, animate, frames=range(len(t)), blit=False, interval=1000/60, repeat=False, save_count=len(t))

plt.tight_layout()
plt.show()