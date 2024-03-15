import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from scipy.integrate import odeint


def ODEFUN(y, t, M1, m1, m2, l1, l2, L, g):
    #constants
    I1 = (m1*l1**2)/12
    I2 = (m2*l2**2)/12

    xdot, th1dot, th2dot, x, th1, th2 = y

    #matrices
    M = np.array([[M1 + m1 + m2, (m1*l1 + m2*L)*np.cos(th1), m2*l2*np.cos(th2), 0, 0, 0],
                  [(m1*l1 + m2*L)*np.cos(th1), I1 + m1*l1**2 + m2*L**2, m2*L*l2*np.cos(th1-th2), 0, 0, 0],
                  [m2*l2*np.cos(th2), m2*L*l2*np.cos(th1-th2), I2 + m2*l2**2, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])
    C = np.array([[0, -(m1*l1 + m2*L)*np.sin(th1)*th1dot, -m2*l2*np.sin(th2)*th2dot, 0, 0, 0],
                  [0, 0, m2*L*l2*np.sin(th1-th2)*th2dot, 0, 0, 0],
                  [0, -m2*L*l2*np.sin(th1-th2)*th1dot, 0, 0, 0, 0],
                  [-1, 0, 0, 0, 0, 0],
                  [0, -1, 0, 0, 0, 0],
                  [0, 0, -1, 0, 0, 0]])
    G = np.array([[0],[-(m1*l1 + m2*L)*g*np.sin(th1)],[-m2*g*l2*np.sin(th2)],[0],[0],[0]])
    H = np.array([[1],[0],[0],[0],[0],[0]])
    K = np.array([-2.74388574, 498.46120322, -558.27105486, -9.16229856, 1.66375002, -55.52626603])
    F = -np.dot(K, y)

    ydotdot = np.dot(np.linalg.inv(M),(H.flatten()*F) - G.flatten() - np.dot(C,y))
    return ydotdot

#constants
M1 = 2
m1 = 0.5
m2 = 0.5
l1 = 0.2
l2 = 0.2
L = 0.4
g = 10

y0 = np.array([0, 0, np.pi/18, 0, 0, 0])

t = np.linspace(0, 10, 60*10)

sol = odeint(ODEFUN, y0, t, args=(M1, m1, m2, l1, l2, L, g))

x = sol[:,0].flatten()
th1 = sol[:,1].flatten()
th2 = sol[:,2].flatten()


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