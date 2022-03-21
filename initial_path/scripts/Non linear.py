from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt

def getxy():
    # https://www.youtube.com/watch?v=egQAKdJsu7E
    m = GEKKO()  # initilze the obkject
    mass = 1
    g = 10

    # time steps
    nt = 100
    m.time = np.linspace(0, 2, nt)

    # initial values
    x = m.Var(value=-0.7, lb=-0.7, ub=1, fixed_initial=True)
    y = m.Var(value=1, lb=0, ub=1, fixed_initial=True)
    theta = m.Var(value=0, lb=0, ub=2, fixed_initial=True)

    m.fix_final(x, -0.3)
    m.fix_final(y, 0.5)
    m.fix_final(theta, np.pi / 2)

    u_x = m.Var(value=0, lb=0)
    u_y = m.Var(value=0, ub=0)
    u_theta = m.Var(value=0)


    m.fix_final(u_x, 0)
    m.fix_final(u_y, 0)
    m.fix_final(u_theta, 0)

    a_x = m.Var(value=0, lb=-10, ub=10)
    a_y = m.Var(value=0, lb=-10, ub=10)
    a_theta = m.Var(value=0, lb=-10, ub=10)
    f_x = m.Intermediate(a_x*mass)
    f_y = m.Intermediate((g + a_y)*mass)
    f_s = m.Intermediate(f_y * m.cos(theta) + f_x * m.sin(theta) + 0.2*a_theta)
    # f_r = m.Intermediate(f_y * m.sin(theta) + f_x * m.cos(theta) + 0.2*u_theta**2)

    p = np.zeros(nt)
    p[-1] = 1
    final = m.Param(value=p)

    m.Equation(x.dt() == u_x)
    m.Equation(y.dt() == u_y)
    m.Equation(theta.dt() == u_theta)


    # m.Equation(u_x.dt() == f_x / mass)
    # m.Equation(u_y.dt() == g - f_y / mass)

    m.Equation(u_x.dt() == a_x)
    m.Equation(u_y.dt() == a_y)
    m.Equation(u_theta.dt() == a_theta)

    # m.Obj(0.8*f_s**2+0.2*f_r**2)  # for maximazine put minus
    m.Obj(f_s**2)  # for maximazine put minus
    m.options.IMODE = 6
    m.solve(disp=True)

    plt.figure(figsize=(4, 3))
    plt.subplot(4, 1, 1)
    plt.plot(m.time, x.value, 'k--', label=r'$x$')
    plt.plot(m.time, y.value, 'r--', label=r'$Y$')
    plt.plot(m.time, theta.value, 'g--', label=r'$Theta$')
    # plt.plot(m.time, f_y.value, 'b--', label=r'$Fy$')
    plt.legend()
    plt.ylabel('m')
    plt.subplot(4, 1, 2)
    plt.plot(m.time, u_x.value, 'k--', label=r'$ux$')
    plt.plot(m.time, u_y.value, 'r--', label=r'$uY$')
    plt.plot(m.time, u_theta.value, 'g--', label=r'$uTheta$')
    # plt.plot(m.time, f_y.value, 'b--', label=r'$Fy$')
    plt.legend()
    plt.ylabel('m/2')
    plt.subplot(4, 1, 3)
    # plt.plot(m.time, f_s.value, 'g:', label=r'$f_shear$')
    plt.plot(m.time, a_x.value, 'r:', label=r'$a_x$')
    plt.plot(m.time, a_y.value, 'b:', label=r'$a_y$')
    plt.plot(m.time, a_theta.value, 'g:', label=r'$a_theta$')
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel(r'$m/s**2$')
    plt.subplot(4, 1, 4)
    plt.plot(m.time[1:], f_s.value[1:], 'g:', label=r'$F_shear$')
    plt.legend()
    plt.xlabel('Time')
    plt.ylabel('N')
    # plt.show()

    x = np.asarray(x.VALUE)
    y = np.asarray(y.VALUE)
    theta = np.asarray(theta.VALUE)
    theta = np.rad2deg(theta)+180
    return x, y, theta, nt

# https://apmonitor.com/wiki/index.php/Main/GekkoPythonOptimization

if __name__ == '__main__':

    x, y, theta, states = getxy()
    plt.figure(2)
    ax = plt.axes()
    # ax.scatter(x, y, alpha=0.5, s=200)
    # U = np.cos(theta * np.pi / 180)
    # V = np.sin(theta * np.pi / 180)
    # plt.quiver(x, y, U, V)
    # plt.show()

    for state in range(states):
        plt.xlim(-1, 0)
        plt.ylim(0, 1.2)
        U = np.cos(theta[state] * np.pi / 180)
        V = np.sin(theta[state] * np.pi / 180)
        ax.scatter(x[state], y[state], alpha=0.5, s=200)
        plt.quiver(x[state], y[state], U, V)
        plt.draw()
        plt.pause(0.1)
    plt.show()





