#!/usr/bin/env python3

from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt
import pickle
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Header
from config import V_Params


class optimal_control:

    def __init__(self):
        rospy.init_node('initial_path_node')
        self.pointc_publisher = rospy.Publisher("/initial_path_pcl", PointCloud2, queue_size=1)
        self.theta_publisher = rospy.Publisher("/theta_array", Float32MultiArray, queue_size=10)
        # https://www.youtube.com/watch?v=egQAKdJsu7E
        # https://apmonitor.com/wiki/index.php/Main/GekkoPythonOptimization
        self.m = GEKKO()  # initialize the object
        self.show = False
        self.live_show = False
        x, y, theta, nt = self.set_conditions()
        if self.live_show:
            self.live_plotter(x, y, theta, nt)

        pcl = self.convert_point_cloud_msg(x, y)
        # Main loop
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            seconds = time.time()
            local_time = time.ctime(seconds)
            print("Published new pcl on time:", local_time)
            self.pointc_publisher.publish(pcl)
            temp = Float32MultiArray(data=list(np.float32(theta)))
            self.theta_publisher.publish(temp)
            rate.sleep()


    def set_conditions(self):
        v_params = V_Params()
        mass = 1  # object mass
        g = 9.8  # gravitation acceleration
        nt = 100  # time steps

        # x_start = v_params.x_start
        # x_end = v_params.x_end
        # y_start = v_params.y_start
        # y_end = v_params.y_end

        x_start = -0.6
        x_end = -0.3
        y_start = 0.8
        y_end = 0.3


        self.m.time = np.linspace(0, 2, nt)

        # Set initial values (initial value, lower boundary and upper boundary)
        x = self.m.Var(value=x_start, lb=-0.7, ub=1, fixed_initial=True)
        y = self.m.Var(value=y_start, lb=0, ub=1, fixed_initial=True)
        theta = self.m.Var(value=0, lb=0, ub=2, fixed_initial=True)
        u_x = self.m.Var(value=0, lb=0)
        u_y = self.m.Var(value=0, ub=0)
        u_theta = self.m.Var(value=0)
        a_x = self.m.Var(value=0, lb=-10, ub=10)
        a_y = self.m.Var(value=0, lb=-10, ub=10)
        a_theta = self.m.Var(value=0, lb=-10, ub=10)

        # Set Final conditions
        self.m.fix_final(x, x_end)
        self.m.fix_final(y, y_end)
        self.m.fix_final(theta, np.pi / 2)
        self.m.fix_final(u_x, 0)
        self.m.fix_final(u_y, 0)
        self.m.fix_final(u_theta, 0)

        # set relations
        f_x = self.m.Intermediate(a_x * mass)
        f_y = self.m.Intermediate((g + a_y) * mass)
        f_s = self.m.Intermediate(f_y * self.m.cos(theta) + f_x * self.m.sin(theta) + 0.2 * a_theta)
        # f_r = m.Intermediate(f_y * m.sin(theta) + f_x * m.cos(theta) + 0.2*u_theta**2)

        p = np.zeros(nt)
        p[-1] = 1
        final = self.m.Param(value=p)

        self.m.Equation(x.dt() == u_x)
        self.m.Equation(y.dt() == u_y)
        self.m.Equation(theta.dt() == u_theta)

        # m.Equation(u_x.dt() == f_x / mass)
        # m.Equation(u_y.dt() == g - f_y / mass)

        self.m.Equation(u_x.dt() == a_x)
        self.m.Equation(u_y.dt() == a_y)
        self.m.Equation(u_theta.dt() == a_theta)

        # m.Obj(0.8*f_s**2+0.2*f_r**2)  # for maximazine put minus
        self.m.Obj(f_s ** 2)  # for maximazine put minus
        self.m.options.IMODE = 6
        # self.m.options.SOLVER = 1
        # self.m.options.MAX_ITER = 10000

        self.m.solve(disp=True)

        if self.show:
            plt.figure(figsize=(4, 3))
            plt.subplot(4, 1, 1)
            plt.plot(self.m.time, x.value, 'k--', label=r'$x$')
            plt.plot(self.m.time, y.value, 'r--', label=r'$Y$')
            plt.plot(self.m.time, theta.value, 'g--', label=r'$Theta$')
            # plt.plot(m.time, f_y.value, 'b--', label=r'$Fy$')
            plt.legend()
            plt.ylabel('m')
            plt.subplot(4, 1, 2)
            plt.plot(self.m.time, u_x.value, 'k--', label=r'$ux$')
            plt.plot(self.m.time, u_y.value, 'r--', label=r'$uY$')
            plt.plot(self.m.time, u_theta.value, 'g--', label=r'$uTheta$')
            # plt.plot(m.time, f_y.value, 'b--', label=r'$Fy$')
            plt.legend()
            plt.ylabel('m/2')
            plt.subplot(4, 1, 3)
            # plt.plot(m.time, f_s.value, 'g:', label=r'$f_shear$')
            plt.plot(self.m.time, a_x.value, 'r:', label=r'$a_x$')
            plt.plot(self.m.time, a_y.value, 'b:', label=r'$a_y$')
            plt.plot(self.m.time, a_theta.value, 'g:', label=r'$a_theta$')
            plt.legend()
            plt.xlabel('Time')
            plt.ylabel(r'$m/s**2$')
            plt.subplot(4, 1, 4)
            plt.plot(self.m.time[1:], f_s.value[1:], 'g:', label=r'$F_shear$')
            plt.legend()
            plt.xlabel('Time')
            plt.ylabel('N')
            plt.show()

        x = np.asarray(x.VALUE)
        y = np.asarray(y.VALUE)
        theta = np.asarray(theta.VALUE)
        theta = np.rad2deg(theta) + 180

        # update to pickle file
        data = np.copy(x)
        data = np.vstack((data, y))
        data = np.vstack((data, theta))
        with open('data_py3.pkl', 'wb') as f:
            pickle.dump(data, f)
        pickle.dump(data, open("data_py2.pkl", "wb"), protocol=2)
        print("saved pickle files")
        return x, y, theta, nt

    @staticmethod
    def convert_point_cloud_msg(x, z):
        y = -x
        x = np.zeros(len(x))
        # x = - x
        # y = np.zeros(len(x))
        xyz_arr = np.column_stack((x.T, y.T))
        xyz_arr = np.column_stack((xyz_arr, z.T))
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]

        header = Header()
        header.frame_id = "base_link"
        header.stamp = rospy.Time.now()
        temp = xyz_arr[:, 2]
        points = np.column_stack((xyz_arr, temp.T))

        pc2 = point_cloud2.create_cloud(header, fields, points)
        return pc2

    @staticmethod
    def live_plotter(x, y, theta, nt):

        plt.figure(2)
        ax = plt.axes()
        for state in range(nt):
            plt.xlim(-1, 0)
            plt.ylim(0, 1.2)
            U = np.cos(theta[state] * np.pi / 180)
            V = np.sin(theta[state] * np.pi / 180)
            ax.scatter(x[state], y[state], alpha=0.5, s=200)
            plt.quiver(x[state], y[state], U, V)
            plt.draw()
            plt.pause(0.1)
        plt.show()


if __name__ == '__main__':
    test = optimal_control()
