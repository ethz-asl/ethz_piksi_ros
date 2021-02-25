# source: https://thepoorengineer.com/wp-content/uploads/2018/07/calibrateMag.txt
# Reads a ros bag and outputs magnetometer calibration
# Tip: Try to rotate magnetometer in all axes.

import pandas as pd
import numpy as np
import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg


def fitEllipsoid(magX, magY, magZ):
    a1 = magX ** 2
    a2 = magY ** 2
    a3 = magZ ** 2
    a4 = 2.0 * np.multiply(magY, magZ)
    a5 = 2.0 * np.multiply(magX, magZ)
    a6 = 2.0 * np.multiply(magX, magY)
    a7 = 2.0 * magX
    a8 = 2.0 * magY
    a9 = 2.0 * magZ
    a10 = np.ones(len(magX)).T
    D = np.array([a1, a2, a3, a4, a5, a6, a7, a8, a9, a10])

    # Eqn 7, k = 4
    C1 = np.array([[-1.0, 1, 1, 0, 0, 0],
                   [1, -1, 1, 0, 0, 0],
                   [1, 1, -1, 0, 0, 0],
                   [0, 0, 0, -4, 0, 0],
                   [0, 0, 0, 0, -4, 0],
                   [0, 0, 0, 0, 0, -4]])

    # Eqn 11
    S = np.matmul(D, D.T)
    S11 = S[:6, :6]
    S12 = S[:6, 6:]
    S21 = S[6:, :6]
    S22 = S[6:, 6:]

    # Eqn 15, find eigenvalue and vector
    # Since S is symmetric, S12.T = S21
    tmp = np.matmul(np.linalg.inv(C1), S11 - np.matmul(S12, np.matmul(np.linalg.inv(S22), S21)))
    eigenValue, eigenVector = np.linalg.eig(tmp)
    u1 = eigenVector[:, np.argmax(eigenValue)]

    # Eqn 13 solution
    u2 = np.matmul(-np.matmul(np.linalg.inv(S22), S21), u1)

    # Total solution
    u = np.concatenate([u1, u2]).T

    Q = np.array([[u[0], u[5], u[4]],
                  [u[5], u[1], u[3]],
                  [u[4], u[3], u[2]]])

    n = np.array([[u[6]],
                  [u[7]],
                  [u[8]]])

    d = u[9]

    return Q, n, d


def getMag(bag, topic):
    msgs = bag.read_messages(topics=[topic])
    N = bag.get_message_count(topic)
    data_mag = np.zeros([N, 4])

    for i, msg in enumerate(msgs):
        data_mag[i, 1:4] = np.array([
            msg.message.magnetic_field.x,
            msg.message.magnetic_field.y,
            msg.message.magnetic_field.z])
        data_mag[i, 0] = msg.message.header.stamp.secs + msg.message.header.stamp.nsecs * 1e-9

    # create pandas frame with index
    idx = pd.DatetimeIndex(data=pd.to_datetime(data_mag[:, 0], unit='s'))
    df = pd.DataFrame({'t': idx, 'x': data_mag[:, 1],
                       'y': data_mag[:, 2],
                       'z': data_mag[:, 3]})
    df = df.set_index('t')
    return df


def main():
    # get data from rosbag
    topic = ""      # set to topic with magnetometer data
    bagfile = ""    # set to bagifle
    bag = rosbag.Bag(bagfile, 'r')
    df_mag = getMag(bag)

    # current scale is in teslas -> convert to microteslas for better
    #   numerical stability of calculation
    magX = df_mag.values[:,0]*1E6
    magY = df_mag.values[:,1]*1E6
    magZ = df_mag.values[:,2]*1E6

    fig1 = plt.figure(1)
    ax1 = fig1.add_subplot(111, projection='3d')

    ax1.scatter(magX, magY, magZ, s=5, color='r')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    # plot unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax1.plot_wireframe(x, y, z, rstride=10, cstride=10, alpha=0.5)
    ax1.plot_surface(x, y, z, alpha=0.3, color='b')

    Q, n, d = fitEllipsoid(magX, magY, magZ)

    Qinv = np.linalg.inv(Q)
    b = -np.dot(Qinv, n)
    Ainv = np.real(1.0 / np.sqrt(np.dot(n.T, np.dot(Qinv, n)) - d) * linalg.sqrtm(Q))

    print("A_inv: ")
    print(Ainv)
    print()
    print("b")
    print(b)
    print()

    calibratedX = np.zeros(magX.shape)
    calibratedY = np.zeros(magY.shape)
    calibratedZ = np.zeros(magZ.shape)

    totalError = 0
    for i in range(len(magX)):
        h = np.array([[magX[i], magY[i], magZ[i]]]).T
        hHat = np.matmul(Ainv, h - b)
        calibratedX[i] = hHat[0]
        calibratedY[i] = hHat[1]
        calibratedZ[i] = hHat[2]
        mag = np.dot(hHat.T, hHat)
        err = (mag[0][0] - 1) ** 2
        totalError += err
    print("Total Error: %f" % totalError)

    fig2 = plt.figure(2)
    ax2 = fig2.add_subplot(111, projection='3d')

    ax2.scatter(calibratedX, calibratedY, calibratedZ, s=1, color='r')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    # plot unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax2.plot_wireframe(x, y, z, rstride=10, cstride=10, alpha=0.5)
    ax2.plot_surface(x, y, z, alpha=0.3, color='b')
    #plt.show()

if __name__ == '__main__':
    main()
