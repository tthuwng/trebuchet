from typing import Tuple

import numpy as np
from matplotlib import pyplot as plt


def calc_phiDD(phi, theta, phiD, thetaD, thetaDD, L2, L3, m, g):
    return (
        -2 * L2 * L3 * thetaDD * np.cos(theta + phi)
        + 2 * L3 * L2 * thetaD * (thetaD + phiD) * np.sin(theta + phi)
        + m * g * L3 * np.cos(phi)
        - 2 * L3 * L2 * phiD * thetaD * np.sin(theta + phi)
    ) / (m * L3**2)


def calc_thetaDD(theta, phi, thetaD, phiD, phiDD, L1, L2, L3, M, m, g):
    return (
        -M * g * L1 * np.cos(theta)
        + m * g * L2 * np.cos(theta)
        - 2 * L3 * L2 * thetaD * phiD * np.sin(theta + phi)
        - 2 * L3 * L2 * phiDD * np.cos(theta + phi)
        + 2 * L3 * L2 * phiD * (thetaD + phiD) * np.sin(theta + phi)
    ) / (M * L1**2 + m * L2**2)


def trebuchet(
    NP: int, t: np.array, dt: float, params: dict
) -> Tuple[np.array, np.array, np.array, np.array]:
    # Extract parameters from the params dictionary
    g = 9.81  # m/s^2  (Acceleration of Gravity on earth)
    M = float(params["M"])  # kg    (Mass of the counterweight)
    m = float(params["m"])  # kg      (Mass of the projectile)
    L1 = float(params["L1"])  # m    (Length of the counterweight arm)
    L2 = float(params["L2"])  # m    (Length of the throwing arm)
    L3 = float(params["L3"])  # m     (Length of the rope/sling)
    h = float(params["h"])  # m    (Height of the counterweight)

    phiInit = float(params["phiInit"]) * np.pi / 180  # -rad (initial sling angle)
    thetaInit = float(params["thetaInit"]) * np.pi / 180  # rad (initial arm angle)
    launchTheta = float(params["launchAngle"]) * np.pi / 180  # rad (Launch angle)

    # Initial Angular Velocity
    phiDInit = 0
    thetaDInit = 0

    phiDD = np.zeros([NP])
    phiD = np.zeros([NP])
    phi = np.zeros([NP])

    thetaDD = np.zeros([NP])
    thetaD = np.zeros([NP])
    theta = np.zeros([NP])

    yM = np.zeros([NP])
    xM = np.zeros([NP])

    ym = np.zeros([NP])
    xm = np.zeros([NP])

    # Initializing position and velocity
    phi[0] = phiInit
    phiD[0] = phiDInit

    theta[0] = thetaInit
    thetaD[0] = thetaDInit

    yM[0] = L1 * np.sin(thetaInit) + h
    xM[0] = L1 * np.cos(thetaInit)

    ym[0] = -L2 * np.sin(thetaInit) - L3 * np.sin(phiInit) + h
    xm[0] = -L2 * np.cos(thetaInit) + L3 * np.cos(phiInit)

    # Verlet Initialization
    phiDD[0] = calc_phiDD(
        phi[0], theta[0], phiD[0], thetaD[0], thetaDD[0], L2, L3, m, g
    )
    phiOld = phi[0] - phiD[0] * dt + 0.5 * phiDD[0] * dt**2
    phiNow = phi[0]
    phiNew = 0

    thetaDD[0] = calc_thetaDD(
        theta[0], phi[0], thetaD[0], phiD[0], phiDD[0], L1, L2, L3, M, m, g
    )
    thetaOld = theta[0] - thetaD[0] * dt + 0.5 * thetaDD[0] * dt**2
    thetaNow = theta[0]
    thetaNew = 0

    j = 0

    while thetaNow > -launchTheta:  # Trebuchet Motion
        j += 1
        phi[j] = phiNow
        theta[j] = thetaNow

        phiDD[j] = calc_phiDD(
            phi[j], theta[j], phiD[j], thetaD[j], thetaDD[j], L2, L3, m, g
        )
        thetaDD[j] = calc_thetaDD(
            theta[j], phi[j], thetaD[j], phiD[j], phiDD[j], L1, L2, L3, M, m, g
        )

        phiNew = 2 * phiNow - phiOld + dt**2 * phiDD[j]
        phiOld = phiNow
        phiNow = phiNew

        thetaNew = 2 * thetaNow - thetaOld + dt**2 * thetaDD[j]
        thetaOld = thetaNow
        thetaNow = thetaNew

        phiD[j + 1] = phiD[j] + phiDD[j] * dt
        thetaD[j + 1] = thetaD[j] + thetaDD[j] * dt

        yM[j] = L1 * np.sin(thetaNow) + h
        xM[j] = L1 * np.cos(thetaNow)
        ym[j] = -L2 * np.sin(thetaNow) - L3 * np.sin(phiNow) + h
        xm[j] = -L2 * np.cos(thetaNow) + L3 * np.cos(phiNow)

    launchYm = ym[j]
    launchXm = xm[j]

    ymDInit = -L2 * thetaD[j] * np.cos(theta[j]) - L3 * phiD[j] * np.cos(phi[j])
    xmDInit = L2 * thetaD[j] * np.sin(theta[j]) - L3 * phiD[j] * np.sin(phi[j])

    c = 0
    while j < (NP - 1):  # Begin Projectile Motion
        j = j + 1
        c = c + 1
        ym[j] = ymDInit * (c) * dt - g / 2 * ((c) * dt) ** 2 + launchYm
        xm[j] = xmDInit * (c) * dt + launchXm

        if 10 < ym[j] < 11:
            print(xm[j])

    return xm, ym, theta, t


## Parameters
# params = {
#     "M": 550 * 133,  # Mass of the counterweight
#     "m": 550,  # Mass of the projectile
#     "L1": 10 * 1 / 10,  # Length of the counterweight arm
#     "L2": 10 * 9 / 10,  # Length of the throwing arm
#     "L3": 10 * 2 / 10,  # Length of the rope/sling
#     "h": 10,  # Height of Pivot
#     "launchAngle": np.pi * 3 / 10 * (180 / np.pi),  # Launch angle in degrees
# }

## Time
# NP = 2**12
# i = np.linspace(1, NP, NP)
# total_time = 10
# dt = total_time / NP
# t = i * dt

# xm, ym, theta, t = trebuchet(NP, t, dt, params)
# plt.figure()
# plt.plot(xm, ym, color="red", label="Position")
# plt.title(
#     "Trebuchet Motion",
#     fontsize="large",
#     loc="left",
#     fontweight="bold",
#     style="normal",
#     family="monospace",
# )
# plt.legend(loc="upper right")
# plt.xlabel("x projectile (m)")
# plt.ylabel("y projectile (m)")
# plt.ylim(-0, 170)
# plt.xlim(-20, 150)

# plt.show()
