import numpy as np

def Quaternion2Euler(quat):
    """
    converts a quaternion attitude to an euler angle attitude
    :param quat: the quaternion array [e0,e1,e2,e3]
    :return: the euler angle equivalent [phi, theta, psi] in a np.array
    """

    e0 = quat[0]
    e1 = quat[1]
    e2 = quat[2]
    e3 = quat[3]

    phi = np.arctan2(2*(e0*e1 + e2*e3), (e0**2 + e3**2 - e1**2 - e2**2))
    theta = np.arcsin(2*(e0*e2 - e1*e3))
    psi = np.arctan2(2*(e0*e3 + e1*e2), (e0**2 + e1**2 - e2**2 - e3**2))

    return np.array([phi, theta, psi])


def Euler2Quaternion(eul):
    """
    Converts an euler angle attitude to a quaternian attitude
    :param eul: Euler angle attitude array [phi, theta, psi]
    :return: Quaternian attitude [e0, e1, e2, e3] in np.array 
    """

    phi2 = eul[0]/2
    tha2 = eul[1]/2
    psi2 = eul[2]/2

    e0 = np.cos(psi2)*np.cos(tha2)*np.cos(phi2) + \
        np.sin(psi2)*np.sin(tha2)*np.sin(phi2)
    e1 = np.cos(psi2)*np.cos(tha2)*np.sin(phi2) - \
        np.sin(psi2)*np.sin(tha2)*np.cos(phi2)
    e2 = np.cos(psi2)*np.sin(tha2)*np.cos(phi2) + \
        np.sin(psi2)*np.cos(tha2)*np.sin(phi2)
    e3 = np.sin(psi2)*np.cos(tha2)*np.cos(phi2) - \
        np.cos(psi2)*np.sin(tha2)*np.sin(phi2)

    return np.array([e0, e1, e2, e3])


def Rotation2Euler(R):
    """
    Converts a rotation matrix (body to inertial) to euler angle attitude
    :param R: 3x3 Rotation matrix (body to inertial)
    :return: euler angle [phi, theta, psi] in a np.array 
    """
    theta = -np.arcsin(R[2,0])
    phi = np.arctan2( R[2,1]/np.cos(theta), R[2,2]/np.cos(theta) )
    psi = np.arctan2( R[1,0]/np.cos(theta), R[0,0]/np.cos(theta) )

    return np.array([phi, theta, psi])