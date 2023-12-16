import numpy as np
# distance in mm
d1 = 0
d2 = 45.24+115
d3 = 132.19
d4 = 104.50
d5 = 83.12
d6 = 80 # need check
d7 = 20 # need check 


def forward_kinematics(theta : np.array):
    # Your forward kinematics function here, returns a 4x4 transformation matrix
    # Example:
    h01 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0, 0],
                  [np.sin(theta[0]), np.cos(theta[0]), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    
    h12 = np.array([[np.sin(theta[1]), np.cos(theta[1]), 0, d2],
                  [0, 0, 1, 0],
                  [np.cos(theta[1]), -np.sin(theta[1]), 0, 0],
                  [0, 0, 0, 1]])
    h23 = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0, d3],
                  [np.sin(theta[2]), np.cos(theta[2]), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    h34 = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0, d4],
                  [np.sin(theta[3]), np.cos(theta[3]), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    h45 = np.array([[-np.sin(theta[4]), 0, np.cos(theta[4]), d5],
                  [np.cos(theta[4]), 0, np.sin(theta[4]), 0],
                  [0, 1, 0, 0],
                  [0, 0, 0, 1]])
    h56 = np.array([[1, 0, 0, -d7],
                  [0, 1, 0, 0],
                  [0, 0, 1, d6],
                  [0, 0, 0, 1]])
    h06 = h01 @ h12 @ h23 @ h34 @ h45 @ h56
    return h06

def error(T_desired, theta):
    T_current = forward_kinematics(theta)
    return np.linalg.norm(T_current - T_desired)

def jacobian(theta, epsilon=1e-6):
    J = np.zeros((6, 5))
    for i in range(5):
        d_theta = np.zeros(5)
        d_theta[i] = epsilon
        J[:, i] = (forward_kinematics(theta + d_theta) - forward_kinematics(theta - d_theta)) / (2 * epsilon)
    return J

def inverse_kinematics(T_desired, theta_initial, max_iterations=100, tolerance=1e-6):
    theta = theta_initial.copy()
    for _ in range(max_iterations):
        T_current = forward_kinematics(theta)
        e = np.linalg.norm(T_current - T_desired)
        print ("iteration: ", _)
        if e < tolerance:
            break
        J = jacobian(theta)
        delta_theta = np.linalg.pinv(J).dot(T_desired[:3, 3] - T_current[:3, 3])
        theta += delta_theta
    return theta

# Define the desired end effector pose as a 4x4 transformation matrix
T_desired = np.array([[-1, 0, 0, 174],
                      [0, -1, 0, 0],
                      [0, 0, -1, 10],
                      [0, 0, 0, 1]])

# Initial guess for joint angles
theta_initial = np.array([0, 0, 0, 0,0])

# Compute the inverse kinematics
theta_solution = inverse_kinematics(T_desired, theta_initial)

print(f"Final joint angles: {theta_solution}")

# def main():
#     # angles = np.array([1,2,3,4,0])
#     angles = np.array([0,0,0,0,0])
#     x = forward_kinematics(angles)
#     print (x)

# if __name__ == "__main__":
#     main()