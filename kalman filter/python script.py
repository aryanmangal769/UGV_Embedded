# The full Kalman filter, consisting of prediction and correction step.


from math import sin, cos, pi, atan2, sqrt
from numpy import *


# the class of Extended kalman filter having correction and prediction steps to fuse motor ticks odometry and imu odometry
class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_covariance):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_covariance=measurement_covariance

    #This is the function which calculates next state given current state and control input.
    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])


    #This function calculates the Jacobian of function g mentioned above with
    #  respect to state variables at current state and  control input.
    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control
        if r != l:
            alpha = (r - l) / w
            R = l/alpha

            g1 = (R + (w/2)) * (cos(theta + alpha) - cos(theta))
            g2 = (R + (w/2)) * (sin(theta + alpha) - sin(theta))
            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])
        else:
            m = array([[1.0, 0.0, -l * sin(theta)], [0.0, 1.0, l* cos(theta)], [0.0, 0.0, 1.0]])
        return m

    #This function calculates the Jacobian of function g mentioned above 
    # with respect to control variables at current state and  control input.
    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        if r != l:
            
            alpha = (r - l) / w

            wr = (w*r)/((r-l)**2)
            wl = (w*l)/((r-l)**2)
            r2l = (r+l)/(2*(r-l))

            g1_l = wr * (sin(theta+alpha)-sin(theta)) - r2l * cos(theta+alpha)
            g2_l = wr * (-cos(theta+alpha)+cos(theta)) - r2l * sin(theta+alpha)
            g3_l = - (1/w)

            g1_r = -wl * (sin(theta+alpha)-sin(theta)) + r2l * cos(theta+alpha)
            g2_r = -wl * (-cos(theta+alpha)+cos(theta)) + r2l * sin(theta+alpha)
            g3_r = 1 / w 

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])
            
            
        else:
            g1_l = .5 * (cos(theta) + (l/w)*sin(theta))
            g2_l = .5 * (sin(theta) - (l/w)*cos(theta))
            g3_l = - 1/w

            g1_r = .5 * ((-l/w)*sin(theta) + cos(theta))
            g2_r = .5 * ((l/w)*cos(theta) + sin(theta))
            g3_r = 1 / w 

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])                        
        return m


    # this function gives the majaor and minor axis of error ellipse having given the covarience matrix
    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        
    
    
    # The prediction step of the Kalman filter.
    def predict(self, control):
        # covariance' = G * covariance * GT + R
        # G=dg_state
        # where R = V * (covariance in control space) * VT.
        #v=dg_control
        # Covariance in control space depends on move distance.
        left, right = control

        alpha_1 = self.control_motion_factor
        alpha_2 = self.control_turn_factor
        
        #Calculation of control covarience having given motion error ration and slip error ratio
        g2l = (alpha_1 * left)**2 + (alpha_2 * (left-right))**2
        g2r = (alpha_1 * right)**2 + (alpha_2 * (left-right))**2

        Sigma_control = diag([g2l,g2r])
        Vt = self.dg_dcontrol( self.state,control,self.robot_width )#!!
        VtT = Vt.T

        Sigma_covariance = self.covariance
        Gt = self.dg_dstate( self.state,control,self.robot_width )#!!
        GtT = Gt.T
        # Note that the transpose of a Numpy array G is expressed as G.T,
        # and the matrix product of A and B is written as dot(A, B).
        # Writing A*B instead will give you the element-wise product, which
        # is not intended here.
        
        
        self.covariance = dot(dot(Gt,Sigma_covariance),GtT) + dot(dot(Vt,Sigma_control),VtT)
        self.state = self.g(self.state,control,self.robot_width)
        #the predicted state and covariences of kalman filter


    #The correction step of the Kalman filter.
    def correct(self, measurement):

        # The measurement matrix H is identity in this case

        #covarience of measurement
        Q =  self.measurement_covariance
        
        # K, from self.covariance, H, and Q.
        Sigma_t = self.covariance
        Kt =  dot(Sigma_t,linalg.inv(Sigma_t + Q))

        # linalg.inv(...) to compute the inverse of a matrix.
        
       
        innovation = measurement- self.state

        # the predicted measurement and the actual measurement of alpha may have
        #  an offset of +/- 2 pi.
        innovation[2] = (innovation[2] + pi) % (2*pi) - pi

        # new self.state.
        mu_t = self.state + dot(Kt, innovation)
        self.state = mu_t

        # the new self.covariance. eye(3) to get a 3x3
        #  identity matrix.
        self.covariance = dot( (eye(3) - Kt), Sigma_t)

if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0


    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
   
    # Measured start position.
    initial_state = array([0,0,0])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])

    # Covariance of measurement
    measurement_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])

    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,measurement_covariance)

    
    states = []
    covariances = []

    # Get motor ticks of the instatnt

    motor_ticks=array([0,0])
    control =motor_ticks*ticks_to_mm
    kf.predict(control)

    #Get the imu reading of the instant

    measurement=array([0,0,0])
    kf.correct(measurement)
    states.append(kf.state)
    covariances.append(kf.covariance)
        
   