# This message reports the velocity in the Vehicle Body Frame. By convention, the x-axis should point
# out the nose of the vehicle and represent the forward direction, while as the y-axis should point out the
# right hand side of the vehicle. Since this is a right handed system, z should point out the bottom of the
# vehicle. This message is only produced by inertial versions of Swift products and is not available from 
# Piksi Multi or Duro

Header header

geometry_msgs/Vector3 velocity 	# Velocity in x,y and z direction (mm/s)
float32 cov_xx			# Estimated variance in x (m^2)
float32 cov_xy			# Covariance of x and y (m^2)
float32 cov_xz		        # Covariance of x and z (m^2)
float32 cov_yy			# Estimated variance in y (m^2)
float32 cov_yz			# Covariance of y and z (m^2)
float32 cov_zz			# Estimated variance in z (m^2)
uint8 vel_mode         		# velocity mode flags
uint8 ins_mode			# inertial mode flags

#velocity mode and inertial mode flag 
uint8 VEL_MODE_INVALID     = 0
uint8 VEL_MODE_MEASURED    = 1
uint8 VEL_MODE_COMPUTED    = 2
uint8 VEL_MODE_DR          = 3 

uint8 INS_NAV_MODE_NONE    = 0
uint8 INS_NAV_MODE_USED    = 1
