float32 position_x            
float32 position_y            # reference y position in NED frame
float32 position_z            # reference z position in NED frame

float32 velocity_x            # reference x velocity in NED frame
float32 velocity_y            # reference y velocity in NED frame
float32 velocity_z            # reference z velocity in NED frame

float32 acceleration_x				# reference x acceleration in NED frame
float32 acceleration_y   			# reference y acceleration in NED frame
float32 acceleration_z				# reference z acceleration in NED frame

float32 roll                  # roll reference (manual mode) in radians, should be ignored if position controllers in AUTO mode
float32 pitch                 # pitch reference (manual mode) in radians, should be ignored if position controllers in AUTO mode
float32 yaw                   # yaw reference
float32 yaw_rate              # yaw rate reference
float32 yaw_manual        	  # [-1,1] , used in manual yaw mode

float32 throttle              # throttle ref (manual mode) in percentage (0-1), should be ignored if height controller in AUTO mode

