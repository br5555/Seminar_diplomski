Header header

int8 motor_id	    	    # 1-front, 2-right, 3-back, 4-left

float32 force_M		        # measured force in Newton
float32 speed_M		        # measured rotational velocity radian per minute
float32 temperatureR_M	    # measured temperature right cylinder in Celsius degree
float32 temperatureL_M	    # measured temperature left cylinder in Celsius degree
float32 fuel_level_M 	    # measured fuel level in % (range 0-1)
float32 throttle_M 	    	# measured throttle (0-1)
int8 choke_M				# measured choke position (open/close)
float32 engine_hours		# total engine hours of gas motor

int8 ignition_S             # set ignition flag (0-off, 1-on)
int16 starter_ppm_S         # set ppm value for e-starter (range 1000-1900)
float32 speed_S		        # set reference for gm velocity in radian per second
float32 throttle_S		    # set throttle reference 
int8 speed_ctl_S		   	# set speed controller flag, 0 - speed controller off, 1 - on
int8 choke_S				# set value for choke