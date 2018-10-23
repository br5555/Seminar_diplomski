#
# Add comment.
#

uint8   mode                            # mode of the controller use auto or manual

float32 gain_proportional      	        # proportional gain
float32 gain_integral      	            # integral gain
float32 gain_derivative      	        # derivative gain

float32 sample_rate                     # controller sample time

float32 setpoint			            # controller setpoint
float32 feedback			            # controller measured value

float32 output                          # controller output
float32 output_proportional             # proportional controller output
float32 output_integral                 # integral controller output
float32 output_derivative               # derivative controller output
