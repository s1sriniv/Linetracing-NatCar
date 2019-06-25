# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# NATCAR CODE OF SANKAR SRINIVASAN
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


THETA_GAIN = 2
RHO_GAIN = 1.8/72 # 72 is max distance from center bottom of image to anywhere on image
P_GAIN = 1
D_GAIN = -0.0008 # THIS NEEDS TO BE NEGATIVE!!! D TERM IS FOR "PREDICTIVE DAMPING"

# (XREF, YREF)  is reference point from which distance to line is calculated
XREF = 40
YREF = 60

NEW_WEIGHT = 1 # Used for weighted moving average to smooth oscillations in output



# FOR SPEED CONTROL PID (RPS = rotations per second)
MAX_RPS = 15
MIN_RPS = 10

SPEED_P_GAIN = 10


THRESHOLD = (240, 255) # Grayscale threshold for dark things...
BINARY_VISIBLE = True # Does binary first so you can see what the linear regression
                      # is being run on... might lower FPS though.

MAG_THRESHOLD = 8



import sensor, image, time, math, pyb
from pyb import UART, Pin, Timer


# LEDs for debugging
red = pyb.LED(1)
blue = pyb.LED(3)
green = pyb.LED(2)


# Setting up UART for Bluetooth
uart = UART(3, 9600, timeout_char = 10000)
uart.init(9600, bits=8, parity=None, stop=1)


# MOTOR AND SERVO INIT CODE
inA = pyb.Pin("P0", pyb.Pin.OUT_PP)
inB = pyb.Pin("P1", pyb.Pin.OUT_PP)

inA.low()
inB.low()

tim = Timer(4, freq=300) # Frequency in Hz

tim2 = Timer(2, freq=200)
ch1 = tim2.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent=20)

ch2 = tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width=4500)
cmd = ""


#INTERRUPT HANDLER FOR HALL EFFECT SENSOR (speed PID control)
def callback1(t):
    global pulse
    global counter
    global RPS

    pulse += 1
    if(pulse == 2): # Using two diametrically opposite magnets on wheel for Hall Effect detection
        counter += 1
        RPS += 1
        pulse = 0

pulse = 0
counter = 0


# HALL EFFECT SENSOR PIN
pin = pyb.Pin("P9", pyb.Pin.IN)
#time.sleep(500)
extint = pyb.ExtInt(pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_DOWN, callback=callback1)



sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time = 2000)
clock = time.clock()




# Using line object, following function returns shortest distance from center of image window
# to extended line's left boundary, as well as two important deviations: theta_deviation, which is the deviation
# of the camera alignment from the straight line, and rho_deviation, which is the deviation
# of the alignment from the perpendicular normal to the left boundary of the line. Both angular deviations could
# be more aptly described as the "SHORTEST TURN ANGLE TO ALIGN WITH RESPECTIVE DIRECTION",
# where the two directions in question are clearly orthogonal.

# "distance" = SHORTEST DISTANCE FROM POSITION TO INNER TURN (+8 PX) OF DISCOVERED LINE. This
# is so that car stays on the inside track (going left).
def line_theta_rho_error(line, x_ref, y_ref):
    if line.rho() < 0:
        if line.theta() < 90: # Quadrant 3 (UNUSED!!)
            print("why the hell is this shit in Q3...")

            distance, theta_deviation, rho_deviation = (0, 0, 0) # Dummy values
        else: # Quadrant 4
            theta_deviation = -(360 - (line.theta() + 180)) # Negative sign since dev. is to the left

            if (((line.y1() + line.y2())/2) < (3/4*(line.x1() + line.x2())/2)): # Q4 CASE 1
                distance = distance_from_line(line, x_ref, y_ref)

                if (distance - 8 < 0):
                    distance = abs(distance - 8)
                    rho_deviation = -(180 - (90 - (360 - (line.theta() + 180)))) # Negative sign since dev. is to the left
                else:
                    distance = distance - 8
                    rho_deviation = 90 - (360 - (line.theta() + 180)) # Positive sign since dev. is to the right

            else: # Q4 CASE 2
                 rho_deviation = -(90 + (360 - (line.theta() + 180))) # Negative sign since dev. is to the left
                 distance = distance_from_line(line, x_ref, y_ref) + 8

    else:
        if line.theta() < 90: # Quadrant 1
            theta_deviation = line.theta() # Positive sign since dev. is to the right

            if (((line.y1() + line.y2())/2) < (-3/4*(line.x1() + line.x2())/2 + 60)): # Q1 CASE 1
                distance = distance_from_line(line, x_ref, y_ref)

                if (distance - 8) < 0:
                    distance = abs(distance - 8)
                    rho_deviation = 180 - (90 - line.theta()) # Positive sign since dev. is to the right
                else:
                    distance = distance - 8
                    rho_deviation = -(90 - line.theta()) # Negative sign since dev. is to the left

            else: # Q1 CASE 2
                rho_deviation = 90 + line.theta() # Positive sign since dev. is to the right
                distance = distance_from_line(line, x_ref, y_ref) + 8

        else: # Quadrant 2
            theta_deviation = -(180 - line.theta()) # Negative sign since dev. is to the left

            if (((line.y1() + line.y2())/2) < (3/4*(line.x1() + line.x2())/2)): # Q2 CASE 2
                distance = distance_from_line(line, x_ref, y_ref)

                if (distance - 8) < 0:
                    distance = abs(distance - 8)
                    rho_deviation = -(180 - (90 - (180 - line.theta()))) # Negative sign since dev. is to the left
                else:
                    distance = distance - 8
                    rho_deviation = 90 - (180 - line.theta()) # Positive sign since dev. is to the right

            else: # Q2 CASE 1
                rho_deviation = -(90 + (180 - line.theta())) # Negative sign since dev. is to the left
                distance = distance_from_line(line, x_ref, y_ref) + 8

    return distance, theta_deviation, rho_deviation



def distance_from_line(line, x_ref, y_ref):
    if line.length() > 0:
        projection_coeff = ((x_ref*(line.x2() - line.x1()) + y_ref*(line.y2() - line.y1())) -
            (line.x1()*(line.x2() - line.x1()) + line.y1()*(line.y2() - line.y1())))/(line.length()*line.length())

        length = math.sqrt((line.x1() + projection_coeff*(line.x2() - line.x1()) - x_ref)*
            (line.x1() + projection_coeff*(line.x2() - line.x1()) - x_ref) + (line.y1() + projection_coeff*
            (line.y2() - line.y1()) - y_ref)*(line.y1() + projection_coeff*(line.y2() - line.y1()) - y_ref))
    else:
        length = 50 # Max possible length

    return length





old_output = 90

# FOLLOWING VARIABLES ARE FOR FOUR-POINT DIFFERENCE EQUATION FOR FILTERED D-TERM
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
old_error_minus_1 = 0
old_error_minus_2 = 0
old_error_minus_3 = 0
old_error_minus_4 = 0


derivative = 0

time_deriv_elapsed = pyb.millis()
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

min_degree = 0
max_degree = 179

RPS = 0 # Comment this out if you use speed PID w/ Hall Effect, though this probably doesn't matter w/ interrupts



go_check = 0
start = pyb.millis() # Used for RPS calculation
speed_pulse_width = 20
while(True):
    # STOP CODE (measures distance to stop by counting rotations of wheel using Hall Effect interrupt)
    # We have a better stop code using "blobs" (SEE VIKRAM'S CODE FOR CHECKOFF 2!!!)
    if(counter > 440):
        inA.high()
        inB.high()

    # BLUETOOTH CONTROL
    if uart.any():
        print("UART")
        cmd = (str)(bytes.decode(uart.read()))
        print(cmd)

        if cmd == "g": #go

            counter = 0
            inA.high()
            inB.low()
            go_check = 1
            cmd = "x"

        elif cmd == "q": #hard brake
            inA.high()
            inB.high()
            go_check = 0

        if cmd == "-": #SLOW DOWN
            if ch1.pulse_width_percent() > 2:
                ch1.pulse_width_percent(ch1.pulse_width_percent()-1)
                uart.write(str(ch1.pulse_width_percent())+ ' ')
                cmd = "x"

        if cmd == "=": #SPEED UP
            if ch1.pulse_width_percent() < 100:
                ch1.pulse_width_percent(ch1.pulse_width_percent()+1)
                uart.write(str(ch1.pulse_width_percent())+ ' ')
                cmd = "x"

        if(cmd == 'd'): #increase D gain
           D_GAIN = D_GAIN + 2
           uart.write(str(D_GAIN)+ 'D ')
           cmd = 'n'

        if(cmd == 'c'): #decrease D gain
           D_GAIN = D_GAIN - 2
           uart.write(str(D_GAIN)+ 'D ')
           cmd = 'n'

        if(cmd == 'p'): #increase P gain
           P_GAIN = P_GAIN + 0.1
           uart.write(str(P_GAIN)+ 'P ')
           cmd = 'n'

        if(cmd == 'l'):
           P_GAIN = P_GAIN - 0.1
           uart.write(str(P_GAIN)+ 'P ')
           cmd = 'n'

        if(cmd == 'r'): #increase D gain
              counter = 0
              cmd = 'n'



    #ALGORITHM

    clock.tick()

    img = sensor.snapshot()
    img.binary([THRESHOLD])
    img.erode(1)



    # FAST LINEAR REGRESSION
    future_line = img.get_regression([(255, 255)], robust=False)
    print_string = ""

    if (go_check == 1):

        if (img.get_statistics()[0] > 1): # Makes sure car hasn't completely lost the course (used for drifting!!!)

            if future_line and (future_line.magnitude() >= MAG_THRESHOLD): # Setting mag threshold is essentially
                                                                             # doing pretzel rejection

                    img.draw_line(future_line.line(), color=127)

                    dist, theta_dev, rho_dev = line_theta_rho_error(future_line, XREF, YREF)

                    new_error = THETA_GAIN*theta_dev + RHO_GAIN*dist*rho_dev

                    time_minus_0 = pyb.millis() # Current time

                    P_output = P_GAIN*new_error


                    # APPROXIMATING DERIVATIVE TERM USING FOUR-POINT DIFFERENCE EQUATION
                    # (See Lance's lecture notes)
                    if (pyb.elapsed_millis(time_deriv_elapsed) > 10):
                        delta_result_new_1 = new_error - old_error_minus_1
                        delta_result_new_2 = new_error - old_error_minus_2
                        delta_result_new_3 = new_error - old_error_minus_3
                        delta_result_new_4 = new_error - old_error_minus_4

                        delta_result_1_2 = old_error_minus_1 - old_error_minus_2
                        delta_result_1_3 = old_error_minus_1 - old_error_minus_3
                        delta_result_1_4 = old_error_minus_1 - old_error_minus_4

                        delta_result_2_3 = old_error_minus_2 - old_error_minus_3
                        delta_result_2_4 = old_error_minus_2 - old_error_minus_4

                        delta_result_3_4 = old_error_minus_3 - old_error_minus_4

                        derivative = 1/2*(delta_result_new_3/30 + delta_result_1_2/20)*1000

                        print("DERIVATIVE: %f" % (derivative))


                        old_error_minus_4 = old_error_minus_3
                        old_error_minus_3 = old_error_minus_2
                        old_error_minus_2 = old_error_minus_1
                        old_error_minus_1 = new_error

                        if (new_error >= 0):
                            if (derivative >= 0):
                                D_output = D_GAIN*derivative
                            else:
                                D_output = 0
                        else:
                            if (derivative <= 0):
                                D_output = D_GAIN*derivative
                            else:
                                D_output = 0

                        time_deriv_elapsed = pyb.millis()


                    # Steering PID output used to set servomotor angle
                    new_output = 90 + max(min(int(P_output + D_output), 90), -90)


                    # Setting servomotor angle based on previous steering PID output
                    angle_pulse_width = int(-240/9*(NEW_WEIGHT*new_output + (1 - NEW_WEIGHT)*old_output) + 9000)
                    ch2.pulse_width(angle_pulse_width)

                    # Desired speed is linear function of current servomotor turn angle (we would like slower speeds
                    # for hard turns and higher speed for straightaways, i.e. minimal turning)
                    desired_speed = MAX_RPS + (MIN_RPS - MAX_RPS)/2000*abs(angle_pulse_width - 6600) # In RPS


                    # SPEED PID CODE
                    if (RPS != 0):
                       speed_error = desired_speed - RPS*5 # Actual RPS = RPS*5, since resets to 0 every 200 ms

                       speed_pulse_width = max(min(int(SPEED_P_GAIN*speed_error), 40), 20)

                       ch1.pulse_width_percent(desired_speed) # Use "speed_pulse_width" if you want to do true
                                                               # PID using Hall Effect

                    old_output = new_output

                    blue.off()
                    red.off()
                    green.off()



                    print_string = "Line Ok - turn %d - line t: %d, r: %d, mag: %d, pix: %d" % (new_output, future_line.theta(),
                        future_line.rho(), future_line.magnitude(), img.get_statistics()[0])
            else:
                print("No line")

                red.on() # RED LED ON

                # Resetting derivative terms
                old_error_minus_1 = 0
                old_error_minus_2 = 0
                old_error_minus_3 = 0
                old_error_minus_4 = 0

        else: # RAN COMPLETELY AWAY FROM COURSE
            # Mock drifting code to turn and find line (usually after speeding on straightaway)

            if (old_output > 90):
                ch2.pulse_width(4200)
                ch1.pulse_width_percent(20)
            else:
                ch2.pulse_width(9000)
                ch1.pulse_width_percent(20)

            green.on()

            # Resetting derivative terms
                old_error_minus_1 = 0
                old_error_minus_2 = 0
                old_error_minus_3 = 0
                old_error_minus_4 = 0



        # RPS CALCULATION FOR SPEED CONTROL (see Hall Effect callback)
        if(pyb.elapsed_millis(start) > 200): #sampling speed every 200 MS
            #print("Rotation per second", counter)
            #uart.write(str(RPS*5) + ' ')
            RPS = 0
            start = pyb.millis()


        #print("FPS %f, SPEED %f, %s" % (clock.fps(), speed_pulse_width, print_string))
        uart.write(str(counter)+ ' ')

