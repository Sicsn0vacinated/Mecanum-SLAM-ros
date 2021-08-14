import time

from shared_param.common_param import WHEEL_PORTS, WHEELBASE_, TRACK_, WHEELRADIUS_
# New Ubuntu Kernel uses lGPIO
import RPi.GPIO as GPIO
from encoder import Encoder


def callback(*argv):
    for arg in argv:
        print("value {}".format( arg), end='\r')

class Encoder2(Encoder):
    # wheel_ports from wheel port configuration
    def __init__(self, wheel_ports):
        
        #self.pulse_speed = [0, 0, 0, 0]
        self.prev_values = [0, 0, 0, 0]
        #self.value = [0, 0, 0, 0]
        self.t0s = [0, 0, 0, 0]
        self.loop_time = 50 #50 ms and not shorter than 16.9 ms because thats is the fastest theoretical monitor refresh rate
        self.speeds = [0, 0, 0, 0]
        self.wheel_ports = wheel_ports


        """
        ###Swap starboard AB in cofig file ###  
        for i in range(self.num_wheels):
            if (i+1) % 2 == 0:
                self.encoders.append(
                    Encoder(wheel_ports[i][0],
                     wheel_ports[i][1], None)
                )
            else:
                # Port side encoders
                self.encoders.append(
                    Encoder(wheel_ports[i][1], 
                    wheel_ports[i][0], None)
                )
        """
        GPIO.setmode(GPIO.BCM)
        self.encoders = [Encoder(wheel_ports[i][0],
                wheel_ports[i][1], None)
                for i in range(len(wheel_ports))]
    
    
    def callback(self, en_id, *argv):
        # Update value and calculate speed
        for arg in argv:
            self.value[en_id] = arg
            print("encoder {} : {}".format(en_id, arg), end='\r')

    def get_encoder_rad(self):
        # return wheel position in radian from en_c * 2PI/300 
        # constrain to -PI to PI, so mod PI
        #time.sleep(0.01) # magic break? 
        return [round(
            c.getValue() * 0.019 % (2*3.1416)
            , 2) 
            for c in self.encoders]

    def get_encoder_rps(self, duration):
        curr_time = time.time_ns() / (10 ** 6)
        if curr_time - self.t0s[0] > duration:
            for i in range(len(self.wheel_ports)):
                value = self.encoders[i].getValue()
                self.speeds[i] = (value - self.prev_values[i]) / \
                    (curr_time - self.t0s[i]) * 19.15
                self.prev_values[i] = value
                self.t0s[i] = time.time_ns() / (10 ** 6) 
        return [speed for speed in self.speeds]

    def cal_r_velocity(self, duration = 50, \
        wheelbase = WHEELBASE_, track = TRACK_, wheel_radius = WHEELRADIUS_):
        # odom from forward kinematics from encoders
        self.get_encoder_rps(duration)
        fraction = 2 / (wheelbase + track)
        v_x = wheel_radius / 4 *( self.speeds[0]  + self.speeds[1] + self.speeds[2] + self.speeds[3])
        v_y = wheel_radius / 4 *(-self.speeds[0]  + self.speeds[1] + self.speeds[2] - self.speeds[3])
        w_z = wheel_radius / 4 *(fraction) * (-self.speeds[0] + self.speeds[1] - self.speeds[2] + self.speeds[3])
        return v_x, v_y, w_z
    #def getValue(self):
    #    return super().getValue()
    def print_encoder_counts(self):
        [print("M{} : {}".format(idx, encoder.getValue()))
         for idx, encoder in enumerate(self.encoders)]
    def print_encoder_speeds(self):
        #print("M1 : {:.2f} rps".format(self.speeds[0]), end='\r')
        [print("M{} : {:.2f} m/s".format(idx+1, rps*0.04), end='\n') 
            for idx, rps in enumerate(self.speeds)]

    
    """
    

    t_start = 0
    prev_value = 0
    def callback(value):
        self.prev_vals = value
        time_stamp = time.time_ns() / (10 ** 6) - t_start
    #loop_time = 50 # milisec
    def timer(loop_time):
        # loop time  50 milisec
        t_start = time.time_ns() / (10 ** 6)
        start_value = self.encoders[wheel].getValue()
        if time.time_ns() / (10 ** 6) - t_start >= looptime: # time reached 50 ms
            curr_value = self.encoders[wheel].getValue()
            distance = start_value - self.encoders[wheel].getValue()
            start_value = self.encoders[wheel].getValue()
            t_start = time.time_ns()
            omega_rad = distance / loop_time *1000 / 52.2 
            print("speed {}".format(omega_rad))

    t_start = time.time_ns() / (10 ** 6)
    start_value = e1.getValue()
    #loop_time = 50 # milisec
    def timer(en, loop_time):
        # loop time  50 milisec
        #t_start = time.time_ns() / (10 ** 6)
        if time.time_ns() / (10 ** 6) - t_start > loop_time: # time reached 50 ms

            distance = start_value - en.getValue()
            omega_rad = distance / loop_time *1000 / 52.2 
            t_start = time.time_ns() / (10 ** 6)
            start_value = en.getValue()
            print("speed {}".format(omega_rad))   
    """

def calculate_odometry(encoders, duration = 50, \
    wheelbase = WHEELBASE_, track = TRACK_, wheel_radius = WHEELRADIUS_):
    # odom from forward kinematics from encoders
    w = encoders.get_encoder_rps(duration)
    fraction = 2 / (wheelbase + track)
    v_x = wheel_radius / 4 *( w[0]  + w[1] + w[2] + w[3])
    v_y = wheel_radius / 4 *(-w[0]  + w[1] + w[2] - w[3])
    w_z = wheel_radius / 4 *(fraction) * (-w[0] + w[1] - w[2] + w[3])

    print('v_x: {:.2f} v_y: {:.2f} w_z: {:.2f}'.format(v_x, v_y, w_z), end='\r')

def main():
    print('Hi from encoder_mecanum.')
    GPIO.setmode(GPIO.BCM)
    en_all = Encoder2(WHEEL_PORTS)
    while 1:
        time.sleep(.5)
        #calculate_odometry(en_all, duration = 50)
        v_x, v_y, w_z = en_all.cal_r_velocity(50)
        print('v_x: {:.2f} v_y: {:.2f} w_z: {:.2f}'.format(v_x, v_y, w_z), end='\r')

        #en_all.print_encoder_speeds()
        #[print("M{} : {:.2f} m/s".format(idx + 1, speed*0.04)) for idx, speed in enumerate(en_all.get_encoder_rps())]
        #print("M1: {} ".format(en_all.get_encoder_rps()[0]), end='\r')
        #en_all.get_encoder_rps()
        #print(en_all.prev_values[0])
        #en_all.print_encoder_speeds()


if __name__ == '__main__':
    main()
