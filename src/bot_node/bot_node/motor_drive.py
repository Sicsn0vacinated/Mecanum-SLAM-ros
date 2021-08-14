import time
import RPi.GPIO as GPIO 
from shared_param.common_param import \
    WHEEL_PORTS, WHEELBASE_, TRACK_, WHEELRADIUS_, \
    PWM_LINEAR_PROFILE, BOT_MAX_SPEED, BOT_MAX_SPEED_RPS, PWM_HZ

wheelbase = WHEELBASE_
track = TRACK_
wheel_radius = WHEELRADIUS_

class Motor:
    def __init__(self, pwm, ena, enb):

        # 00 free 01 10 forward backward 11 brake
        self.mode = '00'
        self.duty_cycles = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.chan_list= (ena, enb)

        GPIO.setup((ena, enb, pwm),  GPIO.OUT)
        self.pwm = GPIO.PWM(pwm, PWM_HZ)

        GPIO.output((ena, enb), GPIO.LOW)
        self.pwm.start(self.duty_cycles)
        #self.pwm.stop()
    def shutdown(self):
        GPIO.cleanup()


    def change_mode(self, newmode):
        if isinstance(newmode, str) and len(newmode) == 2:
            self.mode = newmode
            GPIO.output(self.chan_list, (int(self.mode[0]), int(self.mode[1])))
        else:
            print("mode incorrect")

    def drive(self, dc):
        self.pwm.ChangeDutyCycle(dc)
        self.duty_cycles = dc
    
    def simple_linear_speed_control(self, velocity):

        pwm_dc = linear_profiler(abs(velocity))
        
        if (velocity > 0):
            # positive -> forward mode 01 on H bridge
            self.change_mode('01')
        elif (velocity < 0):
            self.change_mode('10')
        else:
            self.change_mode('00')
        #print("duty cycle {}".format(pwm_dc))
        self.drive(pwm_dc)

def Motor_Init():
    return [Motor(p[2], p[3], p[4]) for p in WHEEL_PORTS]

def bound(x, out_min = 0, out_max = 100):
    #r =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    r = x
            # check constrain
    if r > out_max:
        r = out_max
    elif r < out_min:
        r = out_min
    return r

def linear_profiler(x, m = PWM_LINEAR_PROFILE, min = 12 , max = 90):
    # rps to pwm 
    pwm = x * m
    # no speed change in rps
    if pwm < min:
        return pwm
    elif pwm > max:
        pwm = 100
    # with 
    return bound(pwm)
    





def main():
    """
    motors = [Motor(p[2], p[3], p[4]) for p in WHEEL_PORTS]
    [m.change_mode('01') for m in motors]
    i = 0
    try:
        while True:
            for motor in motors:
                for dc in range(0, 100, 5):
                    motor.drive(dc)
                    time.sleep(0.01)
                for dc in range(100, -1, 5):
                    motor.drive(dc)
                    time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    """

if __name__ == '__main__':
    main()

