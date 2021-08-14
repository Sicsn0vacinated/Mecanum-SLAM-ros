import os

#swap the port here
def swap(a, b):
    #tmp = a
    #a = b
    #b = tmp
    return b, a

def starboard_swap(motor):
    motor['A'], motor['B'] = swap(motor['A'], motor['B'])
    ### enable ports swap not checked
    motor['en0'], motor['en1'] = swap(motor['en0'], motor['en1'])
    return motor

# List wheelbase track radius in meter 
A1_param_ = [0.25, 0.29072, 0.07528]
PT_param_ = [0.17, 0.2, 0.04]
# Define Robot model kinematic parameter
# Useable variable: prototype, A1. Default: A1
# In os environment variable
VALID_MODEL_ = ['A1', 'PROTOTYPE']
try:  
    BOT_MODEL = os.environ['BOT_MODEL'].upper()
    if BOT_MODEL not in VALID_MODEL_:
        print("\tInvalid BOT_MODEL, choose one of %s.\nDefaulting to %s" % (VALID_MODEL_, VALID_MODEL_[0]))
        BOT_MODEL = VALID_MODEL_[0] 
except:
    print("robot model not set! defaulting to %s" % VALID_MODEL_[0])
    BOT_MODEL = VALID_MODEL_[0]

#setting param according to model
if BOT_MODEL == 'A1':
    (WHEELBASE_, TRACK_, WHEELRADIUS_) = tuple(A1_param_)
elif BOT_MODEL == 'PROTOTYPE':
    (WHEELBASE_, TRACK_, WHEELRADIUS_) = tuple(PT_param_)
else:
    (WHEELBASE_, TRACK_, WHEELRADIUS_) = tuple(PT_param_)

#starboard encoders CW turn negative: M2 M4
M1 = dict(
    A = 26,
    B = 23,#19,
    pwm = 7,#16,
    en0 = 20,
    en1 = 21,
    
)
M2 = dict(
    A = 6,
    B = 24,
    pwm = 8,
    en0 = 12,
    en1 = 16,
)
M3 = dict(
    A = 5,
    B = 10,
    pwm = 9,
    en0 = 19,
    en1 = 13,
)
M4 = dict(
    A = 14,
    B = 25,
    pwm = 11,
    en0 = 15,
    en1 = 18,
)

starboard_swap(M2)
starboard_swap(M4)
#M2['A'], M2['B'] = swap(M2['A'], M2['B']) 
#M4['A'], M4['B'] = swap(M4['A'], M4['B']) 
#in 2d array format
# listed as WHEEL_PORTS[0]->M1 ...
# M1[i] -> A B en0 en1
WHEEL_PORTS = [[m[key] for key in m] for m in [M1, M2, M3, M4]]

BOT_MAX_SPEED = 0.5 #m/s
BOT_MAX_SPEED_RPS = 18 # rad/s safe bound#20 # rad/s, tested about 25.12 
PWM_LINEAR_PROFILE = 100/18 # Theory 100/22
PWM_HZ = 70 # bot pwm frequency

# MPU6050?