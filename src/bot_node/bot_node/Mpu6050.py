
from smbus2 import SMBus
from time import sleep, time_ns
from operator import add , sub   
import pathlib
import yaml 

channel = 1
Device_Address = 0x68
bus = SMBus(channel)



#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47



def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


class Mpu6050():
	def __init__(self):

		self.acc = [0.0, 0.0, 0.0]
		self.gyro = [0.0, 0.0, 0.0]
		self.prev_gyro = [0.0, 0.0, 0.0]
		self.r_speed = [0.0, 0.0, 0.0]
		self.calibration = [0.0] * 6
		self.prev_time =0.0
		self.bus = bus
		self.offset_filename = 'mpu6050_offsets.yaml'
		try:
			MPU_Init()
			self.read_()
		except:
			pass
		if pathlib.Path(self.offset_filename).exists():
			print("Loading %s to calibration" % self.offset_filename )
			self.load_offsets()
		else:
			self.calibrate()


	def read_(self):
		try:	
			#Read Accelerometer raw value
			acc_x = read_raw_data(ACCEL_XOUT_H)
			acc_y = read_raw_data(ACCEL_YOUT_H)
			acc_z = read_raw_data(ACCEL_ZOUT_H)

			#Read Gyroscope raw value
			gyro_x = read_raw_data(GYRO_XOUT_H)
			gyro_y = read_raw_data(GYRO_YOUT_H)
			gyro_z = read_raw_data(GYRO_ZOUT_H)

			#Full scale range +/- 250 degree/C as per sensitivity scale factor
			Ax = -acc_x/16384.0
			Ay = -acc_y/16384.0
			Az = -acc_z/16384.0

			Gx = gyro_x/131.0
			Gy = gyro_y/131.0
			Gz = gyro_z/131.0

			self.acc = [round(x, 5) for x in list( map(sub, [Ax, Ay, Az], self.calibration[0:3]))]
			self.gyro = [round(x, 5) for x in list(map(sub, [Gx, Gy, Gz], self.calibration[3:6]))]
			return [Ax, Ay, Az, Gx, Gy, Gz]
		except:
			return [0.0] * 6


	def get(self):
		return self.read_()
	def get_acc(self):
		self.read_()
		return [ac * 9.81 for ac in self.acc]

	def get_gyro(self):
		self.read_()
		return self.gyro

	def get_gyro_rad(self):
		self.read_()
		return [r * 0.0174 for r in self.gyro]
		
	def calibrate(self):
		self.calibration = [0.0] * 6
		for i in range(49):
			self.calibration = list( map(add, self.calibration, self.get()))
		self.calibration = [k/50 for k in self.calibration]

	def rotational_speed(self, duration):
		# Brokey
		#duration in sec
		if time_ns() / (10 ** 9) - self.prev_time > duration:
			self.read_()
			for i in range(len(self.gyro)):
				self.r_speed[i] = self.gyro[i] - self.prev_gyro[i]
				self.r_speed[i] /= (duration)
			self.prev_gyro = self.gyro
			self.prev_time = time_ns() / (10 ** 9)

		return self.r_speed
			

	def save_offsets(self, filename=None):
		if filename is None:
			filename = self.offset_filename
		with open(filename, 'w') as outfile:
			dict_ = dict(
				calibration = self.calibration
			)
			yaml.dump(dict_, outfile, default_flow_style=False)
		print("saving file to: %s" % filename)

	def load_offsets(self, filename=None):
		if filename is None:
			filename = self.offset_filename
		with open(filename, 'r') as stream:
			try:
				data = yaml.safe_load(stream)
				self.calibration = data['calibration']
			except yaml.YAMLERROR as exc:
				print(exc)

	def calibrate(self):

		def validate_accelerometer_calibration(axis, tolerance=0.05, g=1.0):
			# g=1.0 the axis should points toward the ground during validation
			return (abs(axis) - abs(g)) <= 0.05

		def calibrate_accelerometer(axis, g= 1.0):
			offset = 0.0
			try:
				while abs(axis) > 0.6*abs(g):
					print("calibrating....", end='\r')
					for i in range(49):
						offset += (axis - g)
					offset /= 50
					if validate_accelerometer_calibration((axis - offset)):
						print("calibration complete")
						return offset
					else:
						print("BAD Calibration, retrying", end='\r')
						return calibrate_accelerometer(axis, g)
			except:
				print("value not in range skipping")
				return 0.0
		# Begin calibration process.
		for i in range(49):
			self.calibration = list( map(add, self.calibration, self.read_()))

		self.calibration = [k/50 for k in self.calibration]

		print("calibrating Z axis, hold still..")
		self.calibration[2] = calibrate_accelerometer(self.read_()[2], -1.0)

		print("calibrating X axis, point the nose downwards")
		sleep(4)
		self.calibration[0] = calibrate_accelerometer(self.read_()[0], 1.0)

		print("calibrating Y axis, point the leftside(port) down")
		sleep(4)
		self.calibration[1] = calibrate_accelerometer(self.read_()[1], 1.0) 
		# Check gyro calibration
		print("Validating Z axis...",end='')
		if validate_accelerometer_calibration(self.acc[2]):
			print("OK! ", end='')
			self.save_offsets()
		else:
			print ("BAD CALIBRATION")

