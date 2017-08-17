import smbus
import RPi.GPIO as io
import sys
import math
import time

dt = time.time()
Q = 0.1
R = 5
P00 = 0.1
P11 = 0.1
P01 = 0.1

address = 0x68
power_on = 0x6b
bus = smbus.SMBus(1)
bus.write_byte_data(address, power_on, 0)

io.setmode(io.BOARD)
pins = [7, 11, 12, 13, 15, 16, 18, 22, 29, 31, 32, 33, 35, 36, 38, 40]
for num in range(len(pins)):
	io.setup(pins[num], io.OUT)
	io.output(pins[num], 0)

def read_byte(adr):
	return bus.read_byte_data(address, adr)

def read_word(adr):
	high = bus.read_byte_data(address, adr)
	low = bus.read_byte_data(address, adr+1)
	val = (high << 8) + low
	return val

def read_word_2c(adr):
	val = read_word(adr)
	if (val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val

def dist(a, b):
	return math.sqrt((a*a)+(b*b))

def accel_y_rotation(x, y, z):
	radians = math.atan2(x, dist(y, z))
	return -math.degrees(radians)

def accel_x_rotation(x, y, z):
	radians = math.atan2(y, dist(x, z))
	return math.degrees(radians)

def gyro_y_rotation(gyro_pitch, gyro_x, dt):
	gyro_pitch = gyro_pitch + (gyro_x * dt)
	return gyro_pitch

def gyro_x_rotation(gyro_roll, gyro_y, dt):
	gyro_roll = gyro_roll + (gyro_y * dt)
	return gyro_roll

def LED_on(angle, deg, pin):
	if angle > deg:
		io.output(pins[pin], 1)
	else:
		io.output(pins[pin], 0)

def LED_on2(angle, deg, pin):
	if angle < deg:
		io.output(pins[pin], 1)
	else:
		io.output(pins[pin], 0)

init_accel_x = read_word_2c(0x3b)
init_accel_y = read_word_2c(0x3d)
init_accel_z = read_word_2c(0x3f)
gyro_pitch = accel_x_rotation(init_accel_x, init_accel_y, init_accel_z)
gyro_roll = accel_y_rotation(init_accel_x, init_accel_y, init_accel_z)
predicted_pitch = gyro_pitch
predicted_roll = gyro_roll

while 1:
	gyro_x = read_word_2c(0x43) / 131
	gyro_y = read_word_2c(0x45) / 131
	gyro_z = read_word_2c(0x47) / 131

	accel_x = read_word_2c(0x3b) / 16384.0
	accel_y = read_word_2c(0x3d) / 16384.0
	accel_z = read_word_2c(0x3f) / 16384.0

	dt = dt - time.time()

	accel_pitch = accel_y_rotation(accel_x, accel_y, accel_z)
	gyro_pitch = gyro_y_rotation(gyro_pitch, gyro_x, dt)
	accel_roll = accel_x_rotation(accel_x, accel_y, accel_z)
	gyro_roll = gyro_x_rotation(gyro_roll, gyro_y, dt)

	predicted_pitch = predicted_pitch + gyro_pitch
	predicted_roll = predicted_roll - gyro_roll

	P00 += dt * (2 * P01 + dt * P11)
	P01 += dt * P11
	P00 += dt * Q
	P11 += dt * Q
	Kk0 = P00 / (P00 + R)
	Kk1 = P01 / (P01 + R)

	predicted_pitch += (accel_pitch - predicted_pitch) * Kk0
	predicted_roll += (accel_roll - predicted_roll) * Kk0

	P00 *= (1 - Kk0)
	P01 *= (1 - Kk1)
	P11 -= Kk1 * P01

	pitch = str(round(predicted_pitch, 2))
	roll = str(round(predicted_roll, 2))

	sys.stdout.write("\r{}     {}".format(pitch, roll))
	sys.stdout.flush()
	
	LED_on(predicted_pitch, 15, 0)
	LED_on(predicted_pitch, 30, 1)
	LED_on(predicted_pitch, 45, 2)
	LED_on(predicted_pitch, 60, 3)

	LED_on2(predicted_pitch, -15, 4)
	LED_on2(predicted_pitch, -30, 5)
	LED_on2(predicted_pitch, -45, 6)
	LED_on2(predicted_pitch, -60, 7)

	LED_on(predicted_roll, 15, 8)
	LED_on(predicted_roll, 30, 9)
	LED_on(predicted_roll, 45, 10)
	LED_on(predicted_roll, 60, 11)

	LED_on2(predicted_roll, -15, 12)
	LED_on2(predicted_roll, -30, 13)
	LED_on2(predicted_roll, -45, 14)
	LED_on2(predicted_roll, -60, 15)
