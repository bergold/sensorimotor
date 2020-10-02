#!/usr/bin/python

import serial
import argparse
import sys
from time import sleep

default_port = '/dev/ttyUSB0'
baudrate = 1000000
timeout_s = 0.1
sync = [255,255]


def check_board_id(bid):
	if int(bid) > 127 or int(bid) < 0:
		print("Board id '{0}' is out of range 0..127".format(bid))
		return False
	return True


def eat(ser):
	while ser.read(): pass


def assert_byte(ser, c):
	resp = ser.read() # read response byte
	return (resp and ord(resp) == c)


def receive_byte_sequence(ser, seq):
	checksum = 0
	for s in sync+seq:
		checksum += s
		if not assert_byte(ser, s):
			return False
	c = ser.read()
	return c and (checksum + ord(c)) % 256 == 0 # two's complement checksum, adds to zero


def send_byte_sequence(ser, seq):
	sendbuf = sync+seq
	checksum = (~sum(sendbuf) + 1) % 256
	sendbuf.append(checksum)
	assert(sum(sendbuf) % 256 == 0)
	ser.write(sendbuf)


def ping(ser, board_id):
	eat(ser)
	send_byte_sequence(ser, [224, board_id]) # send ping command 0xE0
	return receive_byte_sequence(ser, [225, board_id]) # check for response 0xE1

def set_id(ser, board_id, new_id):
	eat(ser)
	send_byte_sequence(ser, [112, board_id, new_id]) # send set_id command 0x70
	return receive_byte_sequence(ser, [113, new_id]) # check for response 0x71

def set_target_value(ser, board_id, value):
	eat(ser)
	send_byte_sequence(ser, [178, board_id, value]) # send set_target_value command 0xB2
	return


def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('-b', '--board' , default='')
	parser.add_argument('-p', '--port'  , default=default_port)
	parser.add_argument('-n', '--newid' )
	args = parser.parse_args()

	# open serial communication
	with serial.Serial(args.port, baudrate, timeout=timeout_s) as ser:
		print("Connected to port {0}\nwith baudrate {1}.\n".format(ser.port, ser.baudrate))

		if args.board and check_board_id(args.board):
			bid = int(args.board)
			print("Sending ping to board id {0}.".format(bid))
			if ping(ser, bid):
				print("Board {0} responded.".format(bid))
				
				try:

					while True:

						print("Sending target_value {0} to board id {1}.".format(0.2, bid))
						set_target_value(ser, bid, 0.2)
						sleep(3)
						print("Sending target_value {0} to board id {1}.".format(0.2, bid))
						set_target_value(ser, bid, -0.2)
						sleep(3)

				except KeyboardInterrupt:
					print("Aborted.")

			else:
				print("No response.")


	print("\n____\nDONE.")


if __name__ == "__main__": main()