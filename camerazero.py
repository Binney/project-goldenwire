from picamera import PiCamera
from time import sleep
from gpiozero import Button
import os.path
from signal import pause

camera = PiCamera()
camera.start_preview(alpha=200)

sleep(5)

def next_available_filename():
	file_num = 1
	while os.path.isfile("test" + str(file_num) + ".jpg"):
		file_num += 1
	return "test" + str(file_num) + ".jpg"

def snap():
	print("Click!")
	filename = next_available_filename()
	camera.capture(filename)
	# TODO send via Slack
	print("done")

shutter = Button(2)
shutter.when_pressed = snap

pause()
