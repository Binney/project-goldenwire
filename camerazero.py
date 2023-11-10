from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.start_preview(alpha=200)

sleep(5)

def snap():
	filename = "images/test.jpg"
	camera.capture(filename)
	# TODO send via Slack

snap()
camera.stop_preview()
