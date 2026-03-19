from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import time

# ------------------------------
# CONFIGURATION
# ------------------------------
RESOLUTION = (640, 480)    # Width x Height
FRAMERATE = 15             # Frames per second
MJPEG_PORT = 8000          # Flask port
# ------------------------------

app = Flask(__name__)

# Initialize Picamera2
camera = Picamera2()
camera_config = camera.create_video_configuration(main={"size": RESOLUTION})
camera.configure(camera_config)
camera.start()
time.sleep(2)  # camera warm-up

def generate_frames():
    while True:
        frame = camera.capture_array()

        # Convert RGB->BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Rotate feed 180°
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    print(f"Starting MJPEG stream on port {MJPEG_PORT}, resolution {RESOLUTION}, framerate {FRAMERATE}fps")
    app.run(host='0.0.0.0', port=MJPEG_PORT, threaded=True)
