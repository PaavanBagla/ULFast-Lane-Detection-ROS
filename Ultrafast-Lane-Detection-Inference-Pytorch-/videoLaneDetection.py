import cv2
import yt_dlp
from ultrafastLaneDetector import UltrafastLaneDetector, ModelType

model_path = "models/tusimple_18.pth"
model_type = ModelType.TUSIMPLE
use_gpu = False

# Initialize video using yt-dlp
videoUrl = 'https://youtu.be/2CIxM7x-Clc'
ydl_opts = {
    'format': 'bestvideo/best',  # Download the best available format
    'noplaylist': True,
    'outtmpl': 'temp_video.mp4',  # Save the video as temp_video.mp4
    'quiet': False,  # Show output for debugging
}

with yt_dlp.YoutubeDL(ydl_opts) as ydl:
    try:
        ydl.download([videoUrl])
    except Exception as e:
        print(f"Error downloading video: {e}")

cap = cv2.VideoCapture('temp_video.mp4')

if not cap.isOpened():
    raise ValueError("Unable to open video capture with file: temp_video.mp4")

# Initialize lane detection model
lane_detector = UltrafastLaneDetector(model_path, model_type, use_gpu)

cv2.namedWindow("Detected lanes", cv2.WINDOW_NORMAL)

while cap.isOpened():
    try:
        # Read frame from the video
        ret, frame = cap.read()
    except Exception as e:
        print(f"Error reading frame: {e}")
        continue

    if ret:
        # Detect the lanes
        output_img = lane_detector.detect_lanes(frame)
        cv2.imshow("Detected lanes", output_img)
    else:
        break

    # Press key q to stop
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

