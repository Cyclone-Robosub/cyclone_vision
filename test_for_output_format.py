import cv2 as cv

cap = cv.VideoCapture(0)

# Try different fourcc codes for the onboard H.264
fourcc_codes = [
    cv.VideoWriter.fourcc(*'H264'),
    cv.VideoWriter.fourcc(*'MJPG'),
    cv.VideoWriter.fourcc(*'YUYV')
]

for fourcc in fourcc_codes:
    cap.set(cv.CAP_PROP_FOURCC, fourcc)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv.CAP_PROP_FPS, 30)
    
    width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv.CAP_PROP_FPS))
    
    print(f"Fourcc: {fourcc}, Resolution: {width}x{height}@{fps}")