import cv2

cap = cv2.VideoCapture("22008760-stereo.mp4")
ret, frame = cap.read()
cap.release()

if not ret:
    raise RuntimeError("Could not read video")

def click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Clicked:", x, y)

cv2.namedWindow("Stereo")
cv2.setMouseCallback("Stereo", click)
cv2.imshow("Stereo", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()