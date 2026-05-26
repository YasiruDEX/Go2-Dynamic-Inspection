import cv2
import datetime
import os

# Save location
save_dir = r"E:\sem7\FYP\9_24\objectdetetction\record"
os.makedirs(save_dir, exist_ok=True)

# Select the correct camera index (try 0,1,2... depending on system)
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open Insta360 webcam stream")
    exit()

# Get default video properties
frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps          = cap.get(cv2.CAP_PROP_FPS)

if fps == 0:  # fallback if fps not detected
    fps = 30.0

print(f"Recording at {frame_width}x{frame_height}, {fps} FPS")

# Output filename with timestamp
out_filename = os.path.join(
    save_dir,
    datetime.datetime.now().strftime("insta360_record_%Y%m%d_%H%M%S.mp4")
)

# Define codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'XVID' for .avi
out = cv2.VideoWriter(out_filename, fourcc, fps, (frame_width, frame_height))

print("Press 'q' to stop recording...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame")
        break
    
    # Write the frame to file
    out.write(frame)

    # Show preview (optional)
    cv2.imshow('Insta360 Webcam Preview', frame)

    # Stop recording on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()
print(f"Recording saved to {out_filename}")

