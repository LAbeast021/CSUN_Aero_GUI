import cv2

# Create a VideoCapture object to access the webcam
cap = cv2.VideoCapture(0)

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640, 480))

# Loop until the user presses 'q'
while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    # If the frame was read successfully
    if ret:
        # Write the frame to the output video file
        out.write(frame)

        # Display the resulting frame
        cv2.imshow('Webcam', frame)

    # Break the loop if the user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and VideoWriter objects
cap.release()
out.release()

# Destroy all windows
cv2.destroyAllWindows()
