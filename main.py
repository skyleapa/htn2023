import mediapipe as mp
import cv2
import pyfirmata
import time

port = 'COM7'
board = pyfirmata.Arduino(port)
# Define the pin to which the servo is connected (must be a PWM pin)
servo_pin1 = board.get_pin('d:11:s')  # 'd' for digital, 's' for servo
servo_pin2 = board.get_pin('d:10:s')  # 'd' for digital, 's' for servo
servo_pin3 = board.get_pin('d:9:s')  # 'd' for digital, 's' for servo
servo_pin4 = board.get_pin('d:6:s')  # 'd' for digital, 's' for servo
servo_pin5 = board.get_pin('d:5:s')  # 'd' for digital, 's' for servo
servo_pin6 = board.get_pin('d:3:s')  # 'd' for digital, 's' for servo

# Set the initial position of the servo (0 degrees)
servo_position = 0

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

my_drawing_specs = mp_drawing.DrawingSpec(color = (0, 255, 0), thickness = 1)

# 0 is front facing, 1 is back facing, 2 is usb webcam
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)

mp_face_mesh = mp.solutions.face_mesh

def main():
    timeSinceWrite = 0
    with mp_face_mesh.FaceMesh(
            max_num_faces = 1,
            refine_landmarks = True,
            min_detection_confidence = 0.5,
            min_tracking_confidence = 0.5
        ) as face_mesh:

        while cap.isOpened():
            success, image = cap.read()
            if not success:
                break

            results = face_mesh.process(image)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:

                    upperLip = face_landmarks.landmark[13].y
                    lowerLip = face_landmarks.landmark[14].y

                    # print(str(lowerLip - upperLip))
                    if lowerLip - upperLip > 0.05: 
                        print("open")
                    else:
                        print("closed")

                    x = face_landmarks.landmark[0].x
                    y = face_landmarks.landmark[0].y
                    z = face_landmarks.landmark[0].z
                    print(x)
                    if ( time.time() - timeSinceWrite > 0.3):
                        if (x > 0 and x < 1):
                            print(" =============================== CHANGE ANGLE " + str(x * 180))
                            servo_pin1.write(x*180)
                            servo_pin2.write(x*180)
                            servo_pin3.write(x*180)
                            servo_pin4.write(x*180)
                            servo_pin5.write(x*180)
                            servo_pin6.write(x*180)
                            timeSinceWrite = time.time()

                    # print('face_landmarks:', face_landmarks)

                    # the gray lines
                    # mp_drawing.draw_landmarks(
                    #     image = image,
                    #     landmark_list = face_landmarks,
                    #     connections = mp_face_mesh.FACEMESH_TESSELATION,
                    #     landmark_drawing_spec = None,
                    #     connection_drawing_spec = mp_drawing_styles
                    #     .get_default_face_mesh_tesselation_style()
                    # )
                    
                    # the green lines
                    mp_drawing.draw_landmarks(
                        image = image,
                        landmark_list = face_landmarks,
                        connections = mp_face_mesh.FACEMESH_CONTOURS,
                        landmark_drawing_spec = None,
                        connection_drawing_spec = my_drawing_specs
                    )
                    
            cv2.imshow("My video capture", cv2.flip(image, 1))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

        
