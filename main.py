import mediapipe as mp
import cv2
import pyfirmata
import time
import numpy as np
from scipy.optimize import minimize

port = 'COM7'
board = pyfirmata.Arduino(port)
length1 = 10 #(cm) SET
length2 = 12 #SET
spoonLength = 11 #SET

ANGLEBDEFAULT = 0
ANGLE1DEFAULT = 0
ANGLE2DEFAULT = 0
ANGLE3DEFAULT = 0
# CONFIGURE PINS !!
# servo_pin1 = board.get_pin('d:11:s')
# servo_pin2 = board.get_pin('d:10:s')
# servo_pin3 = board.get_pin('d:9:s')
# servo_pin4 = board.get_pin('d:6:s')
# servo_pin5 = board.get_pin('d:5:s')
# servo_pin6 = board.get_pin('d:3:s')

#initial position
base1Pin = board.get_pin('d:11:s')  
base1Angle = ANGLEBDEFAULT #SET DEFAULT ANGLE

point1Pin = board.get_pin('d:10:s') 
point1RevPin = board.get_pin('d:9:s')
point1Angle = ANGLE1DEFAULT

point2Pin = board.get_pin('d:6:s')  
point2RevPin = board.get_pin('d:5:s')  
point2Angle = ANGLE2DEFAULT

point3Pin = board.get_pin('d:3:s')  
global point3Angle
point3Angle = ANGLE3DEFAULT

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

my_drawing_specs = mp_drawing.DrawingSpec(color = (0, 255, 0), thickness = 1)

# 0 is front facing, 1 is back facing, 2 is usb webcam
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)

mp_face_mesh = mp.solutions.face_mesh

def baseRotation(dAngle):
    global base1Angle

    if (base1Angle+dAngle > 270 or base1Angle+dAngle < 0):
        raise ValueError("Base servo: Illegal Angle")
    base1Pin.write(base1Angle + dAngle)
    base1Angle =+ dAngle 

def point1Rotation(dAngle):
    global point1Angle

    if (point1Angle+dAngle > 180 or point1Angle+dAngle < 0):
        print(str(point1Angle+dAngle))
        raise ValueError("Servos 1: Illegal Angle")
    point1Pin.write(point1Angle + dAngle)
    point1RevPin.write(180 - point1Angle + dAngle)
    point1Angle += dAngle 
    
def point2Rotation(dAngle):
    global point2Angle

    if (point2Angle+dAngle > 180 or point2Angle+dAngle < 0):
        print(str(point2Angle+dAngle))
        raise ValueError("Servos 2: Illegal Angle")
    point2Pin.write(point2Angle + dAngle)
    point2RevPin.write(180 - point2Angle + dAngle)
    point2Angle =+ dAngle 

def point3Rotation(dAngle):
    global point3Angle

    if (point3Angle+dAngle > 180 or point3Angle+dAngle < 0):
        print(str(point3Angle+dAngle))
        raise ValueError("Servo 3: Illegal Angle")
    point3Pin.write(point3Angle + dAngle)
    point3Angle =+ dAngle 

#will go as high as possible to keep spoon level and get to max height
# increment is amount t1 angle will drive upwards per clock
# MAKE SURE TO IMPLEMENT TOLERANCE WHEN USING 
# :( hard-coded parameters im too tired, maybe fix later

# def driveYt1Solve(t1, t2, t3, l1, l2, increment):
#return 
# 0 = Max Height
# 1 = Keep going
# def driveYt1Solve(increment):
    
#     #assuming that all angles sum to zero if spoon remains straight
#     #solve for angle 2

#     if (increment + point1Angle > 90):
#         return 0
    
#     point1Rotation(increment)
#     point2Angle()


def main():
    global point1Angle
    global point2Angle
    global point3Angle
    global base1Angle
    
    timeSinceWrite = 0
    closeMouthTime = time.time()
    with mp_face_mesh.FaceMesh(
            max_num_faces = 1,
            refine_landmarks = True,
            min_detection_confidence = 0.5,
            min_tracking_confidence = 0.5
        ) as face_mesh:

        test = 0

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
                        lip = 1
                        print("open")
                    else:
                        print("closed")
                        lip = 0

                    x = face_landmarks.landmark[0].x
                    y = face_landmarks.landmark[0].y
                    z = face_landmarks.landmark[0].z

                    test = point1Angle

                    if(lip == 1):
                        zyjacobian(point1Angle, point2Angle, length1, length2, y-0.5, z-0.5)
                        xcentering(x)
                        closeMouthTime = time.time()
                    if(lip == 0):
                        if(time.time() - closeMouthTime > 3):
                            point1Rotation(ANGLE1DEFAULT - point1Angle)
                            point2Rotation(ANGLE2DEFAULT - point2Angle)
                            point3Rotation(ANGLE3DEFAULT - point3Angle)
                    
                    # print(x)
                    # if ( time.time() - timeSinceWrite > 0.3):
                    #     if (x > 0 and x < 1):
                    #         print(" =============================== CHANGE ANGLE " + str(x * 180))
                    #         servo_pin1.write(x*180)
                    #         servo_pin2.write(x*180)
                    #         servo_pin3.write(x*180)
                    #         servo_pin4.write(x*180)
                    #         servo_pin5.write(x*180)
                    #         servo_pin6.write(x*180)
                    #         timeSinceWrite = time.time()

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
                    
            cv2.imshow("Automatic Human Feeder", cv2.flip(image, 1))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

#NOTE a lot of this is chad gtp because i dont rember numpy
        
def zyjacobian(theta1, theta2, L1, L2, dy, dz):

    # Calculate the Jacobian matrix
    def jacobian(theta1, theta2, L1, L2):
        J = np.zeros((2, 2))
        J[0, 0] = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
        J[0, 1] = -L2 * np.sin(theta1 + theta2)
        J[1, 0] = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
        J[1, 1] = L2 * np.cos(theta1 + theta2)
        return J
    
    delta_pos = np.array([-dz*10, dy*10])
    delta_theta = np.linalg.pinv(jacobian(theta1, theta2, L1, L2)).dot(delta_pos)  
    
    point1Rotation(delta_theta[0])
    point2Rotation(delta_theta[1])

def xcentering(x):
    if( x - 0.5 < 0.1 and x - 0.5 > -0.1):
        return 0
    elif x-0.5 > 0 :
        point1Rotation(1)
    else:
        point1Rotation(-1)

if __name__ == "__main__":
    main()