import mediapipe as mp
import cv2

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

my_drawing_specs = mp_drawing.DrawingSpec(color = (0, 255, 0), thickness = 1)

cap = cv2.VideoCapture(0)

mp_face_mesh = mp.solutions.face_mesh

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
                print(face_landmarks.landmark[0].z*100)
                # face_landmarks.landmark[0].x -> x Coord
                # face_landmarks.landmark[0].y -> y Coord
                # face_landmarks.landmark[0].z -> z Coord

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