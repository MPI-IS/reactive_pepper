import threading
from ..data_exchange.data import Data


class _Face:

    def __init__(self, size, distance, position):
        self.dimension = size
        self.distance = distance
        self.position = position


def _get_one_face(face_info_array, position6d):

    global _EYE_TO_MOUTH
    
    def _get_distance_from_eye_mouth(eye, mouth, eye_to_mouth):
        alpha = abs(eye[0] - mouth[0])
        beta = abs(eye[1] - mouth[1])
        sin_alpha = math.sin(alpha / 2.0)
        sin_beta = math.sin(beta / 2.0)
        if (sin_alpha**2 + sin_beta**2) > 0:
            return math.sqrt(eye_to_mouth**2 / (sin_alpha**2 + sin_beta**2))

    sizeX = face_info_array[0][3]
    sizeY = face_info_array[0][4]
    left_eye = (face_info_array[1][3][0], face_info_array[1][3][1])
    right_eye = (face_info_array[1][4][0], face_info_array[1][4][1])
    left_mouth = (face_info_array[1][8][0], face_info_array[1][8][1])
    right_mouth = (face_info_array[1][8][2], face_info_array[1][8][3])

    d_left = _get_distance_from_eye_mouth(
        left_eye, left_mouth, _EYE_TO_MOUTH)
    d_right = _get_distance_from_eye_mouth(
        right_eye, right_mouth, _EYE_TO_MOUTH)

    if d_left > d_right:
        distance = d_left
    else:
        distance = d_right

    wzCamera, wyCamera = face_info_array[0][1], face_info_array[0][2]
    p6d = almath.Position6D(position6d)

    robotToCamera = almath.transformFromPosition6D(p6d)
    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0,
                                                                        wyCamera,
                                                                        wzCamera)

    cameraToLandmarkTranslationTransform = almath.Transform(distance, 0, 0)

    robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * \
        cameraToLandmarkTranslationTransform

    position = [robotToLandmark.r1_c4,
                robotToLandmark.r2_c4,
                robotToLandmark.r3_c4]

    return _Face(sizeX * sizeY, distance, position)



class _Data_transform:

    def __init__(self):
        self._previous_time_stamp = None

    def __call__(self,from_memory):
        
        faces_from_memory = from_memory[_KEYS[0]]
        faces = []
    
        if (faces_from_memory and len(faces_from_memory) >= 2):
            time_stamp = faces_from_memory[0]
            if time_stamp != self._previous_time_stamp:
                self._previous_time_stamp = time_stamp
                position6D = faces_from_memory[3]
                detected_faces = faces_from_memory[1]
                for face in detected_faces:
                    if (len(face) > 0):
                        try:
                            faces.append(
                                _get_one_face(face, position6D))
                        except Exception as e:
                            continue
        from_memory = faces
    

        
class Face_detection(Data):

    
    def __init__(self,transforms,
                 face_detection_frame,
                 reference_frame,
                 face_detection_id):

        self._face_detection_frame = face_detection_frame
        self._referenrence_frame = reference_frame
        self._transforms = transforms
        dt = _Data_transform()
        Data.__init__(self,data_transform=dt)
        self._face_detection_id = face_detection_id

        
    @classmethod
    def get_keys(cls):
        return [self._face_detection_id]

    
    def get(self):

        data = super(Data,self).get()
        if not data: return

        faces,time_stamp = data

        if len(faces) > 0:
	    for face in faces:
                face.position = self.transforms.to_from(face.position,
                                                        self._reference_frame,
                                                        self._face_detection_frame)

	return faces, time_stamp
