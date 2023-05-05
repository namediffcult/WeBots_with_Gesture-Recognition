'''
    By pipe_dream
    just a simple homework
'''
import cv2
import mediapipe as mp
import math
from controller import Robot

# create the Robot instance.
robot = Robot()

timestep = int(robot.getBasicTimeStep())

right_motor = robot.getDevice('motor right')
left_motor = robot.getDevice('motor left')

led = robot.getDevice('led')
led.set(0)

right_motor.setPosition(float('inf'))
left_motor.setPosition(float('inf'))

NowspeedLeft = 3.0
NowspeedRight = 3.0

left_motor.setVelocity(NowspeedLeft)
right_motor.setVelocity(NowspeedRight)

ds_front = robot.getDevice('DSfront')
ds_front.enable(timestep)

ds_left = robot.getDevice('DSudL')
ds_left.enable(timestep)

ds_right = robot.getDevice('DSudR')
ds_right.enable(timestep)

camera_left = robot.getDevice('CameraLeft')
camera_left.enable(timestep)

camera_right = robot.getDevice('CameraRight')
camera_right.enable(timestep)

# --------- CV_package --------- st#
# base on finger angle
def cal_vector(ve1, ve2):
    v1_x, v1_y, v2_x, v2_y = ve1[0], ve1[1], ve2[0], ve2[1]
    try:
        angle_ = math.degrees(math.acos(
            (v1_x * v2_x + v1_y * v2_y) / (((v1_x ** 2 + v1_y ** 2) ** 0.5) * ((v2_x ** 2 + v2_y ** 2) ** 0.5))))
    except:
        angle_ = 65535.
    if angle_ > 180:
        angle_ = 65535.
    return angle_


def hand_angle(hand_):
    ag_thumb = cal_vector(
        ((int(hand_[0][0]) - int(hand_[2][0])), (int(hand_[0][1]) - int(hand_[2][1]))),
        ((int(hand_[3][0]) - int(hand_[4][0])), (int(hand_[3][1]) - int(hand_[4][1])))
    )
    ag_index = cal_vector(
        ((int(hand_[0][0]) - int(hand_[6][0])), (int(hand_[0][1]) - int(hand_[6][1]))),
        ((int(hand_[7][0]) - int(hand_[8][0])), (int(hand_[7][1]) - int(hand_[8][1])))
    )
    ag_middle = cal_vector(
        ((int(hand_[0][0]) - int(hand_[10][0])), (int(hand_[0][1]) - int(hand_[10][1]))),
        ((int(hand_[11][0]) - int(hand_[12][0])), (int(hand_[11][1]) - int(hand_[12][1])))
    )
    ag_ring = cal_vector(
        ((int(hand_[0][0]) - int(hand_[14][0])), (int(hand_[0][1]) - int(hand_[14][1]))),
        ((int(hand_[15][0]) - int(hand_[16][0])), (int(hand_[15][1]) - int(hand_[16][1])))
    )
    ag_pink = cal_vector(
        ((int(hand_[0][0]) - int(hand_[18][0])), (int(hand_[0][1]) - int(hand_[18][1]))),
        ((int(hand_[19][0]) - int(hand_[20][0])), (int(hand_[19][1]) - int(hand_[20][1])))
    )
    dec = int(hand_[8][0]) - int(hand_[5][0])
    return [
        ag_thumb,
        ag_index,
        ag_middle,
        ag_ring,
        ag_pink,
        dec
    ]


def _jud_Stop(_jud):
    return (_jud[0] == False) & (_jud[1] == False) & (_jud[2] == False) & (_jud[3] == False) & (_jud[4] == False)


def _jud_DriveMyself(_jud):
    return (_jud[0] == True) & (_jud[1] == False) & (_jud[2] == True) & (_jud[3] == True) & (_jud[4] == True)


def _jud_DriveAuto(_jud):
    return (_jud[0] == True) & (_jud[1] == True) & (_jud[2] == True) & (_jud[3] == True) & (_jud[4] == True)


def jud(ag_list):
    thr_ag_thumb = 53.
    thr_ag = 65.
    _jud = [(ag_list[0] > thr_ag_thumb), (ag_list[1] > thr_ag), (ag_list[2] > thr_ag), (ag_list[3] > thr_ag),
            (ag_list[4] > thr_ag)]
    if 65535. not in ag_list:
        print(_jud, ag_list[5])
        if _jud_Stop(_jud):
            return ['stop']
        elif _jud_DriveMyself(_jud):
            dec = min(max(-30, ag_list[5]), 30) / 10
            LeftWheelSpeed = 3 + dec
            RightWheelSpeed = 3 - dec
            return ['drive myself', LeftWheelSpeed, RightWheelSpeed]

        elif _jud_DriveAuto(_jud):
            return ['auto mode']
        else:
            return ['uncaught']

    return ['uncaught']


mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.75)
cap = cv2.VideoCapture(0)


# --------- CV_package --------- ed#

'''
    mode:
        0: Stop
        1: Auto Drive
        2: Drive myself
'''
mode = 1

while robot.step(timestep) != -1:

    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.flip(frame, 1)  # rev
    results = hands.process(frame)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    res_control = []
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            hand_local = []
            for i in range(21):
                x = hand_landmarks.landmark[i].x * frame.shape[1]
                y = hand_landmarks.landmark[i].y * frame.shape[0]
                hand_local.append((x, y))
            if hand_local != []:
                ag_list = hand_angle(hand_local)
                res_control = jud(ag_list)
                if res_control[0] == 'stop':
                    print("mode change to stop")
                    mode = 0

                elif res_control[0] == 'drive myself':
                    print("mode change to drive myself")
                    mode = 2

                else:
                    print("mode change to drive auto")
                    mode = 1
    # fix_bug
    if res_control == []:
        mode = 1

    if mode == 0:
        NowspeedLeft = max(NowspeedLeft - 0.5, 0)
        NowspeedRight = max(NowspeedRight - 0.5, 0)
    elif mode == 2:
        NowspeedLeft = res_control[1]
        NowspeedRight = res_control[2]
    else:
        ds_right_value = ds_right.getValue()
        ds_left_value = ds_left.getValue()
        if (ds_right_value - ds_left_value) > 5:
            NowspeedLeft = 2.5
            NowspeedRight = 0.8
        elif (ds_right_value - ds_left_value) < -5:
            NowspeedLeft = 0.8
            NowspeedRight = 2.5
        else:
            NowspeedLeft = 4.0
            NowspeedRight = 4.0

    left_motor.setVelocity(NowspeedLeft)
    right_motor.setVelocity(NowspeedRight)

    cv2.imshow('MediaPipe Hands', frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

    pass


cap.release()
# Enter here exit cleanup code.
