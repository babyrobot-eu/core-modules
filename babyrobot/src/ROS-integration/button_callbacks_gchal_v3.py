#000000
from naoqi import ALProxy
import vision_definitions as vd
import almath
import time
from time import gmtime, strftime
import numpy as np
from datetime import datetime



# ttsProxy = ALProxy("ALTextToSpeech", IP, PORT)
# joint names
headYawName = "HeadYaw"
headPitchName = "HeadPitch"
RShoulderRollName = "RShoulderRoll"
RShoulderPitchName = "RShoulderPitch"
RElbowRollName = "RElbowRoll"
RElbowYawName = "RElbowYaw"
RWristYawName = "RWristYaw"
RHandName = "RHand"

audio_path = "/home/nao/audio/"

shoulder_pitch_max = -30


def nao_stiffness(ip, port, v):
    motion_proxy = ALProxy("ALMotion", ip, port)
    motion_proxy.setStiffnesses(["Body"], v)


# postures: Sit, Stand, Crouch
def nao_posture(ip, port, posture, speed=0.5):
    posture_proxy = ALProxy("ALRobotPosture", ip, port)
    posture_proxy.goToPosture(posture, speed)  # blocking call


# walk
def nao_walk(ip, port, dist):
    motion_proxy = ALProxy("ALMotion", ip, port)
    speed = 0.2
    motion_proxy.setAngles("RHand", 0.0, speed)
    motion_proxy.moveTo(dist, 0.0, 0.0)  # blocking call, moveTo(x, y, theta)


# turn
def nao_turn(ip, port, theta):
    motion_proxy = ALProxy("ALMotion", ip, port)
    motion_proxy.moveTo(0.0, 0.0, theta)


# Bending
def nao_bend(ip, port, target):
    postureProxy = ALProxy("ALRobotPosture", ip, port)
    motion_proxy = ALProxy("ALMotion", ip, port)
    # Send NAO to Pose Init
    # postureProxy.goToPosture("StandInit", 0.5)

    motion_proxy.wbEnable(True)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motion_proxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motion_proxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Example showing how to set orientation target for LArm tracking.
    effectorName = "RArm"

    motion_proxy.wbEnableEffectorControl(effectorName, True)
    time.sleep(2.0)
    if target == 1:
        target_coordinate = [0.2, 0.0, 0.6]  # bend slightly to the front
    elif target == 2:
        target_coordinate = [0.2, 0.2, 0.0]  # bend down
    # target_coordinate = [0.1, 0.0, 0.3]  # bend slightly to the front

    motion_proxy.wbSetEffectorControl(effectorName, target_coordinate)

    time.sleep(2.0)
    motion_proxy.wbEnable(False)


def open_close(ip,port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    fractionMaxSpeed = 1.0
    for i in range(4):
        motion_proxy.setAngles(RHandName, 0.0, fractionMaxSpeed)  # non-blocking call
        time.sleep(0.2)
        motion_proxy.setAngles(RHandName, 1.0, fractionMaxSpeed)  # non-blocking call
        time.sleep(0.2)


def point(ip, port, speed):
    motion_proxy = ALProxy("ALMotion", ip, port)
    #point
    timeLists  = 0.5
    isAbsolute = True
    motion_proxy.angleInterpolation(RWristYawName, -57 * almath.TO_RAD, timeLists, isAbsolute)  # non-blocking call
    time.sleep(2)
    motion_proxy.angleInterpolation(RWristYawName, 0 * almath.TO_RAD, timeLists, isAbsolute)  # non-blocking call
    # time.sleep(1)


def nao_return_hand(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    speed = 0.1
    motion_proxy.setAngles(RShoulderRollName, -80 * almath.TO_RAD, speed)
    time.sleep(1)
    motion_proxy.setAngles(RShoulderPitchName, 100 * almath.TO_RAD, speed)
    time.sleep(1)
    motion_proxy.setAngles(RShoulderRollName, -16 * almath.TO_RAD, speed)
    time.sleep(1)
    motion_proxy.setAngles(RElbowYawName, 115.0 * almath.TO_RAD, speed)
    motion_proxy.setAngles(RWristYawName, -28.0 * almath.TO_RAD, 0.5)
    # time.sleep(0.2)
    # nao_posture(ip,port,"Stand")


def nao_retry_reachout(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    speed = 0.2
    motion_proxy.setAngles(RShoulderRollName, 40 * almath.TO_RAD, speed)
    time.sleep(1)
    motion_proxy.setAngles(RElbowRollName, -60 * almath.TO_RAD, speed)


# Grasp
def nao_grasp(ip, port, speed=0.5):
    motion_proxy = ALProxy("ALMotion", ip, port)
    motion_proxy.setAngles(RHandName, 0.0, speed)  # non-blocking call
    time.sleep(0.2)


# Open Grasp
def nao_open_hand(ip, port, speed=0.8):
    motion_proxy = ALProxy("ALMotion", ip, port)
    motion_proxy.setAngles(RHandName, 1.0, speed)  # non-blocking call
    time.sleep(0.2)


def unix_time_millis(dt):
    epoch = datetime.utcfromtimestamp(0)
    return (dt - epoch).total_seconds() * 1000.0


def init_process():
    timeset = []
    start_time = unix_time_millis(datetime.now())
    timeset.append(start_time)

    return timeset


def end_process():
    timeset = []
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset
################################################################################################
# Action 1


def action1(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    # raise arm
    speed = 0.3
    timeLists  = 1.0
    isAbsolute = True
    motion_proxy.angleInterpolation(RShoulderRollName, 1.8 * almath.TO_RAD, 1, isAbsolute)
    # motion_proxy.setAngles(RShoulderRollName, 1.8 * almath.TO_RAD, speed)
    #time.sleep(0.05)
    motion_proxy.angleInterpolation(RShoulderPitchName, -25 * almath.TO_RAD, timeLists, isAbsolute)
    #time.sleep(0.05)
    # motion_proxy.angleInterpolation(RShoulderRollName, -2 * almath.TO_RAD, timeLists, isAbsolute)
    # time.sleep(0.05)
    speed = 0.1
    motion_proxy.angleInterpolation(RElbowRollName, 4 * almath.TO_RAD, timeLists, isAbsolute)
    # motion_proxy.angleInterpolation(RShoulderRollName, 9.8 * almath.TO_RAD, timeLists, isAbsolute)
    # motion_proxy.angleInterpolation(RShoulderPitchName, shoulder_pitch_max * almath.TO_RAD, timeLists, isAbsolute)
    motion_proxy.angleInterpolation(RElbowYawName, 62.8 * almath.TO_RAD, 0.5, isAbsolute)
    # motion_proxy.angleInterpolation(RWristYawName, -74.0 * almath.TO_RAD, timeLists, isAbsolute)
    #time.sleep(0.5)
    speed = 1.0
    motion_proxy.angleInterpolation(RHandName, 0.8, timeLists, isAbsolute)
    # point
    point(ip, port, speed)


def nao_action1(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action1(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 2
def action2(ip, port):
    #nao_lower_arm(ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    action1(ip, port)
    # for i in range(2):
    #     open_close(ip, port)
    #     speed = 1.0
    #     point(ip, port, speed)


def nao_action2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action2(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 3
def action3(ip, port):
    nao_return_hand(ip, port)
    action2(ip, port)


def nao_action3(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action3(ip, port)
    time.sleep(3)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 4
# Exchange look from standing pose
def action4(ip, port, yaw, pitch):
    motion_proxy = ALProxy("ALMotion", ip, port)
    for i in range(2):
        speed = 0.2
        motion_proxy.setAngles("HeadYaw", yaw * almath.TO_RAD, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", pitch * almath.TO_RAD, speed)  # non-blocking call
        time.sleep(2)
        motion_proxy.setAngles("HeadYaw", 0, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", 0, speed)  # non-blocking call


def nao_action4(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action4(ip, port, yaw=40, pitch=-20)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


#############################################################################################################
# COMPOSITE ACTIONS
# Action 5
def action5(ip, port):
    action4(ip, port, yaw=40, pitch=-20)
    nao_return_hand(ip, port)
    action1(ip, port)


def nao_action5(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action5(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 6
def action6(ip, port):
    action3(ip, port)
    action4(ip, port, yaw=40, pitch=-20)


def nao_action6(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action6(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 7
def action7(ip, port, yaw, pitch):
    motion_proxy = ALProxy("ALMotion", ip, port)
    nao_return_hand(ip, port)
    action1(ip, port)
    for i in range(2):
        speed = 0.2
        motion_proxy.setAngles("HeadYaw", yaw * almath.TO_RAD, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", pitch * almath.TO_RAD, speed)  # non-blocking call
        time.sleep(1)
        open_close(ip, port)
        motion_proxy.setAngles("HeadYaw", 0, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", 0, speed)  # non-blocking call
        point(ip, port, speed)


def nao_action7(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action7(ip, port, yaw=40, pitch=-20)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 8
def action8(ip, port):
    nao_posture(ip, port, "StandInit")
    nao_bend(ip, port, 1)
    action7(ip, port, yaw=40, pitch=-20)


def nao_action8(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    #nao_action8(ip, port)
    # Wait for it to finish
    nao_posture(ip, port, "StandInit")
    nao_bend(ip, port, 1)
    action7(ip, port, yaw=40, pitch=-20)
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


def nao_close_open_hand(ip, port, n, speed=0.8):
    motion_proxy = ALProxy("ALMotion", ip, port)
    delay = 0.2
    for i in range(n):
        motion_proxy.setAngles(RHandName, 0.0, speed)  # non-blocking call
        time.sleep(delay)
        motion_proxy.setAngles(RHandName, 1.0, speed)  # non-blocking call
        time.sleep(delay)


def nao_turn_wrist(ip, port, val, speed=0.8):
    motion_proxy = ALProxy("ALMotion", ip, port)
    motion_proxy.setAngles(RWristYawName, val * almath.TO_RAD, speed)  # non-blocking call


def nao_head_hand(ip, port, n_head, n_hand):
    motion_proxy = ALProxy("ALMotion", ip, port)
    head_speed = 0.2
    for i in range(n_head):
        motion_proxy.setAngles("HeadYaw", 40 * almath.TO_RAD, head_speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", -20 * almath.TO_RAD, head_speed)  # non-blocking call
        time.sleep(1)
        nao_close_open_hand(ip, port, n_hand)
        time.sleep(1)
        motion_proxy.setAngles("HeadYaw", 0, head_speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", 0, head_speed)  # non-blocking call
        time.sleep(1)


def nao_lower_arm(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    hand_speed = 0.2
    motion_proxy.setAngles(RShoulderPitchName, 58 * almath.TO_RAD, hand_speed)  # non-blocking call


def nao_lift_arm(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    hand_speed = 0.2
    motion_proxy.setAngles(RShoulderPitchName, shoulder_pitch_max * almath.TO_RAD, hand_speed)


def nao_lift_arm_point(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    hand_speed = 0.2
    motion_proxy.setAngles(RShoulderPitchName, -12 * almath.TO_RAD, hand_speed)
    motion_proxy.setAngles(RElbowRollName, 21 * almath.TO_RAD, hand_speed)

    time.sleep(3)

    motion_proxy.setAngles(RElbowRollName, 5.3 * almath.TO_RAD, hand_speed)
    motion_proxy.setAngles(RShoulderPitchName, shoulder_pitch_max * almath.TO_RAD, hand_speed)


def nao_say(ip, port, file):
    audio_proxy = ALProxy("ALAudioPlayer", ip, port)
    audio_proxy.playFile(audio_path + file + ".wav")


# ACTIONS SECOND SET

# Action 1


def action1_s2(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    # raise arm
    speed = 0.3
    motion_proxy.setAngles(RShoulderRollName, -5 * almath.TO_RAD, speed)
    time.sleep(0.05)
    motion_proxy.setAngles(RShoulderPitchName, 40 * almath.TO_RAD, speed)
    time.sleep(0.05)
    speed = 0.1
    motion_proxy.setAngles(RElbowRollName, 5.3 * almath.TO_RAD, speed)
    motion_proxy.setAngles(RWristYawName, -74.0 * almath.TO_RAD, 0.5)
    time.sleep(0.5)
    speed = 1.0
    motion_proxy.setAngles(RHandName, 0.8, speed)
    # point
    point(ip, port, speed)


def nao_action1_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    action1_s2(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 2
def action2_s2(ip, port):
    #nao_lower_arm(ip, port)
    action1_s2(ip, port)
    for i in range(2):
        open_close(ip, port)
        speed = 1.0
        point(ip, port, speed)


def nao_action2_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    action2_s2(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 3
def action3_s2(ip, port):
    nao_return_hand_s2(ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    action2_s2(ip, port)


def nao_action3_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action3_s2(ip, port)
    time.sleep(3)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 4
# Exchange look from standing pose
def action4_s2(ip, port, yaw, pitch):
    motion_proxy = ALProxy("ALMotion", ip, port)
    for i in range(2):
        speed = 0.2
        motion_proxy.setAngles("HeadYaw", yaw * almath.TO_RAD, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", pitch * almath.TO_RAD, speed)  # non-blocking call
        time.sleep(2)
        motion_proxy.setAngles("HeadYaw", 0, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", 0, speed)  # non-blocking call


def nao_action4_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action4_s2(ip, port, yaw=-40, pitch=-20)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


#############################################################################################################
# COMPOSITE ACTIONS
# Action 5
def action5_s2(ip, port):
    action4(ip, port, yaw=-40, pitch=-20)
    nao_return_hand_s2(ip, port)
    action1_s2(ip, port)


def nao_action5_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action5_s2(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 6
def action6_s2(ip, port):
    action3_s2(ip, port)
    action4_s2(ip, port, yaw=-40, pitch=-20)


def nao_action6_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    action6_s2(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 7
def action7_s2(ip, port, yaw, pitch):
    motion_proxy = ALProxy("ALMotion", ip, port)
    nao_return_hand_s2(ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    action1_s2(ip, port)
    for i in range(2):
        speed = 0.2
        motion_proxy.setAngles("HeadYaw", yaw * almath.TO_RAD, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", pitch * almath.TO_RAD, speed)  # non-blocking call
        time.sleep(1)
        open_close(ip, port)
        motion_proxy.setAngles("HeadYaw", 0, speed)  # non-blocking call
        motion_proxy.setAngles("HeadPitch", 0, speed)  # non-blocking call
        point(ip, port, speed)


def nao_action7_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    action7_s2(ip, port, yaw=-40, pitch=-20)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


# Action 8
def action8_s2(ip, port):
    nao_posture(ip, port, "StandInit", speed=0.5)
    nao_bend(ip, port, 2)
    action7_s2(ip, port, yaw=-40, pitch=-20)


def nao_action8_s2(ip, port):
    timeset = []
    motion_proxy = ALProxy("ALMotion", ip, port)
    nao_posture(ip, port, "StandInit", speed=0.5)
    nao_bend(ip, port, 2)
    action7_s2(ip, port, yaw=-40, pitch=-20)
   # nao_action8_s2(ip, port)
    # Wait for it to finish
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(3)
    end_time = unix_time_millis(datetime.now())
    timeset.append(end_time)
    return timeset


def nao_return_hand_s2(ip, port):
    motion_proxy = ALProxy("ALMotion", ip, port)
    speed = 0.1
    motion_proxy.setAngles(RShoulderRollName, 80 * almath.TO_RAD, speed)
