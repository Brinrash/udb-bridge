#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2017-2020 VECTIONEER.
#

import socket,select
import time
import os
import json
import argparse
import sys
import math
import _thread
import serial
import motorcortex
from robot_control import to_radians
from robot_control.robot_command import RobotCommand
from robot_control.motion_program import MotionProgram, Waypoint
from robot_control.system_defs import InterpreterStates

class Udp2Motrorcortex(object):
    def __init__(self):
        self.UDP_PORT_IN = 9090 # UDP server port
        self.UDP_PORT_OUT = 9090
        self.UDP_SERVER_ADDRESS = '192.168.254.117'
        self.MAX_UDP_PACKET=512 # max size of incoming packet to avoid sending too much data to the micro
        self.BROADCAST_MODE=False #set to True if you want a broadcast replay instead of unicast
        self.IP = 0
        self.sendFlag = False
        self.manipulatorStatus = 0
        self.port = serial.Serial("/dev/ttyS1", baudrate=115200, timeout=1.0)
        self.status = '0'
        self.buttonStatus = False

    def loadParam(self, config_file):
        if config_file==None:
            return False
        with open(config_file) as json_file:
            data = json.load(json_file)
            self.UDP_PORT_IN = data['input_port']
            self.UDP_PORT_OUT = data['output_port']
            self.UDP_SERVER_ADDRESS = data['server_address']
            self.Z_MAX = data['height_cap_before']
            self.Z_MIN = data['height_cap']
            self.MIN_ANGLE = data['min_rotation_angle']
            self.MAX_ANGLE = data['max_rotation_angle']
            self.MIN_R = data['min_distance']
            self.MAX_R = data['max_distance']
            self.IP = data['ip']
            self.GRIPPER_LIMIT_0 = data['gripper_limit_0']
            self.GRIPPER_LIMIT_1 = data['gripper_limit_1']
            self.POS_LIMIT_0 = data['pos_limit_0']
            self.POS_LIMIT_1 = data['pos_limit_1']
        return True


    def initMotorcortex(self):
        self.parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()

        self.req, self.sub = motorcortex.connect(
            'ws://127.0.0.1:5558:5557',
            self.motorcortex_types,
            self.parameter_tree,
            timeout_ms=1000,
            login="root",
            password="vectioneer"
        )

        self.subscription = self.sub.subscribe(
            [
                'root/Control/actuatorControlLoops/actuatorControlLoop01/actualMotorPositionIn',
                'root/Control/actuatorControlLoops/actuatorControlLoop02/actualMotorPositionIn',
                'root/Control/actuatorControlLoops/actuatorControlLoop03/actualMotorPositionIn',
                'root/Control/actuatorControlLoops/actuatorControlLoop04/actualMotorPositionIn',
                'root/Control/actuatorControlLoops/actuatorControlLoop05/actualMotorPositionIn',
                'root/Motorcontroller/axis6/motorSetpointOut',
                'root/Motorcontroller/axis1/temperatureIn',
                'root/Motorcontroller/axis2/temperatureIn',
                'root/Motorcontroller/axis3/temperatureIn',
                'root/Motorcontroller/axis4/temperatureIn',
                'root/Motorcontroller/axis5/temperatureIn',
                'root/Motorcontroller/axis6/temperatureIn',
                'root/Motorcontroller/axis1/motorLoadOut',
                'root/Motorcontroller/axis2/motorLoadOut',
                'root/Motorcontroller/axis3/motorLoadOut',
                'root/Motorcontroller/axis4/motorLoadOut',
                'root/Motorcontroller/axis5/motorLoadOut',
                'root/Motorcontroller/axis6/motorLoadOut',
                'root/MotionInterpreter/actualStateOut'
            ],
            'group1',
            5
        )

        self.subscription.get()

        self.robot = RobotCommand(self.req, self.motorcortex_types)
        self.move_robot = MoveRobot(self.robot)

        self.robot.reset()
        self.req.setParameter("root/Logic/stateCommand", 1).get()  # DISENGAGED
        time.sleep(1)
        self.req.setParameter("root/Logic/stateCommand", 2).get()  # ENGAGED
        time.sleep(1)

       # self.move_robot.relaxManipulator(False)
       # self.move_robot.setGripperForce(self.GRIPPER_LIMIT_0, self.GRIPPER_LIMIT_1)
       # self.move_robot.setGripperLimits(self.POS_LIMIT_0, self.POS_LIMIT_1)
        # self.move_robot.relaxManipulator(False)

        # self.move_robot.setGripperForce(self.GRIPPER_LIMIT_0, self.GRIPPER_LIMIT_1)

        # self.move_robot.setGripperLimits(self.POS_LIMIT_0, self.POS_LIMIT_1)

        # self.messageHandler("2")

    def openUpd(self):
        self.udp_client_address=(self.UDP_SERVER_ADDRESS, self.UDP_PORT_OUT) #where to forward packets coming from seriaal port
        self.udp_server_address = ('',self.UDP_PORT_IN) #udp server
        self.udp_broadcast=('<broadcast>',self.UDP_PORT_OUT) #broadcast address

        self.udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.udp_socket.bind(self.udp_server_address)
        if self.BROADCAST_MODE:
            self.udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

    def mainLoop(self):
        _thread.start_new_thread( self.printParams, ("Thread-1", 0.2, ))
        _thread.start_new_thread( self.readStatus, ("Thread-2", 0.05, ))

        while True:
            (rlist, wlist, xlist) = select.select([self.udp_socket], [], [])
            if self.udp_socket in rlist:
                    udp_data,udp_client = self.udp_socket.recvfrom(self.MAX_UDP_PACKET)
                    self.messageHandler(udp_data.decode("utf-8"))
                    time.sleep(0.001)

    def sendMessage(self, msg):
        if self.BROADCAST_MODE:
                self.udp_socket.sendto(msg.encode(),udp_broadcast)
        else:
                self.udp_socket.sendto(msg.encode(),(self.UDP_SERVER_ADDRESS, self.UDP_PORT_OUT)) 

    def messageHandler(self, msg):
        if not msg==None:
            print(msg)
            if msg[0]=='g' and not self.status=='2':
                self.move_robot.relaxManipulator(False)

                list_ = msg.split(":")

                r = float(list_[2])/1000.0

                angle = (float(list_[1])-180.0)/180.0*math.pi

                gripper_angle = float(list_[3])

                z = self.Z_MAX
                if bool(int(list_[4])):
                    z = self.Z_MIN
                else:
                    z = self.Z_MAX

                if r*1000>self.MIN_R and r*1000<self.MAX_R and float(list_[1])>self.MIN_ANGLE and float(list_[1])<self.MAX_ANGLE:
                    self.move_robot.moveLinearTo([r*math.cos(angle), r*math.sin(angle), z/1000.0], gripper_angle)
                    self.move_robot.closeGripper(bool(int(list_[5][0])))
            if msg[0]=='1' and not self.status=='2':
                self.move_robot.relaxManipulator(False)
                self.move_robot.moveJointTo([0, 0, 90.0, 90.0, 0.0])
                self.move_robot.closeGripper(False)
            if msg[0]=='2' and not self.status=='2':
                self.move_robot.relaxManipulator(False)
                r = 0.200
                angle = (45.0)/180.0*math.pi
                gripper_angle = 0
                z = self.Z_MIN
                self.move_robot.moveLinearTo([r*math.cos(angle), r*math.sin(angle), z/1000.0], gripper_angle)
                self.move_robot.closeGripper(True)
            if msg[0]=='3' and not self.status=='2':
                self.move_robot.relaxManipulator(True)
            if msg[0]=='r':
                self.sendFlag = True
            if msg[0]=='s':
                self.sendFlag = False

    def readStatus(self, threadName, delay):
        for i in range(25):
            self.port.readline()
        while True:
            result = ""
            for i in range(2):
                result_ = self.port.readline()
                if not result_==None  and len(result_)>0:
                    result = result_

            if len(result)==11:
                if not self.buttonStatus:
                    self.robot.stop()
                self.buttonStatus = True
                #self.robot.reset()
            if len(result)==9:
                if self.buttonStatus:
                    self.move_robot.moveJointTo([0, 0, 90.0, 90.0, 0.0])
                self.buttonStatus = False

            time.sleep(delay)

    def printParams(self, threadName, delay):
        while True:
            params = self.subscription.read()

            if self.buttonStatus == True:
                self.status = '2'
            else:
                if params[18].value[0]==0:
                    self.status = '0'
                else:
                    self.status = '1'

            if self.sendFlag:
                angle_str = "M:"+str(self.IP)+":" + self.status
                temps_str = "T:"+str(self.IP)+":" + self.status
                loads_str = "L:"+str(self.IP)+":" + self.status
                for i in range(6):
                    angle_str += ":" + str(int(params[i].value[0]))
                    temps_str += ":" + str(int(params[6+i].value[0]))
                    loads_str += ":" + str(int(params[12+i].value[0]))
                angle_str += "#\n"
                temps_str += "#\n"
                loads_str += "#\n"

                self.sendMessage(angle_str+temps_str+loads_str)
                # print(angle_str+temps_str+loads_str)

            time.sleep(delay)

    def closeConnection(self):
        upd_motorcortex.req.close()
        upd_motorcortex.sub.close()

class MoveRobot(object):
    def __init__(self, robot):
        self.__robot = robot
        self.__motorcortex_types = robot.getTypesRef()
        self.__req = robot.getReqRef()
        self.__relaxFlag = False

    def closeGripper(self, flag):
        req = self.__req
        if flag:
            req.setParameter('root/Motorcontroller/axis6/motorSetpointIn', 0).get()
        else:
            req.setParameter('root/Motorcontroller/axis6/motorSetpointIn', 700).get()

        motion_program = MotionProgram(self.__req, self.__motorcortex_types)
        motion_program.addWait(1)
        self.sendProgram(motion_program)
        time.sleep(1)

    def startStopMoving(self, stop_flag):
        # req = self.__req
        if stop_flag:
            self.__robot.stop()
        else:
            self.__robot.engage()
            self.moveJointTo([0, 0, 90.0, 90.0, 0.0])
        time.sleep(1)

    def relaxManipulator(self, flag):
        if not flag==self.__relaxFlag:
            req = self.__req
            if flag:
                self.__robot.stop()
                self.__robot.off()
                req.setParameter('root/Motorcontroller/releaseBrakes', 1).get()
            else:
                req.setParameter('root/Motorcontroller/releaseBrakes', 0).get()
                self.__robot.engage()
              #  self.moveJointTo([0, 0, 90.0, 90.0, 0.0])
            self.__relaxFlag = flag
            time.sleep(1)

    def setGripperForce(self, min_, max_):
        req = self.__req
        req.setParameter('root/Motorcontroller/axis6/loadLimits', [min_, max_]).get()


    def setGripperLimits(self, min_, max_):
        req = self.__req
        req.setParameter('root/Motorcontroller/axis6/posLimits', [min_, max_]).get()


    def moveLinearTo(self, cart_pos_m, rot_z_deg=0):
        # Building example program
        motion_program = MotionProgram(self.__req, self.__motorcortex_types)
        cart_coord = cart_pos_m + to_radians([rot_z_deg + 180.0, 0.0, 180.0])  # orientation is fixed, applying rot_z
        motion_program.addMoveL([Waypoint(cart_coord)], 0.3, 0.3)

        return self.sendProgram(motion_program)

    def moveJointTo(self, jnt_pos_deg):
        motion_program = MotionProgram(self.__req, self.__motorcortex_types)
        jpos_1 = Waypoint(to_radians(jnt_pos_deg))
        motion_program.addMoveJ([jpos_1], 0.5, 0.5)
        return self.sendProgram(motion_program)

    def sendProgram(self, motion_program):
        program_sent = motion_program.send("example1").get()
        print(program_sent.status)

        state = self.__robot.play()

        if state == InterpreterStates.PROGRAM_RUN_S.value:
            print("Playing program")

        elif state == InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print("Can not play program, Robot is not at start")
            print("Moving to start")

            if self.__robot.moveToStart(20):
                print("Move to start completed")

                state = self.__robot.play()
                print("State after moveToStart:", state)

                if state != InterpreterStates.PROGRAM_RUN_S.value:
                    return False
            else:
                print("Failed to move to start")
                return False

        else:
            print("Interpreter state:", state)
            return False

        while self.__robot.getState() != InterpreterStates.PROGRAM_STOP_S.value:
            time.sleep(0.1)

        return True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", type=str,
                    help="config file")
    args = parser.parse_args()

    upd_motorcortex = Udp2Motrorcortex()

    if not upd_motorcortex.loadParam(args.config):
        print("Can't read config file")
        return

    upd_motorcortex.openUpd()

    upd_motorcortex.initMotorcortex()

    upd_motorcortex.mainLoop()

if __name__ == '__main__':
    main()
