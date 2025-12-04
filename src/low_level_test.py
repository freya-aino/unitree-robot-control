import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import go2_constants
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

class Custom:
    def __init__(self):
        self.Kp = 60.0          # position gain for PID control
        self.Kd = 5.0           # velocity gain for PID control
        #self.time_consume = 0
        #self.rate_count = 0
        #self.sin_count = 0
        #self.motiontime = 0
        self.dt = 0.002  # 0.001~0.01   # cycle time of lowcmd write (2ms)

        # low-level command and state
        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None  

        # define target positions
        
        
        # # define durations for each motion phase (number of cycles)
        # self.duration_1 = 500   # 500*0.002=1s
        # self.duration_2 = 500   # 500*0.002=1s
        # self.duration_3 = 1000  # 1000*0.002=2s
        # self.duration_4 = 900   # 900*0.002=1.8s

        # motion progress percentages
        # self.percent_1 = 0
        # self.percent_2 = 0
        # self.percent_3 = 0
        # self.percent_4 = 0

        # define start position
        self.startPos = [0.0] * 12

        # define target positions of current motion
        self.targetPos = [0.0] * 12

        # define durations of current motion
        self.duration = 0

        # define progress of current motion
        self.percent = 0.0

        # Boolean is_moving
        self.is_moving = False

        # first run flag
        self.firstRun = True
        # self.done = False

        # thread handling
        self.lowCmdWriteThreadPtr = None

        self.crc = CRC()

    # Public methods
    def Init(self):
        self.InitLowCmd()

        # create publisher
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        # initialize Sport and MotionSwitcher clients
        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        # check if sport mode is active. if yes, lay down and realse mode
        status, result = self.msc.CheckMode()
        while result['name']:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)


    # Thread start ensures exact timing for calling LowCmdWrite
    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=0.002, target=self.LowCmdWrite, name="writebasiccmd"
        )
        self.lowCmdWriteThreadPtr.Start()


    def move_to_target(self, target_pos, duration_steps):

        time.sleep(0.005)

        # define current position as start position
        for i in range(12):
            self.startPos[i] = self.low_cmd.motor_cmd[i].q

        # set target position and duration
        self.targetPos = target_pos
        self.duration = duration_steps
        # self.percent = 0.0 # Reset progress
        
        # start moving
        self.is_moving = True

    
        while self.is_moving:
            time.sleep(0.002)


    # Private methods
    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q = go2_constants.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2_constants.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        # print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        # print("IMU state: ", msg.imu_state)
        # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)


    def LowCmdWrite(self):  # called every 2ms
        
        # # when it's the firat run: check position and safe as start position
        # if self.firstRun:
        #     for i in range(12):
        #         self.startPos[i] = self.low_state.motor_state[i].q
        #     self.firstRun = False


        if self.firstRun:
            if self.low_state is not None: # wait until we have received the first low_state message
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = self.low_state.motor_state[i].q # set start position as current position
                    
                    # activate position control with zero velocity and torque
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].tau = 0

                self.firstRun = False   # first run is done
            return  # cannot proceed until first run is done
        
        
        if self.is_moving:
            # calculate progress
            self.percent += 1.0 / self.duration # percent is increased every 2ms by 1/duration (e.g., duration=500 -> 1/1000=0.001)
            self.percent = min(self.percent, 1.0)

            # Move Logic
            for i in range(12):
                # set target position by interpolation
                self.low_cmd.motor_cmd[i].q = (1 - self.percent) * self.startPos[i] + self.percent * self.targetPos[i]  # interpolation: (1-p)*start + p*target
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

            # check if motion is done
            if self.percent >= 1.0:
                self.is_moving = False # stopps moving
        
        else:   # hold logic (q is not changed)
            for i in range(12):
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0


        # # phase 1: move to target position 1
        # self.percent_1 += 1.0 / self.duration_1 # count percent_1 as long as less than 1
        # self.percent_1 = min(self.percent_1, 1)
        # if self.percent_1 < 1:  
        #     for i in range(12): # set target position by interpolation
        #         self.low_cmd.motor_cmd[i].q = (1 - self.percent_1) * self.startPos[i] + self.percent_1 * self._targetPos_1[i]
        #         self.low_cmd.motor_cmd[i].dq = 0
        #         self.low_cmd.motor_cmd[i].kp = self.Kp
        #         self.low_cmd.motor_cmd[i].kd = self.Kd
        #         self.low_cmd.motor_cmd[i].tau = 0

        # # phase 2: move to target position 2
        # if (self.percent_1 == 1) and (self.percent_2 <= 1): # true if phase 1 is done and percent_2 less than 1
        #     self.percent_2 += 1.0 / self.duration_2
        #     self.percent_2 = min(self.percent_2, 1)
        #     for i in range(12): # set target position by interpolation
        #         self.low_cmd.motor_cmd[i].q = (1 - self.percent_2) * self._targetPos_1[i] + self.percent_2 * self._targetPos_2[i]
        #         self.low_cmd.motor_cmd[i].dq = 0
        #         self.low_cmd.motor_cmd[i].kp = self.Kp
        #         self.low_cmd.motor_cmd[i].kd = self.Kd
        #         self.low_cmd.motor_cmd[i].tau = 0

        # # phase 3: hold target position 2
        # if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 < 1):
        #     self.percent_3 += 1.0 / self.duration_3
        #     self.percent_3 = min(self.percent_3, 1)
        #     for i in range(12):
        #         self.low_cmd.motor_cmd[i].q = self._targetPos_2[i] # hold position 2
        #         self.low_cmd.motor_cmd[i].dq = 0
        #         self.low_cmd.motor_cmd[i].kp = self.Kp
        #         self.low_cmd.motor_cmd[i].kd = self.Kd
        #         self.low_cmd.motor_cmd[i].tau = 0

        # # phase 4: move to target position 3
        # if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 == 1) and (self.percent_4 <= 1):
        #     self.percent_4 += 1.0 / self.duration_4
        #     self.percent_4 = min(self.percent_4, 1)
        #     for i in range(12):
        #         self.low_cmd.motor_cmd[i].q = (1 - self.percent_4) * self._targetPos_2[i] + self.percent_4 * self._targetPos_3[i]
        #         self.low_cmd.motor_cmd[i].dq = 0
        #         self.low_cmd.motor_cmd[i].kp = self.Kp
        #         self.low_cmd.motor_cmd[i].kd = self.Kd
        #         self.low_cmd.motor_cmd[i].tau = 0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    # custom.Start()

    # while True:        
    #     if custom.percent_4 == 1.0: 
    #        time.sleep(1)
    #        print("Done!")
    #        sys.exit(-1)     
    #     time.sleep(1)

    time.sleep(1)   # wait for initialization

    # individual poses
    # Format: [FR_0, FR_1, FR_2, FL_0, FL_1, FL_2, RR_0, RR_1, RR_2, RL_0, RL_1, RL_2]

    # targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
    # targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
    # targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]
    
    # print("Moving to targetPos_1")
    # custom.move_to_target(targetPos_1, 500)    # 500*0.002=1s

    # print(" Hold current position for 2 seconds")
    # time.sleep(2)

    # print("Moving to targetPos_2")
    # custom.move_to_target(targetPos_2, 1500)   # 1500*0.002=3s
