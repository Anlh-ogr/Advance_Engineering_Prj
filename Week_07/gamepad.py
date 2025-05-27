import pygame as pg
import serial
import time

class GamepadHandler:
    def __init__(self, com_port="COM4", baudrate=115200):
        pg.init()
        pg.joystick.init()
        
        if pg.joystick.get_count() < 2:
            print("Can not find the second gamepad!")
            exit()
        self.gamepad = pg.joystick.Joystick(1)
        self.gamepad.init()
        
        print(f"Gamepad: {self.gamepad.get_name()}")
        print(f"Number of buttons: {self.gamepad.get_numbuttons()}")
        print(f"Number of axes: {self.gamepad.get_numaxes()}")
        print(f"Number of hats: {self.gamepad.get_numhats()}")
        
        self.serial = serial.Serial(com_port, baudrate, timeout=1)
        time.sleep(2)
        print(f"Connected to {com_port} at {baudrate} baud")
        
        # anh xa nut + truc
        self.button_map = {0: "btn_Y", 1: "btn_B", 2: "btn_A", 3: "btn_X", 4: "btn_L1", 5: "btn_R1", 6: "btn_L2", 7: "btn_R2"}
        self.hat_map = {(0, 1): "btn_Up", (0, -1): "btn_Down", (-1, 0): "btn_Left", (1, 0): "btn_Right"}
        self.axis_map = {0: "axis_LX", 1: "axis_LY", 2: "axis_RX", 3: "axis_RY"}
        
        # PWM state (channel 1-6, corresponding to servo 0, 1, 2, 3, 5, 4)
        self.servo_position = {1: 1500,
                               2: 1500,
                               3: 1500, 
                               4: 1500,
                               5: 2500}
        
        # Parameters
        self.pwm_step_base0 = 20
        self.pwm_step_base1 = 20
        self.pwm_step_base2 = 50
        self.pwm_step_base3 = 50
        self.pwm_step_base5 = 200
        self.pwm_min = 500
        self.pwm_max = 2500
        self.move_time = 50
        self.delay = 50
        self.line_ending = "\r\n"
        
    def get_event(self, debug=True):
        events = []
        for event in pg.event.get():
            if debug:
                print(f"Event: {event}")
                
            if event.type == pg.JOYBUTTONDOWN:
                if debug:
                    print(f"Button {event.button} pressed")
                if event.button in self.button_map:
                    events.append(self.button_map[event.button])
                    
            if event.type == pg.JOYHATMOTION:
                hat_value = self.gamepad.get_hat(event.hat)
                if debug:
                    print(f"D-pad: {hat_value}")
                if hat_value in self.hat_map:
                    events.append(self.hat_map[hat_value])
                    
            if event.type == pg.JOYAXISMOTION:
                axis_value = self.gamepad.get_axis(event.axis)
                if event.axis in [1, 3]:
                    axis_value = -axis_value
                if debug:
                    print(f"Axis: {event.axis}, Value: {axis_value:.2f}")
                if abs(axis_value) > 0.05:
                    events.append(f"{self.axis_map[event.axis]}:{axis_value:.2f}")
                    
        return events
    
    def send_servo_cmd(self, commands, full_command=False):
        command_str = ""
        if full_command:
            # Full command for all servos
            for servo_id in range(1, 33):
                pwm = self.servo_position.get(servo_id, 1500)
                if servo_id in commands:
                    pwm = max(self.pwm_min, min(self.pwm_max, commands[servo_id]))
                    if servo_id <= 5:
                        self.servo_position[servo_id] = pwm
                command_str += f"#{servo_id}P{pwm}"
        else:
            # Single command for specific servos
            for servo_id, pwm in commands.items():
                pwm = max(self.pwm_min, min(self.pwm_max, pwm))
                self.servo_position[servo_id] = pwm
                command_str += f"#{servo_id}P{pwm}"
        command_str += f"T{self.move_time}D{self.delay}{self.line_ending}"
        self.serial.write(command_str.encode())
        print(f"Sent: {command_str.strip()}")
        
        # Debut
        time.sleep(0.1)
        response = self.serial.read_all().decode()
        if response:
            print(f"Response: {response}")
        
    def run(self, status=True):
        try:
            print("Mode Control: D-pad Left/Right for servo 0, A/B/X/Y/L2/R2 for servo 1/2/3/5")
            print("Pressed Ctrl+C to exit")
            
            while status:
                events = self.get_event(debug=True)
                if not events:
                    continue
                
                servo_commands = {}
                
                for event in events:
                    print(f"Event: {event}")
                    
                    # Servo 0 (channel 1)
                    if event == "btn_Left":
                        servo_commands[1] = self.servo_position[1] - self.pwm_step_base0
                    if event == "btn_Right":
                        servo_commands[1] = self.servo_position[1] + self.pwm_step_base0
                        
                    # Servo 1 (channel 2)
                    if event == "btn_Up":
                        servo_commands[2] = self.servo_position[2] - self.pwm_step_base1
                    if event == "btn_Down":
                        servo_commands[2] = self.servo_position[2] + self.pwm_step_base1
                    
                    # Servo 2 (channel 3)
                    if event == "btn_X":
                        servo_commands[3] = self.servo_position[3] - self.pwm_step_base2
                    if event == "btn_Y":
                        servo_commands[3] = self.servo_position[3] + self.pwm_step_base2

                    # Servo 3 (channel 4)
                    if event == "btn_L1":
                        servo_commands[4] = self.servo_position[4] - self.pwm_step_base3
                    if event == "btn_R1":
                        servo_commands[4] = self.servo_position[4] + self.pwm_step_base3
                    
                    # Servo 5 (channel 5)
                    if event == "btn_L2":
                        servo_commands[5] = self.servo_position[5] - self.pwm_step_base5
                    if event == "btn_R2":
                        servo_commands[5] = self.servo_position[5] + self.pwm_step_base5
                        
                if servo_commands:
                    self.send_servo_cmd(servo_commands, full_command=False)
                
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Exiting...")
            self.serial.close()
            pg.quit()
            
if __name__ == "__main__":
    gamepad_handler = GamepadHandler(com_port="COM4", baudrate=115200)
    gamepad_handler.run(status=True)