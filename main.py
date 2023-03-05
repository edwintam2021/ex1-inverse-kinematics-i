def on_button_pressed_a():
    RobotArm.set_delay(50)
    RobotArm.ik(-14, -14, 5)
    basic.pause(1000)
    RobotArm.servo(RobotArm.Servos.S4, 55)
input.on_button_pressed(Button.A, on_button_pressed_a)

def on_button_pressed_b():
    RobotArm.set_delay(20)
    RobotArm.ik(-14, 14, 5)
    basic.pause(1000)
    RobotArm.servo(RobotArm.Servos.S4, 0)
input.on_button_pressed(Button.B, on_button_pressed_b)

basic.show_icon(IconNames.HEART)
RobotArm.robot_initialize(7, 11, 15, 90, 60, 60, 0)

def on_forever():
    pass
basic.forever(on_forever)
