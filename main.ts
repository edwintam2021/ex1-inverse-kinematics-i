input.onButtonPressed(Button.A, function () {
    RobotArm.set_delay(50)
    RobotArm.ik(-14, -14, 5)
    basic.pause(1000)
    RobotArm.Servo(RobotArm.Servos.S4, 55)
})
input.onButtonPressed(Button.B, function () {
    RobotArm.set_delay(20)
    RobotArm.ik(-14, 14, 5)
    basic.pause(1000)
    RobotArm.Servo(RobotArm.Servos.S4, 0)
})
basic.showIcon(IconNames.Heart)
RobotArm.robotInitialize(
7,
11,
15,
90,
60,
60,
0
)
basic.forever(function () {
	
})
