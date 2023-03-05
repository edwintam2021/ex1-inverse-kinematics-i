/**
 * Robot Arm Library_v1.02
 * Written by VTC STEM Education Centre
 * 16-09-2020
 */
//% color="#03AA74" icon="\uf013" blockGap=8 block="Robot Arm"
namespace RobotArm {

    //* Inverse Kinematics *//

    // Global variable to init Robot Arm
    declare var a1: number ;
    declare var a2: number ;
    declare var a3: number ;

    declare var s1: number ;
    declare var s2: number ;
    declare var s3: number ;

    declare var offSetS1: number ;
    declare var offSetS2: number ;
    declare var offSetS3: number ;

    declare var s3UpLimit: number;
    declare var s3LowLimit: number;
    declare var s2UpLimit: number;
    declare var s2LowLimit: number;
    declare var s1UpLimit: number;
    declare var s1LowLimit: number;

    declare var movement_delay: number ;

    offSetS1 = 0;
    offSetS2 = 0;
    offSetS3 = 0;

    s3UpLimit = 180;
    s3LowLimit= 0;
    s2UpLimit = 180;
    s2LowLimit = 0;
    s1UpLimit = 180;
    s1LowLimit = 0;

    movement_delay = 50;

    /**
     * Initialize the length and offset of the robot arm
     * @param _a1 [0-20] length of the robot arm; eg: 0
     * @param _a2 [0-20] length of the robot arm; eg: 6
     * @param _a3 [0-20] length of the robot arm; eg: 10
    */
    //% blockId=robotInit block="Set Length1 of a1: %_a1|Set Length1 of a2: %_a2|Set Length1 of a3: %_a3|Set Servo1: %servo1|Set Servo2: %servo2|Set Servo3: %servo3|Set Servo4: %servo4"
    //% weight=510
    //% color="#E00064"
    //% _a1.min=0 _a1.max=20
    //% _a2.min=0 _a2.max=20
    //% _a3.min=0 _a3.max=20
    //% _a1.defl=7
    //% _a2.defl=11
    //% _a3.defl=15
    //% _servo1.defl=90
    //% _servo2.defl=60
    //% _servo3.defl=60
    //% _servo4.defl=0
    //% _servo1.shadow="protractorPicker"
    //% _servo2.shadow="protractorPicker"
    //% _servo3.shadow="protractorPicker"
    //% _servo4.shadow="protractorPicker"
    export function robotInitialize(_a1: number, _a2: number, _a3: number, _servo1: number,_servo2: number, _servo3: number, _servo4: number) {
        a1 = _a1;
        a2 = _a2;  
        a3 = _a3;
        old_s1 = _servo1;
        old_s2 = _servo2;
        old_s3 = _servo3;
        RobotArm.Servo(RobotArm.Servos.S1, old_s1)
        basic.pause(1000)
        RobotArm.Servo(RobotArm.Servos.S2, old_s2)
        basic.pause(1000)
        RobotArm.Servo(RobotArm.Servos.S3, old_s3)
        basic.pause(1000)
        RobotArm.Servo(RobotArm.Servos.S4, _servo4)
        basic.pause(100)
    }

    /**
     * Calulation of IK of robot arm
     * @param x [0-15] length of the robot arm; eg: 6
     * @param y [-15-15] length of the robot arm; eg: 0
     * @param z [-10-15] length of the robot arm; eg: 5
    */

    //% block="Set Movement Delay(ms): %ms"
    //% weight=59
    export function set_delay(ms: number) {
        movement_delay = ms
    }

    //% blockId=robotIK block="Move To |x = %x| y = %y| z = %z"
    //% weight=60
    //% x.min=0 x.max=15
    //% y.min=-15 y.max=15
    //% z.min=-10 z.max=15
    // Calculating the variable joint parameters needed
    // to place the end of a kinematic chain
    export function ik(x: number, y: number, z: number):void{
        let _r1: number = 0;
        let _r2: number = 0;
        let _r3: number = 0;
        let _phi1: number = 0;
        let _phi2: number = 0;
        let _phi3: number = 0;
        let _theta1: number = 0;
        let _theta2: number = 0;
        let _theta3: number = 0;

        //The input x coordinate is postive number, force the number to negative for IK calculation
        if (x>0){
            x = x*(-1); //The x coordinate must be negative
        }else if (x==0){
            x = 0.001;
        }

        _r1 = Math.sqrt((x**2)+(y**2));
        _r2 = z - a1;
        _r3 = Math.sqrt((_r1**2)+(_r2**2));
        _phi1 = Math.acos((a3**2-a2**2-_r3**2)/(-2*a2*_r3))* 180/Math.PI;
        _phi2 = Math.atan(_r2/_r1)* 180/Math.PI;
        _phi3 = Math.acos((_r3**2-a2**2-a3**2)/(-2*a2*a3))* 180/Math.PI;
        _theta1 = Math.atan(y/x)* 180/Math.PI;
        _theta2 = (180 - _phi2)-_phi1;
        _theta3 = 180 - _phi3;

        s1 = Math.round(_theta1 + 90) + offSetS1;
        s2 = Math.round(_theta2) + offSetS2;
        s3 = Math.round(180 - _theta3) + offSetS3;

        console.log("IK Servo s3 is:");
        console.log(s3);
        console.log("IK Servo s2 is:");
        console.log(s2);
        console.log("IK Servo s1 is:");
        console.log(s1);

        RobotMotion(s1,s2,s3);
        
    }
    /**
     * Initialize the length and offset of the robot arm
     * @param _offSetS1 [-30-30] length of the robot arm; eg: 0
     * @param _offSetS2 [-30-30] length of the robot arm; eg: 0
     * @param _offSetS3 [-30-30] length of the robot arm; eg: 0
    */

    //% blockId=servoOffsetBlock block="Offset |S1 %_offSetS1| S2 %_offSetS2| S3 %_offSetS3"
    //% weight=504
    //% color="#E00064"

    //% _offSetS1.min=-30 _offSetS1.max=30
    //% _offSetS2.min=-30 _offSetS2.max=30
    //% _offSetS3.min=-30 _offSetS3.max=30
    export function servoOffSet(_offSetS1: number, _offSetS2: number, _offSetS3: number): void{

        offSetS3 = _offSetS3;
        offSetS2 = _offSetS2;
        offSetS1 = _offSetS1;


    }

    /**
     * Servo Limit of the robot arm
     * @param _s1UpLimit [0-180] of the robot arm; eg: 180
     * @param _s1LowLimit [0-180] of the robot arm; eg: 0
     * @param _s2UpLimit [0-180] of the robot arm; eg: 180
     * @param _s2LowLimit [0-180] of the robot arm; eg: 0
     * @param _s3UpLimit [0-180] of the robot arm; eg: 180
     * @param _s3LowLimit [0-180] of the robot arm; eg: 0
    */

    //% blockId=servoOffsetLimitBlock block="Servo Limit |S1: Upper $_s1UpLimit| Lower $_s1LowLimit|S2: Upper $_s2UpLimit| Lower $_s2LowLimit|S3: Upper $_s3UpLimit| Lower $_s3LowLimit"
    //% weight=503
    //% color="#E00064"
    //% _s1UpLimit.shadow="protractorPicker"
    //% _s1LowLimit.shadow="protractorPicker"
    //% _s2UpLimit.shadow="protractorPicker"
    //% _s2LowLimit.shadow="protractorPicker"
    //% _s3UpLimit.shadow="protractorPicker"
    //% _s3LowLimit.shadow="protractorPicker"
    
    export function servoLimitOffSet(_s1UpLimit: number, _s1LowLimit: number,
    _s2UpLimit: number, _s2LowLimit: number,
    _s3UpLimit: number, _s3LowLimit: number): void{

        s3UpLimit = _s3UpLimit + offSetS3;
        s3LowLimit= _s3LowLimit + offSetS3;
        s2UpLimit = _s2UpLimit + offSetS2;
        s2LowLimit = _s2LowLimit + offSetS2;
        s1UpLimit = _s1UpLimit + offSetS1;
        s1LowLimit = _s1LowLimit + offSetS1;

    }

    //* Movement *//

    declare var old_s1: number ;
    declare var old_s2: number ;
    declare var old_s3: number ;

    /**
    * Control the movement of Robot Arm
    */

    function RobotMotion(_newS1: number, _newS2: number, _newS3: number): void {
        
        let signS3: number;
        let signS2: number;
        let signS1: number;
        
        if(_newS3 <= old_s3){
            signS3 = -1;
        }else{
            signS3 = 1;
        }
        if(_newS2 <= old_s2){
            signS2 = -1;
        }else{
            signS2 = 1;
        }
        if(_newS1 <= old_s1){
            signS1 = -1;
        }else{
            signS1 = 1;
        }

        if (_newS3>s3UpLimit){
            _newS3 = s3UpLimit;
            old_s3 = _newS3;

        }else if (_newS3<s3LowLimit){ 
            _newS3 = s3LowLimit;
            old_s3 = _newS3;
        }
        if (_newS2>s2UpLimit){
            _newS2 = s2UpLimit;
            old_s2 = _newS2;

        }else if (_newS2<s2LowLimit){ 
            _newS2 = s2LowLimit;
            old_s2 = _newS2;
        }
        if (_newS1>s1UpLimit){
            _newS1 = s1UpLimit;
            old_s1 = _newS1;

        }else if (_newS1<s1LowLimit){ 
            _newS1 = s1LowLimit;
            old_s1 = _newS1;
        }

        while ((old_s3!=_newS3)||(old_s2!=_newS2)||(old_s1!=_newS1)){
            if(old_s3!=_newS3){
                RobotArm.Servo(RobotArm.Servos.S3, old_s3)
                old_s3 = old_s3 + signS3;
            }
            if(old_s2!=_newS2){
                RobotArm.Servo(RobotArm.Servos.S2, old_s2)
                old_s2 = old_s2 + signS2;
            }
            if(old_s1!=_newS1){
                RobotArm.Servo(RobotArm.Servos.S1, old_s1)
                old_s1 = old_s1 + signS1;
            }
            basic.pause(movement_delay);

            // console.log("Servo s3 is:");
            // console.log(old_s3);
            // console.log("Servo s2 is:");
            // console.log(old_s2);
            // console.log("Servo s1 is:");
            // console.log(old_s1);
        }
        //old_s1 = _newS1;
        //old_s2 = _newS2;
        //old_s3 = _newS3;
    }

    let _myNewS3: number = old_s3;
    let _myNewS2: number = old_s2;
    let _myNewS1: number = old_s1;
    let _stepS3: number = 0;
    let _stepS2: number = 0;
    let _stepS1: number = 0;

    /**
    * Manual Control of Robot Arm
    */
    //% blockId=manualControl block="Set |S1 = %_mS1| S2 = %_mS2| S3 = %_mS3"
    //% weight=110
    export function manualMotion(_mS1: number, _mS2: number, _mS3: number): void {

        _myNewS3 = old_s3 + _mS3;
        _myNewS2 = old_s2 + _mS2;
        _myNewS1 = old_s1 + _mS1;

        _stepS3 = _mS3/10;
        _stepS2 = _mS2/10;
        _stepS1 = _mS1/10;

        if (_myNewS3>s3UpLimit){
            _myNewS3 = s3UpLimit;
            old_s3 = _myNewS3;

        }else if (_myNewS3<s3LowLimit){ 
            _myNewS3 = s3LowLimit;
            old_s3 = _myNewS3;
        }
        if (_myNewS2>s2UpLimit){
            _myNewS2 = s2UpLimit;
            old_s2 = _myNewS2;

        }else if (_myNewS2<s2LowLimit){ 
            _myNewS2 = s2LowLimit;
            old_s2 = _myNewS2;
        }
        if (_myNewS1>s1UpLimit){
            _myNewS1 = s1UpLimit;
            old_s1 = _myNewS1;

        }else if (_myNewS1<s1LowLimit){ 
            _myNewS1 = s1LowLimit;
            old_s1 = _myNewS1;
        }


        while ((old_s3!=_myNewS3)||(old_s2!=_myNewS2)||(old_s1!=_myNewS1)){
            if(old_s3!=_myNewS3){
                RobotArm.Servo(RobotArm.Servos.S3, old_s3)

                old_s3 = old_s3 + _stepS3;

                console.log("Servo s3 is:");
                console.log(old_s3);

            }
            if(old_s2!=_myNewS2){
                RobotArm.Servo(RobotArm.Servos.S2, old_s2)
                old_s2 = old_s2 + _stepS2;
                console.log("Servo s2 is:");
                console.log(old_s2);
            }
            if(old_s1!=_myNewS1){
                RobotArm.Servo(RobotArm.Servos.S1, old_s1)
                old_s1 = old_s1 + _stepS1;
                console.log("Servo s1 is:");
                console.log(old_s1);
            }
            basic.pause(10);

        }
        //old_s1 = _newS1;
        //old_s2 = _newS2;
        //old_s3 = _newS3;
    }


    //* Servo Motor *//

    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06

    export enum Servos {
        S1 = 0x01,
        S2 = 0x02,
        S3 = 0x03,
        S4 = 0x04,
        S5 = 0x05,
        S6 = 0x06,
        S7 = 0x07,
        S8 = 0x08
    }

    let initialized = false

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }

    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=robotbit_servo block="Servo|%index|degree %degree"
    //% weight=40
    //% color="#00aeef"
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function Servo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = (degree * 1800 / 180 + 600) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    /* Joystick */

    export enum JoystickBitPin {
        //% block="B1"
        P13 = DAL.MICROBIT_ID_IO_P13,
        //% block="B2"
        P14 = DAL.MICROBIT_ID_IO_P14,
        //% block="B3"
        P15 = DAL.MICROBIT_ID_IO_P15,
        //% block="B4"
        P16 = DAL.MICROBIT_ID_IO_P16
    }

    export enum rockerType {
        //% block="X"
        X,
        //% block="Y"
        Y
    }

    export enum ButtonType {
        //% block="pressed"
        down = PulseValue.High,
        //% block="released"
        up = PulseValue.Low
    }

    /**
    * initialization joystick
    */
    export function initJoystick(): void {
        pins.digitalWritePin(DigitalPin.P12, 0)
        pins.setPull(DigitalPin.P13, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P14, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P15, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P16, PinPullMode.PullUp)
        //pins.digitalWritePin(DigitalPin.P0, 1)
    }

    //% block="Initialize Joystick"
    //% weight=500
    //% color="#E00064"
    export function init_joystick() {
        initJoystick()
    }

    /**
    * get Button
    */
    //% blockId=getButton block="button %button is pressed"
    //% color="#FFAA00"
    //% weight=29
    export function getButton(button: JoystickBitPin): boolean {
        return (pins.digitalReadPin(<number>button) == 0 ? true : false)
    }

    /**
    * Registers code to run when a joystick event is detected.
    */
    //% blockId=onButtonEvent block="on button %button|is %event" blockExternalInputs=false
    //% color="#FFAA00"
    //% weight=28
    export function onButtonEvent(button: JoystickBitPin, event: ButtonType, handler: Action): void {
        pins.onPulsed(<number>button, <number>event, handler);
    }

    /**
    * get rocker value
    * @param rocker describe parameter here, eg: 1
    */
    //% blockId=getRockerValue block="rocker value of %rocker"
    //% color="#FFAA00"
    //% weight=27
    export function getRockerValue(rocker: rockerType): number {
        switch (rocker) {
            case rockerType.X: return pins.analogReadPin(AnalogPin.P2);
            case rockerType.Y: return pins.analogReadPin(AnalogPin.P1);
            default: return 0;
        }
    }

    /**
    * vibration motor
    * @param time describe parameter here, eg: 100
    */
    //% blockId=Vibration_Motor block="motor vibrate for %time ms"
    //% color="#FFAA00"
    //% weight=26
    export function Vibration_Motor(time: number): void {
        pins.digitalWritePin(DigitalPin.P0, 0)
        basic.pause(time)
        pins.digitalWritePin(DigitalPin.P0, 1)
    }

} 