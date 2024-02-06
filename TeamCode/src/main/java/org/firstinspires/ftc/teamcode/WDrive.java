package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.Servo;


// todo: write your code here
@TeleOp(name = "WDrive", group = "Iterative OpMode")

public class WDrive extends BasicOpMode_Iterative {
    //variable declarations
    private ElapsedTime runtime = new ElapsedTime();

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private CRServo servo0 = null;
    private ServoImplEx servo1 = null;
    private ServoImplEx servo2 = null;
    private AnalogInput analogInput;
    double targetPosition0 = 0;
    double targetPosition1;

    double targetPosition2;

    private DistanceSensor sensorDistance;
    private ColorSensor colorSensor;

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;

    private DcMotor climb0 = null;
    private DcMotor climb1 = null;

    private CRServo launcher = null;
    double kp = 1.0/100;
    double kpClaw = 1.0/200;
    double ki = 0;
    double kd = 0.01/120;
    double setPoint = 0;
    double position;
    double start = 0;
    double error = setPoint - position;

    double start5;
    double start6;
    double start7;
    double start8;
    double setPoint1 = 0;
    double position1;
    double start1 = 0;
    double error1 = setPoint1 - position1;

    double setPointClaw;
    double positionClaw;
    double errorClaw = setPointClaw - positionClaw;

    double startTime;
    double previousTime;
    double time;
    double deltaTime;

    double previousPosition;
    double deltaPosition;
    double velocity;

    double previousPosition1;
    double deltaPosition1;
    double velocity1;

    double previousPositionClaw;
    double deltaPositionClaw;
    double velocityClaw;

    double motor0Power, motor1Power, motor2Power, motor3Power;

    boolean forward = true;
    boolean back = true;
    double counter = 0;
    double servoPosition;

    double newTime;
    double oldTime;

    boolean slow = false;

    double newTime1;

    double oldTime1;

    boolean slow1 = false;


    double armPower = 0;

    double realArmPower = 0;

    boolean first = true;

    double startClaw;
    boolean moveTurnFirst = true;


    public void init() {
        motor0  = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        arm0  = hardwareMap.get(DcMotor.class, "arm0");
        arm1  = hardwareMap.get(DcMotor.class, "arm1");

        climb0 = hardwareMap.get(DcMotor.class, "climb0");
        climb1 = hardwareMap.get(DcMotor.class, "climb1");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        servo0 = hardwareMap.get(CRServo.class, "servo0");
        launcher = hardwareMap.get(CRServo.class, "launcher");
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");

        servo2 = hardwareMap.get(ServoImplEx.class, "servo2");

        analogInput = hardwareMap.get(AnalogInput.class, "servoEncoder");
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward






        //arm0.setDirection(DcMotor.Direction.REVERSE);

    }

    public void start(){
        start = arm0.getCurrentPosition();
        position = arm0.getCurrentPosition() - start;

        start1 = climb0.getCurrentPosition();
        position1 = climb0.getCurrentPosition() - start1;

        imu.resetYaw();



        startTime = System.nanoTime();
        updateTime();


        counter = 0;
        previousPositionClaw = servoPosition;
        servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;
        startClaw = servoPosition;

        positionClaw = servoPosition - startClaw;
        setPointClaw = servoPosition;
        errorClaw = setPointClaw - servoPosition;

        targetPosition0 = 0;
        servo0.setPower(-(errorClaw * kpClaw));

        targetPosition1 = 0.08;
        targetPosition2 = 0.92;
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);

        launcher.setPower(0);

        oldTime = 0;
        newTime = 0;

        slow = false;

        oldTime1 = 0;
        newTime1 = 0;

        slow1 = false;

        armPower = (error * kp - (kd * velocity));
        realArmPower = Math.max(armPower, -0.7);
        boolean moveTurnFirst = true;
    }

    public void loop(){
        servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;

        if(first){
            startClaw = servoPosition;
            first = false;
        }
        if(errorClaw >= 200 && forward){
            counter += 1;
            forward = false;
            back = true;
        }
        if(errorClaw <= -200 && back){
            counter -= 1;
            back = false;
            forward = true;
        }

        if(gamepad2.right_stick_y < -0.15){
            setPoint += 1000 * deltaTime;
        } else if (gamepad2.right_stick_y > 0.15){
            setPoint -= 1000 * deltaTime;
        }

        if(gamepad2.y){
            setPoint = 0;
        }

        if(gamepad2.right_trigger > 0.05){
            setPoint1 += 2400 * deltaTime;
        } else if (gamepad2.left_trigger > 0.05){
            setPoint1 -= 2400 * deltaTime;
        }

        previousPosition = position;
        position = arm0.getCurrentPosition() - start;
        deltaPosition = position - previousPosition;
        error = setPoint - position;

        armPower = (error * kp - (kd * velocity));
        realArmPower = Math.max(armPower, -0.35);

        arm0.setPower(realArmPower);
        arm1.setPower(-realArmPower);

        previousPosition1 = position1;
        position1 = climb0.getCurrentPosition() - start1;
        deltaPosition1 = position1 - previousPosition1;
        error1 = setPoint1 - position1;

        climb0.setPower(error1 * kp);
        climb1.setPower(-(error1 * kp));

        if(gamepad2.dpad_up){
            launcher.setPower(0.5);
        } else {
            launcher.setPower(0);
        }

        updateTime();

        velocity = deltaPosition / deltaTime;



        if(gamepad1.dpad_up){
            motor0Power = 0.5;
            motor1Power = 0.5;
            motor2Power = 0.5;
            motor3Power = 0.5;
        } else if(gamepad1.dpad_down) {
            motor0Power = -0.5;
            motor1Power = -0.5;
            motor2Power = -0.5;
            motor3Power = -0.5;
        } else if(gamepad1.dpad_left){
            motor0Power = -0.5;
            motor1Power = 0.5;
            motor2Power = -0.5;
            motor3Power = 0.5;
        } else if(gamepad1.dpad_right){
            motor0Power = 0.4;
            motor1Power = -0.4;
            motor2Power = 0.4;
            motor3Power = -0.4;
        } else {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            motor0Power = (y + x + r) / denominator;
            motor1Power = (y - x - r) / denominator;
            motor2Power = (y - x + r) / denominator;
            motor3Power = (y + x - r) / denominator;
        }
        if(gamepad1.a){
            motor0.setPower(0.4 * motor0Power);
            motor1.setPower(0.4 * motor1Power);
            motor2.setPower(0.4 * motor2Power);
            motor3.setPower(0.4 * motor3Power);
        } else {
            motor0.setPower(motor0Power);
            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
            motor3.setPower(motor3Power);
        }


        velocityClaw = deltaPositionClaw / deltaTime;



        if(gamepad2.left_bumper){
            if(gamepad2.left_stick_y <= -0.2){
                setPointClaw += 215 * deltaTime * 0.2;
            } else if (gamepad2.left_stick_y >= 0.2){
                setPointClaw -= 215 * deltaTime * 0.2;
            }
        } else {
            if(gamepad2.left_stick_y <= -0.2){
                setPointClaw += 215 * deltaTime;
            } else if (gamepad2.left_stick_y >= 0.2){
                setPointClaw -= 215 * deltaTime;
            }
        }

        errorClaw = setPointClaw - servoPosition;
        servo0.setPower(-(errorClaw * kpClaw));

        if(gamepad2.right_bumper){

            realArmPower = Math.min(armPower, 0.6);
            setPoint = 1652;
        }

        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);


        if (gamepad2.b){
            targetPosition2 = 0.41;
            slow = false;
        } else if (gamepad2.x){
            targetPosition2 = 0.92;
            oldTime = runtime.seconds();
            slow = true;
        } /*else if(gamepad2.a){
            targetPosition2 = 0;
            slow = false;
        }*/

        newTime = runtime.seconds();

        /*if(newTime - oldTime >= 0.5 && slow){
            targetPosition2 = 0.2;
        }*/

        if (gamepad2.dpad_left){
            targetPosition1 = 0.55;
            slow1 = false;
        } else if (gamepad2.dpad_right){
            targetPosition1 = 0.08;
            oldTime1 = runtime.seconds();
            slow1 = true;
        } /*else if(gamepad2.dpad_down){
            targetPosition1 = 0;
            slow1 = false;
        }*/

        newTime1 = runtime.seconds();
        /*
        if(newTime1 - oldTime1 >= 0.5 && slow1){
            targetPosition1 = 0.2;
        }*/

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
        /*telemetry.addData("ServoStart ", startClaw);
        telemetry.addData("Current Position Arm0: ", arm0.getCurrentPosition());
        telemetry.addData("Current Position Climb0: ", climb0.getCurrentPosition());
        telemetry.addData("Current servoPosition: ", servoPosition);
        telemetry.addData("Motor power ", motor0Power);
        telemetry.addData("Arm Power: ", realArmPower);*/
        telemetry.addData("TargetPosition1 ", targetPosition1);
        telemetry.addData("TargetPosition2 ", targetPosition2);
        telemetry.addData("servo position1 ", servo1.getPosition());
        telemetry.addData("servo position2 ", servo2.getPosition());

        telemetry.update();
    }
    public void updateTime() {
        previousTime = time;
        time = ((double) (System.nanoTime() - startTime) ) / Math.pow(10,9);
        deltaTime = time - previousTime;
    }

}



//Gamepad1 controls:
//Left Bumper - slow mode
//Left stick - move and strafe
//Right stick - rotate
//
//
//
//Gamepad2 controls:
//Left and Right Triggers - raise and lower climb
//Left Bumper - claw slow mode
//Right Bumper - Claw arm reset to start position
//Up Dpad - Launcher
//Left Dpad - open left claw
//Down Dpad - Stop left claw
//Right Dpad - close left claw
//X - Close right claw
//A - stop right claw
//B - open right claw
//Y - reset four-bar position
//Left stick - claw arm move
//Right stick - four-bar move
