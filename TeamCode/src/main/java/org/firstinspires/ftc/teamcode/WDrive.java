package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.Servo;


// todo: write your code here
@TeleOp(name = "WDrive", group = "Iterative OpMode")

public class WDrive extends BasicOpMode_Iterative {
    //variable declarations
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private CRServo servo0 = null;
    private CRServo servo1 = null;
    private CRServo servo2 = null;
    private AnalogInput analogInput;
    double targetPosition0 = 0;
    double targetPosition1 = 0;

    private DistanceSensor sensorDistance;
    private ColorSensor colorSensor;

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;

    private DcMotor climb0 = null;
    private DcMotor climb1 = null;
    double kp = 1.0/100;
    double kpClaw = 1.0/100;
    double ki = 0;
    double kd = 0.01/120;
    double setPoint = 0;
    double position;
    double start = 0;
    double error = setPoint - position;

    double setPoint1 = 0;
    double position1;
    double start1 = 0;
    double error1 = setPoint1 - position1;

    double setPointClaw = 0;
    double positionClaw;
    double startClaw = 0;
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


    double servoPosition;

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
        //servo0.setDirection(Servo.Direction.FORWARD);
        servo1 = hardwareMap.get(CRServo.class, "servo1");

        //servo1.setDirection(Servo.Direction.REVERSE);
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        analogInput = hardwareMap.get(AnalogInput.class, "servoEncoder");

        //servo2.setDirection(Servo.Direction.FORWARD);

        //arm0.setDirection(DcMotor.Direction.REVERSE);

        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    }

    public void start(){
        start = arm0.getCurrentPosition();
        position = arm0.getCurrentPosition() - start;

        start1 = climb0.getCurrentPosition();
        position1 = climb0.getCurrentPosition() - start1;



        startTime = System.nanoTime();
        updateTime();
        servoPosition = analogInput.getVoltage() / 3.3 * 360;


        startClaw = servoPosition;
        positionClaw = servoPosition - startClaw;


        targetPosition0 = 0;
        servo0.setPower(targetPosition0);
        targetPosition1 = 0;
        servo1.setPower(0);
        servo2.setPower(0);
    }

    public void loop(){
        servoPosition = analogInput.getVoltage() / 3.3 * 360;
        if(gamepad2.right_stick_y < -0.15){
            setPoint += 1000 * deltaTime;
        } else if (gamepad2.right_stick_y > 0.15){
            setPoint -= 1000 * deltaTime;
        }
        if(gamepad2.right_trigger > 0.05){
            setPoint1 += 750 * deltaTime;
        } else if (gamepad2.left_trigger > 0.05){
            setPoint1 -= 750 * deltaTime;
        }
        previousPosition = position;
        position = arm0.getCurrentPosition() - start;
        deltaPosition = position - previousPosition;
        error = setPoint - position;


        arm0.setPower((error * kp - (kd * velocity)));
        arm1.setPower(-(error * kp - (kd * velocity)));

        previousPosition1 = position1;
        position1 = climb0.getCurrentPosition() - start1;
        deltaPosition1 = position1 - previousPosition1;
        error1 = setPoint1 - position1;


        climb0.setPower(error1 * kp);
        climb1.setPower(error1 * kp);

        previousPositionClaw = positionClaw;
        positionClaw = servoPosition - startClaw;
        deltaPositionClaw = positionClaw - previousPositionClaw;
        errorClaw = setPointClaw - positionClaw;

        servo0.setPower(errorClaw * kpClaw);

        updateTime();

        velocity = deltaPosition / deltaTime;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);


        motor0Power = (y + x + r) / denominator;
        motor1Power = (y - x - r) / denominator;
        motor2Power = (y - x + r) / denominator;
        motor3Power = (y + x - r) / denominator;

        if(gamepad1.left_bumper){
            motor0.setPower(0.2 * motor0Power);
            motor1.setPower(0.2 * motor1Power);
            motor2.setPower(0.2 * motor2Power);
            motor3.setPower(0.2 * motor3Power);
        } else {
            motor0.setPower(motor0Power);
            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
            motor3.setPower(motor3Power);
        }

        //servo1.setPosition(targetPosition1);
        //servo2.setPosition(targetPosition1);
        servo1.setPower(-targetPosition1);
        servo2.setPower(targetPosition1);


        if(gamepad2.left_stick_y <= -0.2 && setPointClaw < 360){
            setPointClaw += 100 * deltaTime;
        } else if (gamepad2.left_stick_y >= 0.2 && setPointClaw > 0){
            setPointClaw -= 100 * deltaTime;
        }
        if(gamepad2.dpad_up){
            setPointClaw = 180;
        }


        if (gamepad2.b){
            // targetPosition1 -= 0.003;
            targetPosition1 = -0.15;
        } else if (gamepad2.x){
            targetPosition1 = 0.125;
        }else if(gamepad2.a){
            targetPosition1 = 0;
        }

        int A = (65280 >> 24) & 0xff; // or color >>> 24
        int R = (65280 >> 16) & 0xff;
        int G = (65280 >>  8) & 0xff;
        int B = (65280      ) & 0xff;


        /*telemetry.addData("Distance: ", sensorDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("ARGB: ", colorSensor.argb());
        telemetry.addData("Red: ", R);
        telemetry.addData("Green: ", G);
        telemetry.addData("Blue: ", B);
        telemetry.addData("Alpha: ", A);
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("Blue: ", colorSensor.blue());*/


        /*telemetry.addData("Velocity: ", velocity);
        telemetry.addData("Delta Time: ", deltaTime);
        telemetry.addData("Time: ", time);
        telemetry.addData("Current Error: ", error);*/
        telemetry.addData("Current Position Arm0: ", arm0.getCurrentPosition());
        telemetry.addData("Current Position Climb0: ", climb0.getCurrentPosition());
        //telemetry.addData("Current Position Arm1: ", arm1.getCurrentPosition());
        telemetry.addData("Current servoPosition: ", servoPosition);
        telemetry.addData("Motor power ", motor0Power);
        telemetry.update();
    }
    public void updateTime() {
        previousTime = time;
        time = ((double) (System.nanoTime() - startTime) ) / Math.pow(10,9);
        deltaTime = time - previousTime;
    }

}


