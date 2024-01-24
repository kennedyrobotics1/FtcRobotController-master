package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;


// todo: write your code here
@TeleOp(name = "Climb", group = "Iterative OpMode")

public class Climb extends BasicOpMode_Iterative {
    //variable declarations


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

    double setPointClaw;
    double positionClaw;
    double startClaw;
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

    public void init() {
        climb0 = hardwareMap.get(DcMotor.class, "climb0");
        climb1 = hardwareMap.get(DcMotor.class, "climb1");

        //servo2.setDirection(Servo.Direction.FORWARD);

        //arm0.setDirection(DcMotor.Direction.REVERSE);

        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    }

    public void start(){

        start1 = climb0.getCurrentPosition();
        position1 = climb0.getCurrentPosition() - start1;



        startTime = System.nanoTime();
        updateTime();


    }

    public void loop(){

        if(gamepad2.right_trigger > 0.05){
            setPoint1 += 750 * deltaTime;
        } else if (gamepad2.left_trigger > 0.05){
            setPoint1 -= 750 * deltaTime;
        }
        previousPosition1 = position1;
        position1 = climb0.getCurrentPosition() - start1;
        deltaPosition1 = position1 - previousPosition1;
        error1 = setPoint1 - position1;


        climb0.setPower((error1 * kp));
        climb1.setPower(-(error1 * kp));



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
        telemetry.addData("setpoint ", setPoint1);
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


