package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;


// todo: write your code here
@TeleOp(name = "DrivePID", group = "Iterative OpMode")

public class DrivePID extends BasicOpMode_Iterative {
    //variable declarations
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;

    private DistanceSensor sensorDistance;
    private ColorSensor colorSensor;

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    double kp = 3.0/120;
    double ki = 0;
    double kd = 0.01/120;
    double setPoint = 0;
    double position;
    double start = 0;
    double error = setPoint - position;

    double setPoint0 = 0;
    double position0;
    double start0 = 0;
    double error0 = setPoint0 - position0;

    double setPoint1 = 0;
    double position1;
    double start1 = 0;
    double error1 = setPoint1 - position1;

    double setPoint2 = 0;
    double position2;
    double start2 = 0;
    double error2 = setPoint2 - position2;

    double setPoint3 = 0;
    double position3;
    double start3 = 0;
    double error3 = setPoint3 - position3;

    double startTime;
    double previousTime;
    double time;
    double deltaTime;

    double previousPosition;
    double deltaPosition;
    double velocity;

    double motor0Power, motor1Power, motor2Power, motor3Power;

    public void init() {
        motor0  = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        arm0  = hardwareMap.get(DcMotor.class, "arm0");
        arm1  = hardwareMap.get(DcMotor.class, "arm1");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        arm0.setDirection(DcMotor.Direction.REVERSE);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    }

    public void start(){
        start = arm0.getCurrentPosition();
        position = arm0.getCurrentPosition() - start;

        start0 = motor0.getCurrentPosition();
        position0 = motor0.getCurrentPosition() - start0;

        start1 = motor1.getCurrentPosition();
        position1 = motor1.getCurrentPosition() - start1;

        start2 = motor2.getCurrentPosition();
        position2 = motor2.getCurrentPosition() - start2;

        start3 = motor3.getCurrentPosition();
        position3 = motor3.getCurrentPosition() - start3;

        startTime = System.nanoTime();
        updateTime();
    }

    public void loop(){
        if(gamepad2.y){
            setPoint += 100 * deltaTime;
        } else if (gamepad2.a){
            setPoint -= 100 * deltaTime;
        }

        start0 = motor0.getCurrentPosition();
        if(setPoint0 <= start0 + 50){
            setPoint0 += gamepad1.left_stick_x;
        }

        previousPosition = position;
        position = arm0.getCurrentPosition() - start;
        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;
        deltaPosition = position - previousPosition;
        error = setPoint - position;
        arm0.setPower(-(error * kp - (kd * velocity)));
        arm1.setPower(-(error * kp - (kd * velocity)));

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
            motor0.setPower(0.6 * motor0Power);
            motor1.setPower(0.6 * motor1Power);
            motor2.setPower(0.6 * motor2Power);
            motor3.setPower(0.6 * motor3Power);
        } else {
            motor0.setPower(motor0Power);
            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
            motor3.setPower(motor3Power);
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


        telemetry.addData("Velocity: ", velocity);
        telemetry.addData("Delta Time: ", deltaTime);
        telemetry.addData("Time: ", time);
        telemetry.addData("Current Error: ", error);
        telemetry.addData("Current Position Arm0: ", arm0.getCurrentPosition());
        telemetry.addData("Current Position Arm1: ", arm1.getCurrentPosition());
        telemetry.addData("Current Power: ", error * kp);
        telemetry.update();
    }
    public void updateTime() {
        previousTime = time;
        time = ((double) (System.nanoTime() - startTime) ) / Math.pow(10,9);
        deltaTime = time - previousTime;
    }

}