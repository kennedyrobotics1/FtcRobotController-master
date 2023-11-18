package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



// todo: write your code here
@TeleOp(name = "Test", group = "Iterative OpMode")

public class Test extends BasicOpMode_Iterative {
    //variable declarations
    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    double kp = 3.0/120;
    double ki = 0;
    double kd = 0.01/120;
    double setPoint = 120;
    double position;
    double start = 0;
    double error = setPoint - position;

    double startTime;
    double previousTime;
    double time;
    double deltaTime;

    double previousPosition;
    double deltaPosition;
    double velocity;

    public void init() {
        arm0  = hardwareMap.get(DcMotor.class, "arm0");
        arm1  = hardwareMap.get(DcMotor.class, "arm1");
        arm0.setDirection(DcMotor.Direction.REVERSE);
    }

    public void start(){
        start = arm0.getCurrentPosition();
        position = arm0.getCurrentPosition() - start;

        startTime = System.nanoTime();
        updateTime();

    }

    public void loop(){
        if(gamepad2.y){
            setPoint += 100 * deltaTime;
        } else if (gamepad2.a){
            setPoint -= 100 * deltaTime;
        }
        previousPosition = position;
        position = arm0.getCurrentPosition() - start;
        deltaPosition = position - previousPosition;
        error = setPoint - position;
        arm0.setPower(-(error * kp - (kd * velocity)));
        arm1.setPower(-(error * kp - (kd * velocity)));

        updateTime();

        velocity = deltaPosition / deltaTime;

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