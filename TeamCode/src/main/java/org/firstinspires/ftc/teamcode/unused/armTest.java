//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
//
//
//// todo: write your code here
//@TeleOp(name = "Test", group = "Iterative OpMode")
//
//public class armTest extends BasicOpMode_Iterative {
//    //variable declarations
//    private DcMotor arm0 = null;
//    private DcMotor arm1 = null;
//    private DcMotor climb0 = null;
//    private DcMotor climb1 = null;
//    double kp = 1.0/100;
//    double ki = 0;
//    double kd = 0.01/120;
//    double setPoint = 0;
//    double position;
//    double start = 0;
//    double error = setPoint - position;
//
//    double setPoint1 = 0;
//    double position1;
//    double start1 = 0;
//    double error1 = setPoint1 - position1;
//
//    double startTime;
//    double previousTime;
//    double time;
//    double deltaTime;
//
//    double previousPosition;
//    double deltaPosition;
//    double velocity;
//
//    public void init() {
//        arm0  = hardwareMap.get(DcMotor.class, "arm0");
//        arm1  = hardwareMap.get(DcMotor.class, "arm1");
//        //arm0.setDirection(DcMotor.Direction.REVERSE);
//        climb0 = hardwareMap.get(DcMotor.class, "climb0");
//        climb1 = hardwareMap.get(DcMotor.class, "climb1");
//    }
//
//    public void start(){
//        start = arm0.getCurrentPosition();
//        position = arm0.getCurrentPosition() - start;
//
//        startTime = System.nanoTime();
//        updateTime();
//        start1 = climb0.getCurrentPosition();
//        position1 = climb0.getCurrentPosition() - start1;
//
//    }
//
//    public void loop(){
//        if(gamepad2.right_trigger > 0.05){
//            setPoint1 += 1500 * deltaTime;
//        } else if (gamepad2.left_trigger > 0.05){
//            setPoint1 -= 1500 * deltaTime;
//        }
//        if(gamepad1.y){
//            setPoint += 150 * deltaTime;
//        } else if (gamepad1.a){
//            setPoint -= 150 * deltaTime;
//        }
//        previousPosition = position;
//        position = arm0.getCurrentPosition() - start;
//        deltaPosition = position - previousPosition;
//        error = setPoint - position;
//        arm0.setPower((error * kp - (kd * velocity)));
//        arm1.setPower((error * kp - (kd * velocity)));
//
//        updateTime();
//
//        velocity = deltaPosition / deltaTime;
//
//        position1 = climb0.getCurrentPosition() - start1;
//        error1 = setPoint1 - position1;
//
//
//        climb0.setPower((error1 * kp));
//        climb1.setPower((error1 * kp));
//
//        /*telemetry.addData("Velocity: ", velocity);
//
//        telemetry.addData("Delta Time: ", deltaTime);
//        telemetry.addData("Time: ", time);
//        telemetry.addData("SetPoint ", setPoint);
//        telemetry.addData("Current Error: ", error);
//        telemetry.addData("Position ", position);
//        telemetry.addData("Current Position Arm0: ", arm0.getCurrentPosition());
//        telemetry.addData("Current Position Arm1: ", arm1.getCurrentPosition());
//        telemetry.addData("Current Power: ", error * kp);*/
//        telemetry.addData("Position ", position);
//        telemetry.update();
//    }
//    public void updateTime() {
//        previousTime = time;
//        time = ((double) (System.nanoTime() - startTime) ) / Math.pow(10,9);
//        deltaTime = time - previousTime;
//    }
//
//}