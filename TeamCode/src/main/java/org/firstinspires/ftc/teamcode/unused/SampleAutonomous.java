//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
//
//@Autonomous(name="SampleAutonomous", group="Linear OpMode")
//public class SampleAutonomous extends LinearOpMode {
//    private DcMotor motor0 = null;
//    private DcMotor motor1 = null;
//    private DcMotor motor2 = null;
//    private DcMotor motor3 = null;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    double setError0;
//    double setError1;
//    double setError2;
//    double setError3;
//    double kp = 1.0 / 1500;
//    double start0 = 0;
//    double start1 = 0;
//    double start2 = 0;
//    double start3 = 0;
//    double setPoint0 = 0;
//    double setPoint1 = 0;
//    double setPoint2 = 0;
//    double setPoint3 = 0;
//    double position0;
//    double position1;
//    double position2;
//    double position3;
//    double error0;
//    double error1;
//    double error2;
//    double error3;
//    double motorPower0;
//    double motorPower1;
//    double motorPower2;
//    double motorPower3;
//
//    @Override
//    public void runOpMode() {
//        motor0 = hardwareMap.get(DcMotor.class, "motor0");
//        motor1 = hardwareMap.get(DcMotor.class, "motor1");
//        motor2 = hardwareMap.get(DcMotor.class, "motor2");
//        motor3 = hardwareMap.get(DcMotor.class, "motor3");
//
//        motor0.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//
//        position0 = motor0.getCurrentPosition() - start0;
//        position1 = motor1.getCurrentPosition() - start1;
//        position2 = motor2.getCurrentPosition() - start2;
//        position3 = motor3.getCurrentPosition() - start3;
//
//
//        waitForStart();
//
//        start0 = motor0.getCurrentPosition();
//        start1 = motor1.getCurrentPosition();
//        start2 = motor2.getCurrentPosition();
//        start3 = motor3.getCurrentPosition();
//        boolean move1 = true;
//        boolean move1First = true;
//        boolean move2 = false;
//        boolean move2First = true;
//
//        position();
//        while (opModeIsActive()) {
//
//            if (move1) {
//                if(move1First){
//                    start0 -= 200;
//                    move1First = false;
//                }
//                setPoint0 = 1750;
//                setPoint1 = 1750;
//                setPoint2 = 1750;
//                setPoint3 = 1750;
//                if(position0 < error0){
//                    motorPower0 = position0 * kp;
//                    motorPower1 = position1 * kp;
//                    motorPower2 = position2 * kp;
//                    motorPower3 = position3 * kp;
//                } else {
//                    motorPower0 = error0 * kp;
//                    motorPower1 = error1 * kp;
//                    motorPower2 = error2 * kp;
//                    motorPower3 = error3 * kp;
//                }
//                motor0.setPower(motorPower0);
//                motor1.setPower(motorPower0);
//                motor2.setPower(motorPower0);
//                motor3.setPower(motorPower0);
//                position();
//                if ((error0 <= 10 && error0 >= -10)) {
//                    move1 = false;
//                    move2 = true;
//                    motor0.setPower(0);
//                    motor1.setPower(0);
//                    motor2.setPower(0);
//                    motor3.setPower(0);
//                }
//
//            }
//
//
//            /*else if (move2) {
//                if(move2First){
//                    start0 = motor0.getCurrentPosition();
//                    start1 = motor1.getCurrentPosition();
//                    start2 = motor2.getCurrentPosition();
//                    start3 = motor3.getCurrentPosition();
//                    move2First = false;
//                }
//                setPoint0 = -900;
//                setPoint1 = 900;
//                setPoint2 = -900;
//                setPoint3 = 900;
//                if(position0 < error0){
//                    motorPower0 = position0 * kp;
//                    motorPower1 = position1 * kp;
//                    motorPower2 = position2 * kp;
//                    motorPower3 = position3 * kp;
//                } else {
//                    motorPower0 = error0 * kp;
//                    motorPower1 = error1 * kp;
//                    motorPower2 = error2 * kp;
//                    motorPower3 = error3 * kp;
//                }
//
//                motor0.setPower(error0 * kp);
//                motor1.setPower(error1 * kp);
//                motor2.setPower(error2 * kp);
//                motor3.setPower(error3 * kp);
//                position();
//                if ((error0 <= 100 && error0 >= -100) && (error1 <= 100 && error1 >= -100)) {
//                    move2 = false;
//                }
//            }*/
//            telemetry.addData("error0 ", error0);
//            telemetry.addData("error1 ", error1);
//            telemetry.addData("position0 ", position0);
//            telemetry.addData("position1 ", position1);
//            telemetry.addData("start0 ", start0);
//            telemetry.update();
//        }
//    }
//    public void position() {
//        position0 = motor0.getCurrentPosition() - start0;
//        position1 = motor1.getCurrentPosition() - start1;
//        position2 = motor2.getCurrentPosition() - start2;
//        position3 = motor3.getCurrentPosition() - start3;
//        error0 = setPoint0 - position0;
//        error1 = setPoint1 - position1;
//        error2 = setPoint2 - position2;
//        error3 = setPoint3 - position3;
//    }
//}
//
//
