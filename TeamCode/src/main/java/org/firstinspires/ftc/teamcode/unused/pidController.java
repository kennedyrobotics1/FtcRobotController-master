//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class pidController{
//    private DcMotor arm0 = null;
//    private DcMotor arm1 = null;
//    static double integralSum = 0;
//    static double Kp = 1;
//    static double Ki = 0;
//    static double Kd = 0;
//
//    static ElapsedTime timer = new ElapsedTime();
//    private static double lastError = 0;
//    /*public void runOpMode() throws  InterruptedException {
//        arm0 = hardwareMap.get(DcMotor.class, "arm0");
//        arm1 = hardwareMap.get(DcMotor.class, "arm1");
//        arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//        while(opModeIsActive()){
//            double arm0Power = PIDControl(100, arm0.getCurrentPosition());
//            double arm1Power = PIDControl(100, arm1.getCurrentPosition());
//            arm0.setPower(arm0Power);
//            arm1.setPower(arm1Power);
//        }
//    }
//*/
//    public static double PIDControl(double reference, double state){
//        double error = reference - state;
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//
//        timer.reset();
//
//        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
//    }
//}