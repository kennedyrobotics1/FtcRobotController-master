//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
//@Autonomous (name="ryanTestAuto", group="Linear OpMode")
//public class ryanTestAuto extends LinearOpMode {
//
//    private DcMotor motor0 = null;
//
//    private DcMotor motor1 = null;
//
//    private DcMotor motor2 = null;
//
//    private DcMotor motor3 = null;
//
//    double motorPower0;
//    double motorPower1;
//    double motorPower2;
//    double motorPower3;
//    private CRServo servo0 = null;
//    private ServoImplEx servo1 = null;
//    private ServoImplEx servo2 = null;
//    private DcMotor arm0 = null;
//    private DcMotor arm1 = null;
//    private AnalogInput analogInput;
//    IMU imu;
//    int logoFacingDirectionPosition;
//    int usbFacingDirectionPosition;
//    private static final boolean USE_WEBCAM = true;
//    private TfodProcessor tfod;
//    private VisionPortal visionPortal;
//    private static final String TFOD_MODEL_ASSET = "BlueBox.tflite";
//    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueBox.tflite";
//    private static final String[] LABELS = {
//            "BB",
//    };
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
//        servo0 = hardwareMap.get(CRServo.class, "servo0");
//        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");
//        servo2 = hardwareMap.get(ServoImplEx.class, "servo2");
//
//        arm0  = hardwareMap.get(DcMotor.class, "arm0");
//        arm1  = hardwareMap.get(DcMotor.class, "arm1");
//
//        analogInput = hardwareMap.get(AnalogInput.class, "servoEncoder");
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        logoFacingDirectionPosition = 0; // Up
//        usbFacingDirectionPosition = 2; // Forward
//
//        initTfod();
//
//        waitForStart();
//
//        imu.resetYaw();
//    }
//}
