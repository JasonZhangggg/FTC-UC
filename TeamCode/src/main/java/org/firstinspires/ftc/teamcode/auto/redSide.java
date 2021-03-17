    package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.ArmSubsystem;
import org.firstinspires.ftc.teamcode.robot.DriveTrainStrafe;
import org.firstinspires.ftc.teamcode.robot.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.vision.UGRectDetector;

@Autonomous(name = "redSide", group = "UltimateGoal")
//@Disabled
public class redSide extends LinearOpMode {
    DriveTrainStrafe robot = new DriveTrainStrafe();
    IntakeSubsystem intake = new IntakeSubsystem();
    ShooterSubsystem shooter = new ShooterSubsystem();
    ArmSubsystem arm = new ArmSubsystem();
    UGRectDetector vision = new UGRectDetector();
    ElapsedTime runtime = new ElapsedTime();
    String ringNum = "";
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this, "teleop");
        intake.init(hardwareMap, this, "teleop");
        shooter.init(hardwareMap, this, "teleop");
        arm.init(hardwareMap, this, "teleop");
        vision.init(hardwareMap, "Webcam");
        waitForStart();
        ringNum = vision.getStack().toString();
        telemetry.addData("ringNum", ringNum);
        telemetry.update();
        robot.encoderDriveStrafe(0.4, 30, "right", 4);
        robot.encoderDriveStrafe(0.5, 2, "left", 5);

        if(ringNum.equals("ZERO")){
            robot.encoderDrive(0.6, 0.6, 55, 55, 5);
            robot.encoderDriveStrafe(0.4, 10, "right", 1.5);
            arm.move(-180);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -6, -6, 5);
            arm.rotateForward();
            sleep(500);
            robot.encoderDriveStrafe(0.4, 5, "left", 1.5);
            robot.encoderDrive(0.6, 0.6, -42, 42 , 5);
            robot.encoderDrive(0.6, 0.6, -6, -6, 5);

        }
        else if(ringNum.equals("ONE")){
            robot.encoderDrive(0.6, 0.6, 98, 98, 5);
            robot.encoderDriveStrafe(0.4, 10, "right", 1.5);
            robot.encoderDriveStrafe(0.4, 5, "left", 1.5);
            robot.encoderDrive(0.6, 0.6, -24, 24, 5); // 24 instead
            robot.encoderDrive(0.6, 0.6, 6, 6, 5);
            arm.move(-180);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -6, -6, 5);
            arm.rotateForward();
            sleep(500);
            robot.encoderDrive(0.6, 0.6, -18, 18, 5);
            robot.encoderDrive(0.6, 0.6, 36, 36, 5);
        }
        else if(ringNum.equals("FOUR")){
            robot.encoderDrive(0.5, 0.5, 101, 101, 5);
            robot.encoderDriveStrafe(0.4, 10, "right", 1.5);
            arm.move(-180);
            sleep(1000);
            robot.encoderDrive(0.5, 0.5, -42, -42, 5);
            arm.rotateForward();
            sleep(500);
            robot.encoderDriveStrafe(0.4, 5, "left", 1.5);
            robot.encoderDrive(0.6, 0.6, -42, 42, 5);
        }
        robot.encoderDriveStrafe(0.4, 30, "left", 3);
        robot.encoderDriveStrafe(0.4, 18, "right", 5);
        shooter.setPower(0.87);
        shooter.turnOn();
        sleep(250);
        shoot();
        shoot();
        shoot();
        shoot();
        robot.encoderDrive(0.6, 0.6, -10,-10, 5);
    }
    public void shoot(){
        shooter.trigger();
        sleep(200);
        shooter.neutral();
        sleep(200);
    }
}