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
        robot.encoderDriveStrafe(0.6, 16, "right", 5);
        if(ringNum.equals("ZERO")){
            robot.encoderDrive(0.6, 0.6, 48, 48, 5);
            arm.move(-150);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -6, -6, 5);
            arm.move(20);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -42, 42 , 5);

        }
        else if(ringNum.equals("ONE")){
            robot.encoderDrive(0.6, 0.6, 98, 98, 5);
            robot.encoderDrive(0.6, 0.6, -26, 26, 5);
            robot.encoderDrive(0.6, 0.6, 6, 6, 5);
            arm.move(-150);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -6, -6, 5);
            arm.move(20);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -26, 26, 5);
            robot.encoderDrive(0.6, 0.6, 32, 32, 5);
        }
        else if(ringNum.equals("FOUR")){
            robot.encoderDrive(0.6, 0.6, 96, 96, 5);
            arm.move(-150);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -50, -50, 5);
            arm.move(20);
            sleep(1000);
            robot.encoderDrive(0.6, 0.6, -42, 42, 5);

        }
        robot.encoderDriveStrafe(0.6, 46, "right", 5);
        shooter.setPower(0.85);
        shooter.turnOn();
        sleep(250);
        robot.encoderDrive(0.6, 0.6, 0,0, 0);
        shoot();
        shoot();
        robot.encoderDrive(0.6, 0.6, 2,-2, 5);
        shoot();
        robot.encoderDrive(0.6, 0.6, 2, -2, 5);
        shoot();
        robot.encoderDrive(0.6, 0.6, -20,-20, 5);
    }
    public void shoot(){
        shooter.trigger();
        sleep(200);
        shooter.neutral();
        sleep(200);
    }
}