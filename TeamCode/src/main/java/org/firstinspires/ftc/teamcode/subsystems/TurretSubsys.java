package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class TurretSubsys {

    private Motor turretMotor;
    private PDController pdController;
    private double offset;

    public TurretSubsys(HardwareMap hwMap){

        turretMotor = new Motor(hwMap, RobotConstants.turretName, RobotConstants.turretMotType);
        turretMotor.setInverted(RobotConstants.turretReversed);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.stopAndResetEncoder();

        pdController = new PDController(RobotConstants.turretPCoeff, RobotConstants.turretDCoeff);
        pdController.setTolerance(RobotConstants.turretTolerance);

    }

    public double getAngle() {
        return turretMotor.getCurrentPosition() / RobotConstants.turretTicksPerDegree;
    }

    public void turretToAngle(double targetAngle){
        double output = pdController.calculate(getAngle(), targetAngle);
        turretMotor.set(Math.max(-1, Math.min(1, output)));
    }


    public void turretTrack(Pose robotPose){
        Pose goal = RobotConstants.getGoal(robotPose);

        double dx = abs(goal.x() - robotPose.x());
        double dy = abs(goal.y() - robotPose.y());

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(robotPose.heading()) + offset;

        double target = fieldAngle - robotHeading;
        //target = ((target + 180) % 360 + 360) % 360 - 180;

        turretToAngle(target);
    }

    public void resetTurretEncoder() {
        turretMotor.stopAndResetEncoder();
    }

    public void changeOffset(double change){
        offset += change;
    }

    public void updateConstants(){
        turretMotor.setInverted(RobotConstants.turretReversed);
        pdController.setP(RobotConstants.turretPCoeff);
        pdController.setD(RobotConstants.turretDCoeff);
    }

    public void stop() {
        turretMotor.set(0);
    }

}
