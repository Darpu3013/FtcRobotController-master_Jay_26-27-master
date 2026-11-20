package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class ShooterSubsys {

    private Motor shooterMotor;
    private VoltageSensor voltageSensor;

    private PIDFController pidf = new PIDFController(RobotConstants.shooterP, RobotConstants.shooterI, RobotConstants.shooterD, 0);
    InterpLUT shooterLUT;

    private double voltage = RobotConstants.volt;
    private int loopCount = 0;

    public ShooterSubsys(final HardwareMap hwMap) {

        shooterMotor = new Motor(hwMap, RobotConstants.shooterName).setInverted(RobotConstants.shooterReversed).setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setRunMode(Motor.RunMode.RawPower);

        voltageSensor = hwMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();

        shooterLUT = new InterpLUT();

        shooterLUT.add(1, 2);
        shooterLUT.add(3,5);
        shooterLUT.add(5,6);
        shooterLUT.add(7,8);
        shooterLUT.add(9,10);

        shooterLUT.createLUT();

    }

    private double getVoltage() {
        if (loopCount++ % 50 == 0) {
            voltage = voltageSensor.getVoltage();
        }
        return voltage;
    }
    public void setPower(double power) {
        driveToVelocity(power);
    }

    private void driveToVelocity(double target) {
        double vel = shooterMotor.getCorrectedVelocity();
        double pidOutput = pidf.calculate(vel, target);
        double feedforward = target * RobotConstants.shooterKV;
        double output = (feedforward + pidOutput) * (RobotConstants.volt / getVoltage());
        shooterMotor.set(Math.max(-1, Math.min(1, output))); // idk if this is how you range
    }
    public void runLUT(Pose robotPose){
        double distance = RobotConstants.getDistance(robotPose);
        driveToVelocity(shooterLUT.get(distance));
    }

    public double getVelocity(){
        return shooterMotor.getCorrectedVelocity();
    }

    public void updatePIDF(){

        pidf.setPIDF(RobotConstants.shooterP, RobotConstants.shooterI, RobotConstants.shooterD, 0);

    }

}
