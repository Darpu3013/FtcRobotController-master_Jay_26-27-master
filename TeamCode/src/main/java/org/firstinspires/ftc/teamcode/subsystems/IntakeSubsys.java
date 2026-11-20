package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class IntakeSubsys extends SubsystemBase {

    private Motor intakeMotor;

    public IntakeSubsys(final HardwareMap hwMap) {

        intakeMotor = new Motor(hwMap, RobotConstants.intakeName, Motor.GoBILDA.BARE);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

    }

    public void runIntake(double power){
        intakeMotor.set(power);
    }

}
