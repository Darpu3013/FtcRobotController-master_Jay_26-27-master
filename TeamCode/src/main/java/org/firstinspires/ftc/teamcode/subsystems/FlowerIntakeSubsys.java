package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class FlowerIntakeSubsys {

    private ServoEx flowerIntakeServo;

    public FlowerIntakeSubsys(final HardwareMap hwMap){
        flowerIntakeServo = new ServoEx(hwMap, RobotConstants.flowerIntakeName);
    }

    public void intakeFlower() {}

}
