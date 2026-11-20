package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.teamcode.globals.RobotConstants;

public class HoodSubsys extends SubsystemBase {

    private ServoEx hoodServo;
    private double hoodPos = 0.0;
    private InterpLUT hoodLUT;
    private final double minDist, maxDist;



    public HoodSubsys(final HardwareMap hwMap){
        hoodServo = new ServoEx(hwMap, RobotConstants.hoodName);

        hoodLUT = new InterpLUT();

        hoodLUT.add(1, 2);
        hoodLUT.add(3,4);
        hoodLUT.add(5,6 );
        hoodLUT.add(7,8);
        hoodLUT.add(9, 10);

        hoodLUT.createLUT();

        minDist = 1;
        maxDist = 9;

    }

    public void hoodTo(double postion){
        hoodPos = Math.max(RobotConstants.hoodMin, Math.min(RobotConstants.hoodMax, postion));
        hoodServo.set(hoodPos);
    }

    public double getPos(){
        return hoodPos;
    }

    public double getDistance(Pose robotPose){

        return RobotConstants.getDistance(robotPose);

    }

    public void hoodTrack(Pose robotPose){

        double dist = getDistance(robotPose);
        dist = Math.max(minDist, Math.min(maxDist, dist));
        hoodTo(hoodLUT.get(dist));

    }


}
