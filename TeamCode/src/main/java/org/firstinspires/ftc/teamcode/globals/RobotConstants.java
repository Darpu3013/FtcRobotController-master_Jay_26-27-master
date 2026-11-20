package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class RobotConstants {

    public enum RobotTeam{
        RED, BLUE
    }
    public static RobotTeam robotTeam = RobotTeam.RED;

    public static Pose redCloseGoal = new Pose(58, 58);
    public static Pose redFarGoal = new Pose(58, 84);

    public static Pose blueCloseGoal = new Pose(84, 58);
    public static Pose blueFarGoal = new Pose(84, 84);
    public static final double goalSwitchY = 71;
    public static double goalHeight = 53.5;

    //intake
    public static final String intakeName = "intake";
    public static double flowerMin;

    //flower intake
    public static final String flowerIntakeName = "flowerIntake";

    //stopper
    public static final String stopperName = "stopper";
    public static double stopperMin;
    public static double stopperMax;

    //turret
    public static final String turretName = "turret";
    public static final Motor.GoBILDA turretMotType = Motor.GoBILDA.RPM_312;
    public static boolean turretReversed = false;
    public static double turretPCoeff = 0.02, turretDCoeff = 0.0;
    public static double turretTolerance = 1.0;
    public static final double turretRatio = 2/1;
    public static final double turretTicksPerDegree = (turretMotType.getCPR() * turretRatio) / 360;

    //hood
    public static final String hoodName = "hood";
    public static double hoodMin = 0.0, hoodMax = 1.0;

    // shooter
    public static final String shooterName = "shooter";
    public static boolean shooterReversed = false;
    public static double shooterP = 0, shooterI = 0, shooterD = 0;
    public static double shooterKV = 0;

    public static final double volt = 12.0;

    public static Pose getGoal(Pose robotPose){
        boolean far =  robotPose.y() > goalSwitchY;
        if(robotTeam == RobotTeam.RED){
            return far ? redFarGoal : redCloseGoal;
        }
        else {
            return far ? blueFarGoal : blueCloseGoal;
        }
    }

    public static double getDistance(Pose robotPose) {

        Pose goal = getGoal(robotPose);
        double dx = goal.x() - robotPose.x();
        double dy = goal.y() - robotPose.y();
        return Math.sqrt(dx * dx + dy * dy + goalHeight * goalHeight);

    }

}
