package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class RobotSpecificConstants {
    
    public static final boolean IS_COMPETITION_ROBOT = false; // Wonder if we can test robot itself to determine this

    public static double getFrontToBackAxleToAxleMeters() {
        return Units.inchesToMeters(ROBOT_SPECIFIC_CONSTANTS.getFrontToBackAxleToAxleInches());
    }

    public static double getSideToSideTreadCenterToTreadCenterMeters() {
        return Units.inchesToMeters(ROBOT_SPECIFIC_CONSTANTS.getSideToSideTreadCenterToTreadCenterInches());
    }

    public static Rotation2d getAngleOffsetModule0() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod0());
    }

    public static Rotation2d getAngleOffsetModule1() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod1());
    }

    public static Rotation2d getAngleOffsetModule2() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod2());
    }

    public static Rotation2d getAngleOffsetModule3() {
        return Rotation2d.fromDegrees(ROBOT_SPECIFIC_CONSTANTS.getAngleOffsetDegreesMod3());
    }

    private interface SpecificConstants {
        double getFrontToBackAxleToAxleInches();
        double getSideToSideTreadCenterToTreadCenterInches();
        double getAngleOffsetDegreesMod0();
        double getAngleOffsetDegreesMod1();
        double getAngleOffsetDegreesMod2();
        double getAngleOffsetDegreesMod3();
    }

    private static final class CompetitionRobot implements SpecificConstants {
        public double getFrontToBackAxleToAxleInches() { return 24.875; }
        public double getSideToSideTreadCenterToTreadCenterInches() { return 24.5; }
        public double getAngleOffsetDegreesMod0() { return 0.0; }
        public double getAngleOffsetDegreesMod1() { return 0.0; }
        public double getAngleOffsetDegreesMod2() { return 0.0; }
        public double getAngleOffsetDegreesMod3() { return 0.0; }
    }

    private static final class TestRobot implements SpecificConstants {
        public double getFrontToBackAxleToAxleInches() { return 27.25; }
        public double getSideToSideTreadCenterToTreadCenterInches() { return 27.25; }
        public double getAngleOffsetDegreesMod0() { return 22.939; }
        public double getAngleOffsetDegreesMod1() { return 139.394531; }
        public double getAngleOffsetDegreesMod2() { return 241.962891; }
        public double getAngleOffsetDegreesMod3() { return 21.708984; }
    }

    private static final SpecificConstants ROBOT_SPECIFIC_CONSTANTS = IS_COMPETITION_ROBOT ? new CompetitionRobot() : new TestRobot();

}
