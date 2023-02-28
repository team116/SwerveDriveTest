package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private static final String LIMELIGHT_HOST_NAME = "limelight";   // NOTE: getTable("limelight hostname")

    public static final int STREAM_MODE_STANDARD = 0;  // Primary and secondary cameras shown side-by-side
    public static final int STREAM_MODE_PIP_MAIN = 1;  // Picture in picture with secondary camera in lower right corner
    public static final int STREAM_MODE_PIP_SECONDARY = 2; // Picture in picture with primary camera in lower right corner

    public static final int LED_BY_CURRENT_PIPELINE = 0;
    public static final int LED_FORCE_OFF = 1;
    public static final int LED_FORCE_BLINK = 2;
    public static final int LED_FORCE_ON = 3;

    NetworkTable limelightTable;
    int streamModeToggleValue = STREAM_MODE_STANDARD;
    int currentStreamMode;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_HOST_NAME);
    }

    public void toggleStreamMode() {
        setStreamMode(currentStreamMode == STREAM_MODE_PIP_MAIN ? STREAM_MODE_PIP_SECONDARY : STREAM_MODE_PIP_MAIN);
    }

    public void setStreamMode(int newStreamMode) {
        limelightTable.getEntry("stream").setNumber(newStreamMode);
        currentStreamMode = newStreamMode;
    }

    public void setLedMode(int newLedMode) {
        limelightTable.getEntry("ledMode").setNumber(newLedMode);
    }

    public void ledOn() {
        setLedMode(LED_FORCE_ON);
    }

    public void ledOff() {
        setLedMode(LED_FORCE_OFF);
    }

    public boolean hasValidTarget(){
        return limelightTable.getEntry("ta").getDouble(0.0d) > 0.25d;
    }

    public double targetAreaPercentageOfImage(){
        return limelightTable.getEntry("ta").getDouble(0.0d);
    }

    /**
     * Returns a degree value in the range of -29.8 to 29.8 degrees, that represents how far off the
     * targeting crosshair on the limelight is horizontally away from the center of the recognized
     * target.
     * 
     * @return a degree value from -29.8 to 29.8 degrees
     */
    public double horizontalOffsetFromCrosshairAsDegrees() {
        return limelightTable.getEntry("tx").getDouble(0.0d);
    }

}
