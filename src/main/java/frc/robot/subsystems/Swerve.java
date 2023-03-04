package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.PIGEON_ID);
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.MOD_0.CONSTANTS),
          new SwerveModule(1, Constants.Swerve.MOD_1.CONSTANTS),
          new SwerveModule(2, Constants.Swerve.MOD_2.CONSTANTS),
          new SwerveModule(3, Constants.Swerve.MOD_3.CONSTANTS)
        };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, getYaw(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void resetRelativeEncoders() {
    for (SwerveModule swerveModule : mSwerveMods) {
      swerveModule.resetRelativeEncoders();
    }
  }

  public void setMinMax(double min, double max, int pidSlot){
    for (SwerveModule swerveModule : mSwerveMods){
      swerveModule.setMinMax(min, max, pidSlot);
    }
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
      
    }
  }

  public void setModulePositions(SwerveModulePosition[] desiredPositions) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredPosition(desiredPositions[mod.moduleNumber]);
    }
  }

  public void setModulePositions(SwerveModulePosition[] desiredPositions, int pidSlot) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredPosition(desiredPositions[mod.moduleNumber], pidSlot);
    }
  }

  public void setPID(double p, double i, double d, int pidSlot){
    for (SwerveModule mod : mSwerveMods){
      mod.setP(p, pidSlot);
      mod.setI(i, pidSlot);
      mod.setD(d, pidSlot);
    }
  }

  

  public void burnFlash(){
    for (SwerveModule mod : mSwerveMods){
      mod.burnFlash();
    }
  }
  public void runToPosition(double position, int pidSlot){
    for (SwerveModule mod : mSwerveMods){
      mod.goToPosition(position, pidSlot);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    //swerveOdometry.resetPosition(pose, getYaw());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod: mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();  // NOTE: position is already turned into meters from conversion factor
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.INVERT_GYRO)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    // swerveOdometry.update(getYaw(), getStates());
    field.setRobotPose(getPose());

    SmartDashboard.putString("Pose ", getPose().toString());
    for (SwerveModule mod : mSwerveMods) {
      String modName = "Mod " + mod.moduleNumber;
      SmartDashboard.putNumber(
          modName + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          modName + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          modName + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
        modName + " Desired", mod.getDesiredAngle());
      SmartDashboard.putNumber(
        modName + " Adj Cancoder", mod.getCanCoder().getDegrees() - mod.getAngleOffset());
      SmartDashboard.putNumber(modName + " distance Meters", mod.getPosition().distanceMeters);
      SmartDashboard.putNumber(modName + " drive motor angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(modName + " drive motor encoder", mod.getDriveEncoder());
    }
  }
}
