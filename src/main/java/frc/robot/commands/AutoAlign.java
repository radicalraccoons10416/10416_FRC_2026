package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.States;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


import java.util.function.DoubleSupplier;

public class AutoAlign extends Command {

    // Swerve drivetrain used to command translation and rotation.
    private final SwerveSubsystem drivebase;
    // Limelight subsystem used to read target info (tx, distance, tag status).
    private final LimelightSubsystem limelight;
    // Shooter subsystem used to set shooter wheel speed.
    private final ShooterSubsystem shooter;
    // Hopper subsystem used to feed game pieces when shooter is up to speed.
    private final HopperSubsystem hopper;
    // Driver forward/back translation input supplier.
    private final DoubleSupplier translationXSupplier;
    // Driver left/right translation input supplier.
    private final DoubleSupplier translationYSupplier;

    // PID controller that turns the robot to reduce tx error.
    // This controller runs in radians so output is consistent with swerve rad/s rotation command.
    private final PIDController turnController = new PIDController(1.8, 0.0, 0.015);

    // How close tx (in degrees) must be to be considered aligned.
    private static final double TX_TOLERANCE_DEG = 1.0;
    // Max auto rotation command sent to swerve (rad/s).
    private static final double MAX_AUTO_ROTATION_RAD_PER_SEC = 2.0;
    // Scale factor for driver translation while auto-align is active.
    private static final double DRIVE_SCALE = 0.8;
    // Maximum lead angle allowed for moving-shot compensation (degrees).
    private static final double MAX_LEAD_DEG = 6.0;
    // Blend factor for lead compensation. 1.0 = full model, lower values reduce over-rotation while moving.
    private static final double LEAD_GAIN = 0.6;
    // Small heading error deadband to prevent jitter/spin near alignment.
    private static final double HEADING_ERROR_DEADBAND_DEG = 0.35;
    // Minimum allowed distance used in calculations to avoid divide-by-zero behavior.
    private static final double MIN_DISTANCE_METERS = 0.1;
    // Maximum plausible shooting distance; reject outliers from bad vision frames.
    private static final double MAX_DISTANCE_METERS = 10.0;
    // Extra fixed delay added to time-of-flight estimate (camera/control lag).
    private static final double EXTRA_SYSTEM_DELAY_SECONDS = 0.04;

    // Base projectile time-of-flight in the linear TOF model.
    private static final double TOF_BASE_SECONDS = 0.09;
    // Added TOF per meter of distance.
    private static final double TOF_SECONDS_PER_METER = 0.065;

    // Most recent shooter speed computed from a valid tag distance; used as fallback when tag is lost.
    private double lastShooterSpeed = 0;
    // Most recent lead setpoint computed from valid data; reused if distance is temporarily invalid.
    private double lastLeadSetpointDeg = 0.0;

    private NetworkTableEntry distanceFromHubEntry;

    private InterpolatingDoubleTreeMap speedTable;

    public AutoAlign(
        SwerveSubsystem drivebase,
        LimelightSubsystem limelight,
        ShooterSubsystem shooter,
        HopperSubsystem hopper,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier
        
    ) {
        this.drivebase = drivebase;
        this.limelight = limelight;
        this.shooter = shooter;
        this.hopper = hopper;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        turnController.setTolerance(Units.degreesToRadians(TX_TOLERANCE_DEG));
        addRequirements(drivebase, shooter, hopper);

        NetworkTable autoAlignTable = NetworkTableInstance.getDefault().getTable("autoAlign");
        distanceFromHubEntry = autoAlignTable.getEntry("DistanceFromHub");

        speedTable = new InterpolatingDoubleTreeMap();
        speedTable.put(1.0, 22.0);
        speedTable.put(2.0, 24.0);
    }

    @Override
    public void initialize() {
        turnController.reset();
        lastShooterSpeed = 0; // reset to safe default each time the command starts
        lastLeadSetpointDeg = 0.0;
    }

    @Override
    public void execute() {
        double xInput = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Constants.OperatorConstants.DEADBAND);
        double yInput = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Constants.OperatorConstants.DEADBAND);

        double xVelocity = xInput * Constants.MAX_SPEED * DRIVE_SCALE;
        double yVelocity = yInput * Constants.MAX_SPEED * DRIVE_SCALE;

        boolean tagDetected = limelight.isTagDetected();
        boolean hubTagDetected = limelight.isHubTagDetected();

        double txDeg = hubTagDetected ? limelight.getTx() : 0.0;
        double distanceMeters = hubTagDetected ? limelight.getDistanceToTargetMeters() : Double.NaN;
        boolean hasValidDistance = hubTagDetected && isDistanceValid(distanceMeters);

        if (hasValidDistance) {
            distanceFromHubEntry.setDouble(distanceMeters);

            lastShooterSpeed = mapDistanceToShooterSpeed(distanceMeters);
            System.out.println(lastShooterSpeed);
            lastLeadSetpointDeg = calculateLeadSetpointDegrees(txDeg, distanceMeters);
        }

        double shooterSpeedCmd = lastShooterSpeed;
        shooter.setShooterWheelSpeed(shooterSpeedCmd);
        hopper.setHopper(States.FORWARD, shooter::isShooterAtSpeed);

        double rotationCmd = 0.0;
        if (hubTagDetected) {
            double txRad = Units.degreesToRadians(txDeg);
            double leadSetpointRad = Units.degreesToRadians(lastLeadSetpointDeg);
            rotationCmd = -turnController.calculate(txRad, leadSetpointRad);
            double headingErrorDeg = Math.abs(txDeg - lastLeadSetpointDeg);
            if (headingErrorDeg < HEADING_ERROR_DEADBAND_DEG) {
                rotationCmd = 0.0;
            }
            rotationCmd = MathUtil.clamp(rotationCmd, -MAX_AUTO_ROTATION_RAD_PER_SEC, MAX_AUTO_ROTATION_RAD_PER_SEC);
        }

        SmartDashboard.putNumber("AutoAlign/tx", txDeg);
        SmartDashboard.putNumber("AutoAlign/txLeadSetpoint", lastLeadSetpointDeg);
        SmartDashboard.putBoolean("AutoAlign/AtHeading", hubTagDetected && turnController.atSetpoint());
        SmartDashboard.putBoolean("AutoAlign/TagDetected", tagDetected);
        SmartDashboard.putBoolean("AutoAlign/HubTagDetected", hubTagDetected);
        SmartDashboard.putBoolean("AutoAlign/HasValidDistance", hasValidDistance);
        SmartDashboard.putBoolean("AutoAlign/UsingFallbackSpeed", !hasValidDistance);
        SmartDashboard.putNumber("AutoAlign/DistanceMeters", distanceMeters);
        SmartDashboard.putNumber("AutoAlign/ShooterSpeed", shooterSpeedCmd);
        SmartDashboard.putNumber("AutoAlign/RotationCmd", rotationCmd);

        drivebase.drive(new Translation2d(xVelocity, yVelocity), rotationCmd, true);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterWheelSpeed(0.0);
        drivebase.drive(new Translation2d(), 0.0, true);
    }

    private double mapDistanceToShooterSpeed(double distanceMeters) {
        return speedTable.get(distanceMeters);
    }

    private boolean isDistanceValid(double distanceMeters) {
        return Double.isFinite(distanceMeters)
            && distanceMeters > MIN_DISTANCE_METERS
            && distanceMeters < MAX_DISTANCE_METERS;
    }

    private double calculateLeadSetpointDegrees(double txDegrees, double distanceMeters) {
        ChassisSpeeds fieldVelocity = drivebase.getFieldVelocity();
        double robotHeadingRad = drivebase.getPose().getRotation().getRadians();
        double losHeadingRad = robotHeadingRad + Units.degreesToRadians(txDegrees);

        double lateralVelocityMetersPerSecond =
            fieldVelocity.vxMetersPerSecond * -Math.sin(losHeadingRad)
                + fieldVelocity.vyMetersPerSecond * Math.cos(losHeadingRad);

        double timeOfFlightSeconds = estimateTimeOfFlightSeconds(distanceMeters);
        double leadRadians = -Math.atan2(
            lateralVelocityMetersPerSecond * timeOfFlightSeconds,
            Math.max(distanceMeters, MIN_DISTANCE_METERS)
        );

        double leadDegrees = Units.radiansToDegrees(leadRadians) * LEAD_GAIN;
        return MathUtil.clamp(leadDegrees, -MAX_LEAD_DEG, MAX_LEAD_DEG);
    }

    private double estimateTimeOfFlightSeconds(double distanceMeters) {
        return TOF_BASE_SECONDS
            + (TOF_SECONDS_PER_METER * distanceMeters)
            + EXTRA_SYSTEM_DELAY_SECONDS;
    }
}