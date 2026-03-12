package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int[] HUB_TAG_IDS = {
        2, 3, 4, 5, 8, 9, 10, 11,
        18, 19, 20, 21, 24, 25, 26, 27
    };

    // Limelight Data table
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Basic targeting data

    // The ID of the targetted AprilTag
    NetworkTableEntry tid = limelightTable.getEntry("tid");
    // Horizontal offset from crosshair to target in degrees
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    // Target area (0% to 100% of image)
    NetworkTableEntry ta = limelightTable.getEntry("ta");
    // Valid target flag (1 = target found, 0 = no target)
    NetworkTableEntry tv = limelightTable.getEntry("tv");

    // A list of the robot's position relative to the april tag [tx, ty, tz, Pitch, Yaw, Roll]
    NetworkTableEntry botPose = limelightTable.getEntry("botpose_targetspace");
    // A  list of the robot's position relative to the field from the Blue alliance
    // [X, Y, Z, Roll, Pitch, Yaw, Latency, Tag Count, Tag Span, Avg Tag Distance, and Avg Tag Area]
    NetworkTableEntry botPoseFieldBlue = limelightTable.getEntry("botpose_wpiblue");
    // The current pipeline index
    NetworkTableEntry activePipeline = limelightTable.getEntry("getpipe");
    // Allows pipeline to be set
    NetworkTableEntry pipelineToSet = limelightTable.getEntry("pipeline");

    // Horizontal offset from principal pixel to target in degrees
    NetworkTableEntry txnc = limelightTable.getEntry("txnc");
    // Vertical offset from principal pixel to target in degrees
    NetworkTableEntry tync = limelightTable.getEntry("tync");

    // Indicates if limelight is being used
    private final boolean kUseLimelight = false;
    // RobotContainer
    private final RobotContainer m_robotContainer;

    // CONSTRUCTOR
    public LimelightSubsystem(RobotContainer m_robotContainer) {
        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
        // Let pipeline control LED behavior instead of forcing LEDs off.
        LimelightHelpers.setLEDMode_PipelineControl(LIMELIGHT_NAME);

        // The Robot Container is needed to access the Drivetrain
        this.m_robotContainer = m_robotContainer;
    }

    @Override
    public void periodic() {
        // If Limelight is in use
        if (kUseLimelight) {
            // Returns the robot's current state(position, orientation, and velocity)
            var drivebase = m_robotContainer.drivebase;
            // Gets the heading/rotation of the robot in degrees
            double headingDeg = drivebase.getPose().getRotation().getDegrees();
            // Gets the robot's angular velocity, converts from radians to rotations per second
            double omegaRps = Units.radiansToRotations(drivebase.getRobotVelocity().omegaRadiansPerSecond);
            // Initializes the Robot's Orientation
            LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);
            // Retrieves the robots pose estimation on the field from the Blue Origin using Megatag 2
            //var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            //llMeasurement = null; // using both megatag 1 and 2?? Im just keeping the 1 from CommandSwerveDrivetrain - kevin
            //if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
            //    // Adds the limelight pose estimate to the drivetrain's odometry calculation
            //    m_robotContainer.drivebase.updateVisionOdometry();
            //}
            // Posts data to SmartDashboard
            // The parameter inside getDouble or getDoubleArray is what is returned if nothing is found
            SmartDashboard.putNumber("limelight tx", tx.getDouble(0));
            SmartDashboard.putNumber("limelight yaw", getYaw());
            SmartDashboard.putNumber("limelight ta", ta.getDouble(0));
            SmartDashboard.putBoolean("Tag Detected", isTagDetected());
        }
    }
    
    // Returns the botPose list
    public double[] getBotPose(){
        return LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_NAME);
    }

    // Returns the botPoseFieldBlue list
    public double[] getBotPoseFieldBlue(){
        return LimelightHelpers.getBotPose_wpiBlue(LIMELIGHT_NAME);
    }

    // Returns the Yaw of the robot
    public double getYaw() {
        double[] botPoseValues = getBotPose();
        if (botPoseValues.length < 5) {
            return 0.0;
        }
        return botPoseValues[4];
    }

    public double getTx() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }

    public double getDistanceToTargetMeters() {
        double[] targetSpacePose = getBotPose();
        if (targetSpacePose.length < 3) {
            return Double.NaN;
        }
        double planarDistanceMeters = Math.hypot(targetSpacePose[0], targetSpacePose[2]);
        return Double.isFinite(planarDistanceMeters) ? planarDistanceMeters : Double.NaN;
    }

    public double getTa() {
        return ta.getDouble(0);
    }

    public int getTid() {
        return (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    }

    public boolean isTagDetected() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }

    public boolean isHubTagDetected() {
        if (!isTagDetected()) {
            return false;
        }
        int id = getTid();
        for (int hubTagId : HUB_TAG_IDS) {
            if (hubTagId == id) {
                return true;
            }
        }
        return false;
    }

    public double getTagCount() {
        double[] fieldPoseValues = getBotPoseFieldBlue();
        if (fieldPoseValues.length < 8) {
            return 0.0;
        }
        return fieldPoseValues[7];
    }

    // Returns a Pose2D of the robot's X, Y and Yaw relative to the field
    public Pose2d getPose() {
        double[] fieldPoseValues = getBotPoseFieldBlue();
        if (fieldPoseValues.length < 6) {
            return new Pose2d();
        }
        // Creates Rotation 2D of the robot's yaw, converts to radians
        Rotation2d rot = new Rotation2d(fieldPoseValues[5] * Math.PI / 180);
        // Creates a Pose2D of the robot's x, y, and yaw
        Pose2d out = new Pose2d(fieldPoseValues[0], fieldPoseValues[1], rot);
        return out;
    }

    public int getActivePipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(LIMELIGHT_NAME);
    }

    
    
}