package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName;

    // Vision targeting constants
    private static final double AIMING_KP = 0.035; // Proportional constant for aiming (rotation)
    private static final double RANGING_KP = 0.1;  // Proportional constant for ranging (forward/back)

    // Targeting tolerances
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
    private static final double RANGE_TOLERANCE_DEGREES = 1.0;

    // AprilTag pose estimation constants
    private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0; // Reject vision updates above this

    public VisionSubsystem(String limelightName) {
        this.limelightName = limelightName;
    }

    public VisionSubsystem() {
        this("limelight"); // Default limelight name
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Gets the horizontal offset from the crosshair to the target
     * @return Horizontal offset in degrees
     */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    /**
     * Gets the vertical offset from the crosshair to the target
     * @return Vertical offset in degrees
     */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /**
     * Checks if the limelight has a valid target
     * @return true if target is detected
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Gets the target area (0%-100% of image)
     * @return Target area percentage
     */
    public double getTargetArea() {
        return LimelightHelpers.getTA(limelightName);
    }

    /**
     * Calculates the rotation speed for aiming at target
     * Uses proportional control: output = kP * error
     * @return Rotation speed in radians per second
     */
    public double getAimingRotationSpeed() {
        if (!hasTarget()) {
            return 0.0;
        }

        // tx is the horizontal offset in degrees
        // Negative because we want to rotate opposite to the error
        double tx = getTX();
        return -tx * AIMING_KP;
    }

    /**
     * Calculates the forward speed for ranging to target
     * Uses proportional control: output = kP * error
     * @return Forward speed in meters per second
     */
    public double getRangingForwardSpeed() {
        if (!hasTarget()) {
            return 0.0;
        }

        // ty is the vertical offset in degrees
        // Negative because we want to move opposite to the error
        double ty = getTY();
        return -ty * RANGING_KP;
    }

    /**
     * Checks if robot is aimed at target within tolerance
     * @return true if aimed at target
     */
    public boolean isAimedAtTarget() {
        if (!hasTarget()) {
            return false;
        }
        return Math.abs(getTX()) < AIM_TOLERANCE_DEGREES;
    }

    /**
     * Checks if robot is at proper range from target
     * @return true if at proper range
     */
    public boolean isAtTargetRange() {
        if (!hasTarget()) {
            return false;
        }
        return Math.abs(getTY()) < RANGE_TOLERANCE_DEGREES;
    }

    /**
     * Sets the LED mode of the limelight
     * @param mode 0=pipeline default, 1=force off, 2=force blink, 3=force on
     */
    public void setLEDMode(int mode) {
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
        if (mode == 1) {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        } else if (mode == 2) {
            LimelightHelpers.setLEDMode_ForceBlink(limelightName);
        } else if (mode == 3) {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        } else {
            LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        }
    }

    /**
     * Sets the pipeline index
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    /**
     * Gets the current pipeline index
     * @return Current pipeline index
     */
    public double getCurrentPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    /**
     * Calculates strafe speed to center target horizontally
     * Uses tx (horizontal offset) to strafe left/right while maintaining forward facing
     * @return Strafe speed in meters per second (positive = left, negative = right)
     */
    public double getStrafeSpeed() {
        if (!hasTarget()) {
            return 0.0;
        }

        // tx is positive when target is to the right
        // Strafe left (positive) when target is right to center it
        double tx = getTX();
        return tx * AIMING_KP;
    }

    // ==================== APRILTAG POSE ESTIMATION ====================

    /**
     * Gets MegaTag2 pose estimate from Limelight
     * MegaTag2 uses gyro fusion for better accuracy
     * @param currentYawDegrees Current robot yaw from gyro (CCW positive, 0 = facing red alliance wall)
     * @return PoseEstimate containing robot pose, timestamp, and tag count
     */
    public PoseEstimate getMegaTag2Estimate(double currentYawDegrees) {
        // Tell Limelight the robot's current orientation for MegaTag2 fusion
        LimelightHelpers.SetRobotOrientation(limelightName, currentYawDegrees, 0, 0, 0, 0, 0);

        // Get MegaTag2 pose estimate (uses WPILib blue origin coordinate system)
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    /**
     * Gets the number of AprilTags currently visible
     * @return Number of tags detected (0 if no tags)
     */
    public int getTagCount() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        return estimate.tagCount;
    }

    /**
     * Checks if vision update should be rejected based on filtering criteria
     * @param angularVelocityDegPerSec Current angular velocity in degrees per second
     * @param tagCount Number of tags currently visible
     * @return true if update should be rejected
     */
    public boolean shouldRejectVisionUpdate(double angularVelocityDegPerSec, int tagCount) {
        // Reject if spinning too fast (motion blur)
        if (Math.abs(angularVelocityDegPerSec) > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            return true;
        }

        // Reject if no tags visible
        if (tagCount == 0) {
            return true;
        }

        return false;
    }

    /**
     * Calculates vision measurement standard deviations based on tag count
     * More tags = more reliable = lower standard deviation
     * @param tagCount Number of tags currently visible
     * @return Standard deviation vector [x, y, theta]
     */
    public Vector<N3> getVisionStdDevs(int tagCount) {
        if (tagCount >= 2) {
            // Multiple tags: High confidence
            // X, Y: ±0.5m, Theta: IGNORE (use gyro instead)
            return VecBuilder.fill(0.5, 0.5, 9999999);
        } else if (tagCount == 1) {
            // Single tag: Lower confidence
            // X, Y: ±0.9m, Theta: IGNORE (use gyro instead)
            return VecBuilder.fill(0.9, 0.9, 9999999);
        } else {
            // No tags: Very low confidence (shouldn't be used)
            return VecBuilder.fill(10, 10, 9999999);
        }
    }
}
