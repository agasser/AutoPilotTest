package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPilotTest extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    private final APConstraints apConstraints = new APConstraints()
                .withVelocity(5)
                .withAcceleration(5)
                .withJerk(2);

    private final Autopilot autopilot = new Autopilot(new APProfile()
                .withConstraints(apConstraints)
                .withErrorXY(Centimeters.of(2))
                .withErrorTheta(Degrees.of(0.5))
                .withBeelineRadius(Centimeters.of(8)));
    private final APTarget apTarget;

    private final SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5, 0, 0)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private final NetworkTable autoPilotTable = NetworkTableInstance.getDefault().getTable("AutoPilot");
    private final ProtobufPublisher<Transform2d> targetPublisher = autoPilotTable
            .getProtobufTopic("AutoPilot Output", Transform2d.proto).publish();

    public AutoPilotTest(CommandSwerveDrivetrain drivetrain, Pose2d targetPose, Rotation2d entryAngle) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        apTarget = new APTarget(targetPose).withEntryAngle(entryAngle);
    }

    @Override
    public void execute() {
        var driveState = drivetrain.getState();
        var fieldOrientedRobotSpeed =
                new Translation2d(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond)
                        .rotateBy(driveState.Pose.getRotation());

        var autoPilotSpeedAndHeading = autopilot.calculate(
                driveState.Pose,
                fieldOrientedRobotSpeed,
                apTarget);

        targetPublisher.accept(autoPilotSpeedAndHeading);

        drivetrain.setControl(swerveRequest
                .withVelocityX(autoPilotSpeedAndHeading.getX())
                .withVelocityX(autoPilotSpeedAndHeading.getY())
                .withTargetDirection(autoPilotSpeedAndHeading.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return autopilot.atTarget(drivetrain.getState().Pose, apTarget);
    }

    @Override
    public void end(boolean interrupted) {
        targetPublisher.accept(new Transform2d());
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
