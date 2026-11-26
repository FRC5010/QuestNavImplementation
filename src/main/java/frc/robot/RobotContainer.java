// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.Utility;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private final SendableChooser<Command> autoChooser =
            new SendableChooser<>();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain);

    /** Auto chooser with one entry per BLine path file, plus the original drive-forward auto. */
    public final BLineAutos autos;

    private void configureAutos() {
        autoChooser.setDefaultOption(
            "Do Nothing",
            Commands.none()
        );

        autoChooser.addOption(
            "driveForwardAuto",
            driveForwardAuto()
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public RobotContainer() {
        configureBindings();
        configureAutos();

        // Two builders, because BLine resets the robot's pose at the start of EVERY path built
        // with withPoseReset (see the "Multi-step autos" notes above twoStepExampleAuto).
        //   firstPathBuilder: for the first path of an auto. Resets the pose to the path's start,
        //                     which is where we placed the robot on the field.
        //   laterPathBuilder: for every path after that. Never resets the pose.
        FollowPath.Builder firstPathBuilder = newPathBuilder().withPoseReset(drivetrain::resetPose);
        FollowPath.Builder laterPathBuilder = newPathBuilder();

        // One chooser entry per path file (each built with firstPathBuilder) ...
        autos = new BLineAutos(firstPathBuilder, driveForwardAuto(), drivetrain::getPose);
        // ... plus the hand-built multi-step autos.
        autos.add(TWO_STEP_EXAMPLE_AUTO, twoStepExampleAuto(firstPathBuilder, laterPathBuilder));
    }

    /**
     * A path-following builder with our controller gains, but no pose reset. Each call makes a new
     * builder so adding withPoseReset to one does not change the other.
     */
    private FollowPath.Builder newPathBuilder() {
        return new FollowPath.Builder(
            drivetrain,
            drivetrain::getPose,
            drivetrain::getRobotRelativeSpeeds,
            drivetrain::driveRobotRelative,
            new PIDController(2.0, 0.0, 0.0), // translation
            // Rotation: P-only stops where its output drops below drive friction. 1.0 stalled ~7 deg short
            // in sim and 3.0 ~2.01 deg short (2025-l-shape), outside the 2 deg end tolerance, so autos never ended.
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(0.2, 0.0, 0.0)  // cross-track
        ).withDefaultShouldFlip()
         .withTRatioBasedTranslationHandoffs(true);
    }

    /** Chooser name of {@link #twoStepExampleAuto}. */
    public static final String TWO_STEP_EXAMPLE_AUTO = "2025 Two-Step Example";

    /*
     * Multi-step autos: how to make the robot wait for an action
     * -----------------------------------------------------------
     * 1. A BLine path never stops partway. Waypoints in the middle of a path are pass-through:
     *    the robot rounds the corner and keeps going. It only stops at the LAST point of a path.
     *
     * 2. So if the robot must stop and do something (score, pick up), END the path there.
     *    Then put the action and the next path after it in a Commands.sequence, as below.
     *    The sequence runs one step at a time: a step starts only when the one before it ends.
     *
     * 3. The action decides how long the robot waits. Make it END when the job is really done
     *    (a sensor sees the piece is gone, the elevator is at height), not after a guessed time.
     *    A fixed wait is too long on a fresh battery and too short on a tired one.
     *    Always add .withTimeout(seconds): if a sensor never reports, the auto skips one step
     *    instead of sitting at the reef until the end of the match.
     *
     * 4. Things that should run DURING the drive (intake, spinning up a shooter): run them
     *    alongside the path, for example
     *        laterPathBuilder.build(path).deadlineFor(intake.run())
     *    The path is the "deadline": the intake runs while the robot drives and is stopped when
     *    the path ends.
     *    BLine event triggers (FollowPath.registerEventTrigger) can also start a command at a
     *    point along a path, but BLine only schedules it and keeps driving. That command is not
     *    part of the auto: nothing waits for it, it is NOT stopped when the path or the auto ends,
     *    and it can fight the auto for a subsystem. Never use a trigger for something the next
     *    step depends on.
     *
     * 5. The pose-reset trap. Only the FIRST path may use firstPathBuilder. If a later path reset
     *    the pose, the robot would "teleport" its estimate to that path's start (position only;
     *    BLine keeps the current heading), wherever it really is, and every error from earlier
     *    paths would be hidden. In simulation it is worse: a pose
     *    reset also moves the simulated robot, so a wrong auto would still look perfect.
     *
     * 6. Each path should start where the previous one ended. The next path starts from wherever
     *    the robot really is, so a gap shows up as a sharp correction at the start of the path.
     *    Build the paths in the BLine GUI by continuing from the end point.
     */

    /**
     * Example multi-step auto: drive to the reef, "score", then back out. Copy this pattern for
     * real autos. Blue side; BLine flips both paths for red.
     */
    private Command twoStepExampleAuto(FollowPath.Builder firstPathBuilder, FollowPath.Builder laterPathBuilder) {
        return Commands.sequence(
            // Step 1: first path of the auto, so it resets the pose to its start.
            // Ends stopped in front of reef face tag 21 at (5.9, 4.021).
            firstPathBuilder.build(new Path("2025-start-to-reef-face")),

            // Step 2: the action. The robot is stopped at the reef while this runs.
            // On a real robot this is the scoring command, ending on a sensor, with a timeout:
            //     scorer.score().until(() -> !scorer.hasCoral()).withTimeout(1.5)
            // This sandbox has no mechanisms, so a fixed 1 s wait stands in for scoring.
            Commands.waitSeconds(1.0).withName("Score (stand-in)"),

            // Step 3: a later path, so it does NOT reset the pose. It starts at (5.9, 4.021),
            // exactly where step 1 ended.
            laterPathBuilder.build(new Path("2025-reef-face-back-out"))
        ).withName(TWO_STEP_EXAMPLE_AUTO);
    }

    /** Call every robot loop to keep the sim/AdvantageScope field view up to date. */
    public void periodic() {
        autos.periodic();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Utility.controllerDeadband(-joystick.getLeftY(), 0.07) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(Utility.controllerDeadband(-joystick.getLeftX(), 0.2) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(Utility.controllerDeadband(-joystick.getRightX(), 0.07) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.rightBumper().whileTrue(questNavSubsystem.determineOffsetToRobotCenter(drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }

    private Command driveForwardAuto() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

}
