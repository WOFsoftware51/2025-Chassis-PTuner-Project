// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Autons.test;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.commands.GoToPositionCommand;
import frc.robot.commands.MoveToAngle;
import frc.robot.commands.PivotPoseDefaultCommand;
import frc.robot.commands.TurretCameraDefaultCommand;
import frc.robot.commands.TurretCameraPoseDefaultCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.FeederIOHardware;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotIOHardware;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOHardware;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretIOHardware;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionTurretSubsystem;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.vision.VisionChassisSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity: 0.8435211984 RPS

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric poseTuning = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController joystick = new CommandXboxController(3);
    private final CommandXboxController test = new CommandXboxController(5);

    private final SendableChooser<Integer> a_chooser = new SendableChooser<>();


    private final VisionTurretSubsystem limelightTurret;
    private final VisionChassisSubsystem limelightChassis;

    public final Swerve swerve;
    public final RobotState robotState = RobotState.getInstance();
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final PivotSubsystem pivot;
    private final FeederSubsystem feeder;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;
    private final IntakePivotSubsystem intakePivot;

    

    LoggedTunableNumber speedLeft = new LoggedTunableNumber("Pose/speedLeft", 5);
    LoggedTunableNumber speedRight = new LoggedTunableNumber("Pose/speedRight", 5);
    LoggedTunableNumber speedForward = new LoggedTunableNumber("Pose/speedLeft", 1);
    LoggedTunableNumber speedBackward = new LoggedTunableNumber("Pose/speedRight", 1);
    
    LoggedTunableNumber pivotAngle = new LoggedTunableNumber("Shooting/Pivot", 0.0);
    LoggedTunableNumber turretAngle = new LoggedTunableNumber("Shooting/Turret", 0.0);

    public RobotContainer() {
        this.intakePivot = new IntakePivotSubsystem(
            
        );
        this.limelightTurret = new VisionTurretSubsystem(
            //  new VisionIOHardware(Constants.VisionConstants.kTurretLimelight)
            Robot.isReal() ? new VisionIOHardware(Constants.VisionConstants.kTurretLimelight) : new VisionIOSim()
        );

        this.spindexer = new SpindexerSubsystem(
            Robot.isReal() ? new SpindexerIOHardware() : new SpindexerIOSim()
        );

        this.feeder = new FeederSubsystem(
            Robot.isReal() ? new FeederIOHardware() : new FeederIOSim()
        );

        this.pivot = new PivotSubsystem(
            Robot.isReal() ? new PivotIOHardware() : new PivotIOSim(), 
            robotState
        );
        
        this.limelightChassis = new VisionChassisSubsystem(
            Robot.isReal() ? new VisionIOHardware(Constants.VisionConstants.kChassisLimelight) : new VisionIOSim()
        );

        this.turret = new TurretSubsystem(
            Robot.isReal() ? new TurretIOHardware() : new TurretIOSim(),
            limelightTurret, 
            robotState
        );

        this.swerve = TunerConstants.createDrivetrain(limelightTurret);

        this.intake = new IntakeSubsystem(
            Robot.isReal() ? new IntakeIOHardware() : new IntakeIOSim(swerve.mapleSimSwerveDrivetrain.mapleSimDrive)
        );

        this.shooter = new ShooterSubsystem(
            Robot.isReal() ? new ShooterIOHardware() : new ShooterIOSim(this.swerve.mapleSimSwerveDrivetrain.mapleSimDrive)
        );
        

        configureBindings();
        printAutons();
    }

    private void configureBindings() {
        /*
        Swerve Controls
        */
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            swerve.setDefaultCommand(
                // Drivetrain will execute this command periodically
                swerve.applyRequest(() ->
                    drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
            driver.y().whileTrue(
                swerve.applyRequest(() ->
                    poseTuning.withVelocityX(speedForward.get())
                        .withVelocityY(0) 
                        .withRotationalRate(0)
                )
            );  
            driver.a().whileTrue(
                swerve.applyRequest(() ->
                    poseTuning.withVelocityX(-speedBackward.get()) // Drive forward with negative Y (forward)
                        .withVelocityY(0) // Drive left with negative X (left)
                        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
                )
            );
            // driver.leftTrigger().whileTrue(swerve.applyRequest(() -> brake));
            // driver.y().whileTrue(swerve.applyRequest(() ->
            //     point.withModuleDirection(new Rotation2d(0))
            // ));
            // driver.rightBumper().whileTrue(
            //     new GoToPositionCommand(swerve, robotState, new Pose2d(new Translation2d(2, 4), 
            //     new Rotation2d(Units.degreesToRadians(-90))), 1));
            driver.leftBumper().whileTrue(
                new MoveToAngle(
                    swerve, 
                    robotState, 
                    robotState.getPose2d(),
                    () -> robotState.getRobotToRedHubDegrees(),
                    1
                )
                .finallyDo(() -> 
                    operator.setRumble(RumbleType.kBothRumble, 0.5)
                )
            );

            // driver.leftBumper().whileTrue(
            //     new GoToPositionCommand(
            //         swerve, 
            //         robotState, 
            //         () -> new Pose2d(robotState.getPose2d().getTranslation(), new Rotation2d(robotState.getRobotToRedHubDegrees())),
            //         1
            //     )
            // );

            new Trigger(() -> swerve.testConfigsChanged).onTrue(swerve.setDriveGains());

            // Reset the field-centric heading on left bumper press.
            // driver.back().onTrue(swerve.runOnce(swerve::seedFieldCentric));
            driver.povDown().onTrue(swerve.runOnce(() -> swerve.resetPose(new Pose2d())));

        /*
        Turret Controls
        */
            // turret.setDefaultCommand(new TurretCameraPoseDefaultCommand(turret));
            // turret.setDefaultCommand(turret.TurretToSetpointCommand(Degrees.of(turretAngle.get())));
            // operator.b().whileTrue(turret.TurretRunWithVolts(Volts.of(-3))); //To the right
            // operator.x().whileTrue(turret.TurretRunWithVolts(Volts.of(3))); //To the left
            // operator.a().whileTrue(turret.TurretToSetpointCommand(Degrees.of(0))); 
            // operator.povUp().whileTrue(turret.resetEncoder()); 

        /*
        Shooter Controls
        */
            new Trigger(() -> shooter.gainsChanged).whileTrue(shooter.updateGainsCommand());
            // operator.rightTrigger().whileTrue(shooter.runRPMCommand());
            driver.rightTrigger().whileTrue(shooter.treeMapRPMCommand());

        /*
        Pivot Controls
        */
            // pivot.setDefaultCommand(new PivotPoseDefaultCommand(pivot));
            pivot.setDefaultCommand(pivot.treeMapRPMCommand());
            // operator.y().whileTrue(pivot.runVolts(6));
            // operator.a().whileTrue(pivot.runVolts(-6));
            // operator.rightBumper().whileTrue(pivot.runToPositionCommand(10));
            operator.povUp().whileTrue(pivot.resetEncoder());
        
        /*
        Feeder Controls
        */
            driver.leftTrigger().whileTrue(feeder.runFeederVoltsCommand(12));

        /*
        Spindexer Controls
        */
            driver.leftTrigger().whileTrue(spindexer.runSpindexerVoltsCommand(12));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*
        Intake
        */
            driver.rightBumper().whileTrue(intake.runVolts(12));
        
        /*
        Intake Pivot
        */
            // intakePivot.setDefaultCommand(intakePivot.runVoltsJoystick(() -> operator.getLeftY()));
            operator.y().whileTrue(intakePivot.runVolts(-3));
            operator.a().whileTrue(intakePivot.runVolts(3));
            // operator.povUp().whileTrue(intakePivot.resetEncoder());


        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));


        swerve.registerTelemetry(logger::telemeterize);




    }

    public void printAutons(){
        SmartDashboard.putData("Auton", a_chooser);
        a_chooser.setDefaultOption("test", 1);
        a_chooser.addOption("test", 1);
    }


    public Command getAutonomousCommand() {
        switch (a_chooser.getSelected()) {
            case 1:
                return new test(swerve, robotState);
                
            default:
                return new test(swerve, robotState);

        }
    }  
}
