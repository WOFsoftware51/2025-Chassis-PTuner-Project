// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autons.BUMP_HP;
import frc.robot.Autons.OSMOSIS;
import frc.robot.Autons.S_C_S;
import frc.robot.Autons.S_C_S_LEFT;
import frc.robot.Autons.Shoot8;
import frc.robot.Autons.Test;
import frc.robot.commands.MoveToAngle;
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
import frc.robot.subsystems.spindexer.SpindexerIOHardware;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretIOHardware;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionChassisSubsystem;
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionTurretSubsystem;
import frc.robot.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// Optional: If you need wait until a specific time in the match
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
    // private final CommandPS5Controller driver = new CommandPS5Controller(0);
    // private final CommandPS5Controller operator = new CommandPS5Controller(1);
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

    
    private boolean operatorRumble = false;

    LoggedTunableNumber speedLeft = new LoggedTunableNumber("Pose/speedLeft", 5);
    LoggedTunableNumber speedRight = new LoggedTunableNumber("Pose/speedRight", 5);
    LoggedTunableNumber speedForward = new LoggedTunableNumber("Pose/speedLeft", 1);
    LoggedTunableNumber speedBackward = new LoggedTunableNumber("Pose/speedRight", 1);
    
    LoggedTunableNumber pivotAngle = new LoggedTunableNumber("Shooting/Pivot", 0.0);
    LoggedTunableNumber turretAngle = new LoggedTunableNumber("Shooting/Turret", 0.0);

    //  HashMap<String, Command> eventMap = new HashMap<>();

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
        
         
        // eventMap.put("IntakePivot",intakePivot.runVolts(6));
        // eventMap.put("Shoot", shooter.treeMapRPMCommand());
        // eventMap.put("Pivot", pivot.treeMapRPMCommand());
        // eventMap.put("IntakePivotUp", intakePivot.runVolts(-6));
        // eventMap.put("Intake", intake.runVolts(10.56));


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
            // driver.y().whileTrue(
            //     swerve.applyRequest(() ->
            //         poseTuning.withVelocityX(speedForward.get())
            //             .withVelocityY(0) 
            //             .withRotationalRate(0)
            //     )
            // );  
            // driver.a().whileTrue(
            //     swerve.applyRequest(() ->
            //         poseTuning.withVelocityX(-speedBackward.get()) // Drive forward with negative Y (forward)
            //             .withVelocityY(0) // Drive left with negative X (left)
            //             .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            //     )
            // );
            // driver.leftTrigger().whileTrue(swerve.applyRequest(() -> brake));
            // driver.y().whileTrue(swerve.applyRequest(() ->
            //     point.withModuleDirection(new Rotation2d(0))
            // ));
            // driver.rightBumper().whileTrue(
            //     new GoToPositionCommand(swerve, robotState, new Pose2d(new Translation2d(2, 4), 
            //     new Rotation2d(Units.degreesToRadians(-90))), 1));

            driver.rightBumper().whileTrue(
            // driver.R1().whileTrue(
                new MoveToAngle(
                    swerve, 
                    robotState, 
                    robotState.getPose2d(),
                    () -> robotState.justinTurretAngle(),
                    // () -> robotState.getRobotToAllianceHubDegrees(),
                    // () -> robotState.getTurretToAllianceHubDegrees(),
                    1
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
            
            driver.start().onTrue(swerve.runOnce(() -> swerve.resetPose(new Pose2d())));
            // driver.touchpad().onTrue(swerve.runOnce(() -> swerve.resetPose(new Pose2d())));

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
            operator.leftTrigger().whileTrue(shooter.runRPMCommand(3500));


            operator.rightTrigger().whileTrue(shooter.treeMapRPMCommand());
            // operator.R2().whileTrue(shooter.treeMapRPMCommand());
            
            new Trigger(() -> shooter.atRPM).onTrue(
                Commands.runOnce(() ->
                    operator.setRumble(RumbleType.kBothRumble, 0.5)
                )
            )
            .onFalse(
                Commands.runOnce(() ->
                    operator.setRumble(RumbleType.kBothRumble, 0.0)
                )
            );

        /*
        Pivot Controls
        */
            pivot.setDefaultCommand(pivot.runToPositionCommand(0));
            // pivot.setDefaultCommand(pivot.treeMapRPMCommand());
            // operator.y().whileTrue(pivot.runVolts(6));
            // operator.a().whileTrue(pivot.runVolts(-6));
            operator.rightTrigger().whileTrue(pivot.treeMapRPMCommand());
            operator.leftTrigger().whileTrue(pivot.runToPositionCommand(15));
            // operator.povUp().whileTrue(pivot.resetEncoder());
        
        /*
        Feeder Controls
        */
            // operator.leftTrigger().whileTrue(feeder.runFeederVoltsCommand(12));
            // operator.L2().whileTrue(feeder.runFeederVoltsCommand(12));

            new Trigger(() -> shooter.atRPM).whileTrue(feeder.runFeederVoltsCommand(12));

        /*
        Spindexer Controls
        */
            // operator.leftTrigger().whileTrue(spindexer.runSpindexerVoltsCommand(12));
            // operator.L2().whileTrue(spindexer.runSpindexerVoltsCommand(12));

            new Trigger(() -> shooter.atRPM).whileTrue(spindexer.runSpindexerVoltsCommand(12));


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*
        Intake
        */
            driver.leftBumper().whileTrue(intake.runVolts(10.56));
            // driver.L1().whileTrue(intake.runVolts(12));
        
        /*
        Intake Pivot
        */
            // intakePivot.setDefaultCommand(intakePivot.runVoltsJoystick(() -> operator.getLeftY()));
            operator.leftBumper().whileTrue(intakePivot.runVolts(-6));
            operator.rightBumper().whileTrue(intakePivot.runVolts(6));
            // operator.L1().whileTrue(intakePivot.runVolts(-3));
            // operator.L2().whileTrue(intakePivot.runVolts(3));
            // operator.povUp().whileTrue(intakePivot.resetEncoder());


        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));
       

        swerve.registerTelemetry(logger::telemeterize);


    NamedCommands.registerCommand("Shoot",shooter.treeMapRPMCommand().withTimeout(2.5));
    NamedCommands.registerCommand("Pivot",pivot.treeMapRPMCommand().withTimeout(2.5));
    NamedCommands.registerCommand("IntakePivot",intakePivot.runVolts(6).withTimeout(0.7));
    NamedCommands.registerCommand("IntakePivotUp", intakePivot.runVolts(-6).withTimeout(0.53));
    NamedCommands.registerCommand("Intake", intake.runVolts(10.56));
    NamedCommands.registerCommand("ShootMore",shooter.treeMapRPMCommand().withTimeout(5));
    NamedCommands.registerCommand("PivotMore",pivot.treeMapRPMCommand().withTimeout(5));
    

    // NamedCommands.registerCommand("Spin", spindexer.runSpindexerVoltsCommand(12.0));
    // NamedCommands.registerCommand("Feed", feeder.runFeederVoltsCommand(12));

 


    }

    public void printAutons(){
        SmartDashboard.putData("Auton", a_chooser);
        a_chooser.setDefaultOption("test", 1);
        a_chooser.addOption("test", 1);
        a_chooser.addOption("Shoot8", 2);
        a_chooser.addOption("S_C_S", 3);
        a_chooser.addOption("S_C_S_LEFT", 4);
        a_chooser.addOption("BUMP_HP", 5);
        a_chooser.addOption("OSMOSIS", 6);

    }


    public Command getAutonomousCommand() {
        switch (a_chooser.getSelected()) {
            case 1:
                return new Test(swerve, robotState, shooter, feeder, spindexer, pivot);
            case 2:
                return new Shoot8(spindexer,feeder,shooter);
            case 3:
                return new S_C_S(spindexer, feeder, shooter, intake, intakePivot, pivot);
            case 4:
                return new S_C_S_LEFT(spindexer, feeder, shooter, intake, intakePivot, pivot);
            case 5:
                return new BUMP_HP(spindexer, feeder, shooter, intake, intakePivot, pivot);
            case 6:
                return new PathPlannerAuto("OSMOSIS");
            default:
                 return new Shoot8(spindexer,feeder,shooter);

        }
    }  
}
