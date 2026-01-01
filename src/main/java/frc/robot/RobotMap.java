package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.SwerveModuleStatusUtil;

public class RobotMap {

    public static class SafetyMap {
        public static class AutonConstraints {
            public static final double kMaxSpeed = 1.0;
            public static final double kMaxAcceleration = 3.0;
            public static final double kMaxAngularRate = Units.degreesToRadians(0);
            public static final double kMaxAngularAcceleration = Units.degreesToRadians(360);
            public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeed, kMaxAcceleration,
                    kMaxAngularRate, kMaxAngularAcceleration);
        }

        public static class SwerveConstants {
            public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(2).in(RadiansPerSecond);
            public static final double L4_SLEW_RATE = 3;
            public static final double BARGE_SPEED_MULTIPLIER = 0.15;
        }
    }

    public static class DrivetrainConstants{
        private static final int kFrontLeftDriveMotorId = 11;
        private static final int kFrontLeftSteerMotorId = 12;
        private static final int kFrontLeftEncoderId = 13;
        private static final int kFrontRightDriveMotorId = 21;
        private static final int kFrontRightSteerMotorId = 22;
        private static final int kFrontRightEncoderId = 23;
        private static final int kBackLeftDriveMotorId = 32;
        private static final int kBackLeftSteerMotorId = 31;
        private static final int kBackLeftEncoderId = 33;
        private static final int kBackRightDriveMotorId = 41;
        private static final int kBackRightSteerMotorId = 42;
        private static final int kBackRightEncoderId = 43;

        public static CommandSwerveDrivetrain createDrivetrain(){
            SwerveModuleStatusUtil.addSwerveModule(SwerveModuleStatusUtil.ModuleLocation.FL, kFrontLeftDriveMotorId, kFrontLeftSteerMotorId, kFrontLeftEncoderId);
            SwerveModuleStatusUtil.addSwerveModule(SwerveModuleStatusUtil.ModuleLocation.FR, kFrontRightDriveMotorId, kFrontRightSteerMotorId, kFrontRightEncoderId);
            SwerveModuleStatusUtil.addSwerveModule(SwerveModuleStatusUtil.ModuleLocation.BL, kBackLeftDriveMotorId, kBackLeftSteerMotorId, kBackLeftEncoderId);
            SwerveModuleStatusUtil.addSwerveModule(SwerveModuleStatusUtil.ModuleLocation.BR, kBackRightDriveMotorId, kBackRightSteerMotorId, kBackRightEncoderId);
        
            return TunerConstants.createDrivetrain();
        }
    }

    // CAN IDs for Swerve Drive System
    public static class SwerveMap {
        public static final int FRONT_LEFT_STEER = 0;
        public static final int FRONT_RIGHT_STEER = 1;
        public static final int BACK_LEFT_STEER = 2;
        public static final int BACK_RIGHT_STEER = 3;
        public static final int FRONT_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 5;
        public static final int BACK_LEFT_DRIVE = 6;
        public static final int BACK_RIGHT_DRIVE = 7;
        public static final double FAST_BACKWARDS_SPEED = -1.2;
        public static final double BACKWARDS_SPEED = -0.6;
    }

    public static class ElevatorMap {
        public static final int ELEVATOR_LEADER = 61;
        public static final int ELEVATOR_FOLLOWER = 62;
        public static final int LIMIT_SWITCH_CHANNEL = 9;
        public static final double ELEVATOR_P = .53; //.53
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0;
        public static final double ELEVATOR_G = 0.6; ///.5
        public static final double ELEVATOR_S = 0.245; //.245
        public static final double ELEVATOR_CURRENT_LIMIT = 50.0;
        public static final double ELEVATOR_LIMIT_SWITCH_HEIGHT = 0.45;
        public static final double CORAL_INTAKE_ROTATION = 1.33;
        public static final double ALGAE_GROUND_INTAKE_ROTATION = 1.67;
        public static final double L1ROTATION = 4.472;
        public static final double LOWALGAEROTATION = 8.0;
        public static final double HIGHALGAEROTATION = 15.5;
        public static final double L2ROTATION = 8.3;
        public static final double L3ROTATION = 15.7;
        public static final double L4ROTATION = 28.48;
        public static final double BARGEROTATION = 31;
    }

    public static class ClimberMap {
        public static final int CLIMBER_LEADER_MOTOR = 51;
        public static final int CLIMBER_FOLLOWER_MOTOR = 52;
        public static final int CLIMBER_CANCODER = 19;
        public static final int CLIMBER_SERVO = 9;
        public static final int CLIMBER_ENCODER = 53;
        public static final int CLIMBER_LEFT_DI = 1;
        public static final int CLIMBER_RIGHT_DI = 2;
        public static final int CLIMBER_CURRENT_LIMIT = 40;
        public static final double CLIMBER_PID_TOLERANCE = 3;
        public static final double CLIMBER_UP_ANGLE = 90;
        public static final double CLIMBER_STOW_ANGLE = 0;
        public static final PIDController climberPID = new PIDController(0, 0, 0);
        public static final double GEAR_RATIO = 48.6111;

        ClimberMap() {
            climberPID.setTolerance(CLIMBER_PID_TOLERANCE);
        }
    }

    // Additional motor controllers or sensors could be added here
    public static class SensorMap {
        // Example: Add sensor ports (like encoders, gyros, etc.)
        public static final int GYRO_PORT = 0;
        public static final int INTAKE_IR_SENSOR = 2;

    }

    // You can add more mappings for other subsystems like intake, shooter, etc.

    public static class VisionMap {

        public static final double ballRadius = 9; // cm ; 3.5 inches
        public static final double targetHeight = 1.6; // m ; 38.7 inches
        public static final double cameraHeight = .6; // m ; 16 inches
        public static final double cameraAngle = 40; // degrees

        public static class CameraConfig {
            public static class BackCam {
                public static final int CAMERA_HEIGHT = 480;
                public static final int CAMERA_WIDTH = 640;
                public static final double TARGET_HEIGHT = 0.0;
                public static final double HORIZONTAL_FOV = 59.6;
                public static final double VERTICAL_FOV = 45.7;

            }

            public static class FrontCam {
                public static final int CAMERA_HEIGHT = 480;
                public static final int CAMERA_WIDTH = 640;
                public static final double TARGET_HEIGHT = 0.0;
                public static final double HORIZONTAL_FOV = 59.6;
                public static final double VERTICAL_FOV = 45.7;
            }

            public static double tx;
            public static double ty;
            public static double ta;
            public static double distance;
        }

    }

    
       

        public static class GooseneckConstants {
            public static final double PROCESSORANGLE = 0.0966;
            public static final double L4ANGLE = .113;
            public static final double STANDARDANGLE = .04; 
            public static final double ALGAERETRIEVEANGLE = 0.117;
            public static final double ALGAEGROUNDANGLE = 0.22;
            public static final double INTAKEANGLE = .016;
            public static final double BARGEANGLE = .022;
            public static final double L1ANGLE = .01;
            public static final double tolerance = 0.007;
            public static final int INTAKE_MOTOR = 59;
            public static final int PIVOT_MOTOR = 58;
            public static final int INTAKE_MOTOR_CURRENT_LIMIT = 40;
            public static final int PIVOT_MOTOR_CURRENT_LIMIT = 40;
            public static final double WHEEL_SPEED_INTAKE = -0.2;
            public static final double WHEEL_SPEED_BACKTAKE = 0.2;
            public static final double WHEEL_SPEED_SCORE = -0.4;
            public static final double ALGAE_WHEEL_SPEED = -.8;
            public static final double ALGAE_WHEEL_SPEED_SCORE = .8;
            public static final double L1_SPEED_SCORE = -0.175;
            public static final Current ALGAE_HOLD_CURRENT = Current.ofBaseUnits(0, edu.wpi.first.units.Units.Amps); // tune
            public static final double CANRANGETHRESHOLDVALUE = 0.15;
            public static final double SENSORTOMECHRATIO = 55.22727273;
            public static final int CORALCANRANGE = 57;
        }

       

        public static TalonFXConfiguration getBreakConfigurationGooseNeck() {
            TalonFXConfiguration configuration = new TalonFXConfiguration();
            configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            configuration.Feedback.SensorToMechanismRatio = 55.22727273;
            return configuration;
        }

        public static TalonFXConfiguration getCoastConfiguration() {
            TalonFXConfiguration configuration = new TalonFXConfiguration();
            configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            return configuration;
        }

    public static class CurrentLimiter {
        public static CurrentLimitsConfigs getCurrentLimitConfiguration(double amps) {
            CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

            configs.StatorCurrentLimit = amps;
            configs.StatorCurrentLimitEnable = true;

            return configs;
        }
    }

    public static class AutonPosesMap {
        public static final Pose2d right16 = new Pose2d(4.953, 5.185, Rotation2d.fromDegrees(-120));
        public static final Pose2d left16 = new Pose2d(5.226, 5.058, Rotation2d.fromDegrees(-120));
        public static final Pose2d right26 = new Pose2d(5.704, 4.152, Rotation2d.fromDegrees(180));
        public static final Pose2d left26 = new Pose2d(5.713, 3.879, Rotation2d.fromDegrees(180));
        public static final Pose2d right36 = new Pose2d(5.245, 3.03, Rotation2d.fromDegrees(120));
        public static final Pose2d left36 = new Pose2d(4.953, 2.874, Rotation2d.fromDegrees(120));
        public static final Pose2d right46 = new Pose2d(4, 2.874, Rotation2d.fromDegrees(60));
        public static final Pose2d left46 = new Pose2d(3.715, 3.03, Rotation2d.fromDegrees(60));
        public static final Pose2d right56 = new Pose2d(3.247, 3.849, Rotation2d.fromDegrees(0));
        public static final Pose2d left56 = new Pose2d(3.256, 4.191, Rotation2d.fromDegrees(0));
        public static final Pose2d right66 = new Pose2d(3.715, 5.029, Rotation2d.fromDegrees(-60));
        public static final Pose2d left66 = new Pose2d(4.014, 5.187, Rotation2d.fromDegrees(-60));
    }

    public static class ReefCentersPoses {
        public static final Pose2d center16 = new Pose2d(5.125,5.121, Rotation2d.fromDegrees(-120));
        public static final Pose2d center26 = new Pose2d(5.775,4.030, Rotation2d.fromDegrees(180));
        public static final Pose2d center36 = new Pose2d(5.130,2.918, Rotation2d.fromDegrees(120));
        public static final Pose2d center46 = new Pose2d(3.844,2.918, Rotation2d.fromDegrees(60));
        public static final Pose2d center56 = new Pose2d(3.201,4.020, Rotation2d.fromDegrees(0));
        public static final Pose2d center66 = new Pose2d(3.842,5.144, Rotation2d.fromDegrees(-60));
    }
    public static class RedReefTagLocations {
            public static final Pose2d id1116 = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240));
            public static final Pose2d id1026 = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180));
            public static final Pose2d id936 = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120));
            public static final Pose2d id846 = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60));
            public static final Pose2d id756 = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0));
            public static final Pose2d id666 = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300));

            public static final List<Pose2d> REDREEFTAGS = List.of(id1116, id1026, id936, id846, id756, id666);
    }

    public static class BlueReefTagLocations {
            public static final Pose2d id2016 = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60));
            public static final Pose2d id2126 = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0));
            public static final Pose2d id2236 = new Pose2d(Units.inchesToMeters(193.1), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300));
            public static final Pose2d id1746 = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240));
            public static final Pose2d id1856 = new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180));
            public static final Pose2d id1966 = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60));

            public static final List<Pose2d> BLUEREEFTAGS = List.of(id2016, id2126, id2236, id1746, id1856, id1966); 
            
    }
    public static final class ShooterSubsystemConstants {
        public static final int kLeaderTalonID = 1;
        public static final int kFollowerTalonID = 2;
        public static final int kBeamBreakID = 3;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
        public static final double kMaxVelocity = 0.0;
        public static final double kMaxAcceleration = 0.0;
    }

}