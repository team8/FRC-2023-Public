package frc2023.robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.esotericsoftware.minlog.Log;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.json.JSONObject;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc2023.auto.*;
import frc2023.behavior.*;
import frc2023.behavior.routines.arm.ArmReZeroRoutine;
import frc2023.behavior.routines.arm.PivotReZeroRoutine;
import frc2023.config.RobotConfig;
import frc2023.config.constants.RobotConstants;
import frc2023.subsystems.*;
import frc2023.subsystems.Drive;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Lighting;
import frc2023.subsystems.SubsystemBase;
import frc2023.subsystems.Vision;
import frc2023.subsystems.swerve.SwerveModule;
import frc2023.util.LiveGraph;
import frc2023.util.LoopOverrunDebugger;
import frc2023.util.Util;
import frc2023.util.config.Configs;
import frc2023.util.service.RobotService;

public class Robot extends LoggedRobot {

	public static SimulatedRobot sim;

	private static final String loggerTag = Util.classToJsonName(Robot.class);
	private static final boolean canUseHardware = RobotBase.isReal() || !System.getProperty("os.name").startsWith("Mac");
	private final RobotConfig config = Configs.get(RobotConfig.class);
	private final OperatorInterface operatorInterface = new OperatorInterface();
	private final RoutineManager routineManager = new RoutineManager();
	private final Commands commands = new Commands();
	private final RobotState robotState = new RobotState(new Pose2d());
	private AutoBase auto;
	public static boolean shouldEngage;
	public static DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
	private RoutineBase autoRoutine;

	/* Subsystems */
	private final Drive drive = Drive.getInstance();
	private final Intake intake = Intake.getInstance();
	private final Vision vision = Vision.getInstance();
	private final Arm arm = Arm.getInstance();
	private final Pivot pivot = Pivot.getInstance();
	private final Lighting lighting = Lighting.getInstance();

	// for cc2 "serverService", "networkLoggerService", "graphingService","telemetryService", "webService"
	private final Set<SubsystemBase> subsystems = Set.of(drive, intake, vision, pivot, arm, lighting);

	private Set<SubsystemBase> enabledSubsystems = Set.of(vision);
	private final Set<RobotService> services = Set.of();
	private Set<RobotService> enabledServices;

	public Robot() {
		super(RobotConstants.period);
	}

	@Override
	public void robotInit() {
		LiveWindow.disableAllTelemetry();
		//commands.wantedArmSecondStageState = Arm.SecondStageState.RE_ZERO;

		commands.addWantedRoutine(new ParallelRoutine(new ArmReZeroRoutine(0.2), new PivotReZeroRoutine(0.2)));

		Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

		Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick
		Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

		String setupSummary = setupSubsystemsAndServices();

		enabledSubsystems.forEach(SubsystemBase::configureHardware);
		enabledServices.forEach(RobotService::start);

		Log.info(loggerTag, setupSummary);

		// TODO: do lighting
//		mCommands.lightingWantedState = Lighting.State.INIT;
//		if (mEnabledSubsystems.contains(mLighting)) {
//			mLighting.update(mCommands, mRobotState);
//			mLighting.writeHardware(mRobotState);
//		}

		// pre-loads config information to upload manager
		Iterator configIterator = Configs.getActiveConfigNames().iterator();
		JSONObject configJson = new JSONObject();
		Object temp;
		while (configIterator.hasNext()) {
			temp = configIterator.next();
			configJson.put(temp.toString(), new JSONObject(Configs.get(Configs.getClassFromName(temp.toString())).toString()));
		}
		Log.info(configJson.toString());

		Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

		// AUTO
		alliance = DriverStation.Alliance.Red;

		auto = new UpperMove(robotState);
		shouldEngage = true;
		autoRoutine = auto.getRoutine();

	}

	@Override
	public void simulationInit() {
//		Log.info(kLoggerTag, "Writing path CSV file...");
		sim = new SimulatedRobot();
		robotState.simulated = true;
		enabledSubsystems.forEach(SubsystemBase::simulationInit);
		alliance = DriverStation.Alliance.Red;
		pathToCsv();
	}

	private void pathToCsv() {
//		var drivePath = new StartCenterFriendlyTrenchThreeShootThree().getRoutine();
		RoutineBase drivePath = null;
		try (var writer = new PrintWriter(new BufferedWriter(new FileWriter("auto.csv")))) {
			writer.write("x,y,d" + '\n');
			var points = new LinkedList<Pose2d>();
			recurseRoutine(drivePath, points);
			for (Pose2d pose : points) {
				Translation2d point = pose.getTranslation();
				writer.write(String.format("%f,%f,%f%n", point.getY() * -39.37, point.getX() * 39.37, pose.getRotation().getDegrees()));
			}
		} catch (IOException writeException) {
			writeException.printStackTrace();
		}
	}

	private void recurseRoutine(RoutineBase routine, Deque<Pose2d> points) {
		if (routine instanceof MultipleRoutineBase) {
			var multiple = (MultipleRoutineBase) routine;
			for (RoutineBase childRoutine : multiple.getRoutines()) {
				recurseRoutine(childRoutine, points);
			}
		}
	}

	@Override
	public void disabledInit() {
		robotState.gamePeriod = RobotState.GamePeriod.DISABLED;
		resetCommandsAndRoutines();

		updateDriveNeutralMode(config.coastDriveWhenDisabled);
		arm.setBrakeMode(false);

		if (enabledSubsystems.contains(lighting)) {
			commands.lightingWantedState = Lighting.State.DISABLE;
			lighting.update(commands, robotState);
			lighting.writeHardware(robotState);

		}
	}

	@Override
	public void autonomousInit() {
		/*
		commands.wantedArmSecondStageState = Arm.SecondStageState.RE_ZERO;
		startStage(RobotState.GamePeriod.AUTO);
		if (autoRoutine.isFinished())
			autoRoutine = auto.getRoutine();
		commands.addWantedRoutine(autoRoutine);
		 */

//		commands.lightingWantedState = Lighting.State.OFF;
	}

	private void startStage(RobotState.GamePeriod period) {
		robotState.gamePeriod = period;
		resetCommandsAndRoutines();
		updateDriveNeutralMode(false);
	}

	@Override
	public void teleopInit() {
		//TODO: check ramifications of line above
		commands.wantedArmSecondStageState = Arm.SecondStageState.RE_ZERO;
		commands.wantedPivotState = Pivot.State.RE_ZERO;
		commands.wantedArmFirstStageState = Arm.FirstStageState.RAISED;
		commands.resetOdometry(robotState.robotPoseMeters.transformBy(new Transform2d(new Translation2d(), new Rotation2d(Math.PI))));
		commands.addWantedRoutine(new ArmReZeroRoutine(0.1));
		arm.setBrakeMode(true);
		startStage(RobotState.GamePeriod.TELEOP);
		commands.setDriveTeleop();
		operatorInterface.setConeWanted();
//		commands.lightingWantedState = Lighting.State.OFF;
//		if (enabledSubsystems.contains(mLighting)) {
//			mLighting.update(commands, robotState);
//			mLighting.writeHardware(robotState);
//		}
	}

	@Override
	public void testInit() {
		startStage(RobotState.GamePeriod.TESTING);
	}

	@Override
	public void robotPeriodic() {
		commands.reset();
		for (RobotService robotService : enabledServices) {
			robotService.update(robotState, commands);
		}
		LiveGraph.add("isEnabled", isEnabled());

		int d = 0;
		for (SwerveModule s : Drive.getInstance().getSwerveModules()) {
			d++;
			LiveGraph.add("Swerve/Module" + d, s.getPosition().angle.getDegrees());
		}
		vision.update(commands, robotState);
		updateRobotState();
	}

	@Override
	public void simulationPeriodic() {
		routineManager.update(commands, robotState);
		enabledSubsystems.forEach(s -> s.simulate(robotState, commands));
	}

	@Override
	public void disabledPeriodic() {
		updateDriveNeutralMode(config.coastDriveWhenDisabled);
		enabledSubsystems.forEach(s -> s.readHardware(robotState));
		enabledSubsystems.forEach(s -> s.logSubsystem(robotState));
		robotState.log();
		lighting.update(commands, robotState);
		lighting.writeHardware(robotState);
	}

	@Override
	public void autonomousPeriodic() {
		/*
		updateRobotState();
		routineManager.update(commands, robotState);
		LiveGraph.add("robot/usevision", robotState.useAutoVision);
		commands.log();
		updateSubsystemsAndApplyOutputs();
		operatorInterface.resetPeriodic(commands);
		
		 */
	}

	public static LoopOverrunDebugger overrunDebugger = new LoopOverrunDebugger("teleop", 0.02);

	@Override
	public void teleopPeriodic() {
		overrunDebugger.reset();
		updateRobotState();
		overrunDebugger.addPoint("robotState");
		operatorInterface.updateCommands(commands, robotState);
		overrunDebugger.addPoint("updateCommands");
		routineManager.update(commands, robotState);
		overrunDebugger.addPoint("routineManagerUpdate");
		commands.log();
		updateSubsystemsAndApplyOutputs();
		overrunDebugger.addPoint("updateSubsystemsAndApplyOutputs");
		overrunDebugger.finish();
		operatorInterface.resetPeriodic(commands);
	}

	@Override
	public void testPeriodic() {
		teleopPeriodic();
	}

	private void resetCommandsAndRoutines() {
		operatorInterface.reset(commands);
		routineManager.clearRunningRoutines();
		updateSubsystemsAndApplyOutputs();
	}

	private void updateRobotState() {
		enabledSubsystems.forEach(s -> s.readHardware(robotState));
		robotState.calculations();
		robotState.log();
	}

	private void updateSubsystemsAndApplyOutputs() {
		for (SubsystemBase subsystem : enabledSubsystems) {
			subsystem.update(commands, robotState);
			subsystem.logSubsystem(robotState);
			overrunDebugger.addPoint(subsystem.getName());
		}
		enabledSubsystems.forEach(s -> s.writeHardware(robotState));
		overrunDebugger.addPoint("updateHardware");
	}

	private String setupSubsystemsAndServices() {
		// TODO: same logic twice in a row
		Map<String, RobotService> configToService = services.stream()
				.collect(Collectors.toUnmodifiableMap(RobotService::getConfigName, Function.identity()));
		enabledServices = config.enabledServices.stream().map(configToService::get)
				.collect(Collectors.toUnmodifiableSet());
		Map<String, SubsystemBase> configToSubsystem = subsystems.stream()
				.collect(Collectors.toUnmodifiableMap(SubsystemBase::getName, Function.identity()));
		enabledSubsystems = config.enabledSubsystems.stream().map(configToSubsystem::get)
				.collect(Collectors.toUnmodifiableSet());
		var summaryBuilder = new StringBuilder("\n");
		summaryBuilder.append("===================\n");
		summaryBuilder.append("Enabled subsystems:\n");
		summaryBuilder.append("-------------------\n");
		for (SubsystemBase enabledSubsystem : enabledSubsystems) {
			summaryBuilder.append(enabledSubsystem.getName()).append("\n");
		}
		summaryBuilder.append("=================\n");
		summaryBuilder.append("Enabled services:\n");
		summaryBuilder.append("-----------------\n");
		for (RobotService enabledService : enabledServices) {
			summaryBuilder.append(enabledService.getConfigName()).append("\n");
		}
		return summaryBuilder.toString();
	}

	private void updateDriveNeutralMode(boolean shouldCoast) {
		// TODO: with swerve
//		if (enabledSubsystems.contains(mDrive)) mHardwareWriter.setDriveNeutralMode(shouldCoast ? NeutralMode.Coast : NeutralMode.Brake);
	}
}
