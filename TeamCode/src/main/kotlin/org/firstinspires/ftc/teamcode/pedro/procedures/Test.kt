package org.firstinspires.ftc.teamcode.pedro.procedures

import com.pedropathing.algorithm.Algorithm
import com.pedropathing.api.Paths
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Pose
import com.pedropathing.paths.curves.Curve
import com.pedropathing.paths.interpolator.Interpolator
import com.pedropathing.tuning.autotune.*
import com.pedropathing.utils.Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.Function
import java.util.function.Supplier

class Tests(
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>?,
	var localizerFunction: Function<HardwareMap?, Localizer>?,
	var algorithmSupplier: Supplier<Algorithm?>?,
) : Procedure("Tests", "A procedure for testing the Follower.") {
	internal enum class Test {
		@DisplayName("Hold Test")
		HOLD,
		
		@DisplayName("Line Test")
		LINE,
		
		@DisplayName("Curve Test")
		CURVED,
		
		@DisplayName("Interpolation Test")
		INTERPOLATION_CURVED,
		
		@DisplayName("Localization Test")
		LOCALIZATION,
		
		@DisplayName("Odometry Test")
		ODOMETRY,
		
		@DisplayName("Driving Test")
		DRIVING,
		
		@DisplayName("Pose Test")
		POSE
	}
	
	var followerFunction: Function<HardwareMap?, Follower>? = null
	
	@Throws(InterruptedException::class)
	override fun run() {
		var completed = false
		var algorithm = true
		var localizer = true
		var drivetrain = true
		
		if (algorithmSupplier == null) algorithm = false
		
		if (localizerFunction == null) localizer = false
		
		if (drivetrainFunction == null) drivetrain = false
		
		if (algorithm && localizer && drivetrain) followerFunction =
			Function { hardwareMap: HardwareMap? ->
				Follower(
					localizerFunction!!.apply(hardwareMap),
					drivetrainFunction!!.apply(hardwareMap),
					algorithmSupplier!!.get()
				)
			}
		
		val inputs = inputs("Select", "Select")
		val selectedTest = inputs.e<Test>("Test", Test::class.java).withDefault(Test.LINE)
		val distance: Inputs.Field<Double?> = inputs.d("Distance").withDefault(48.0)
		
		awaitInputs(inputs)
		
		when (selectedTest.get()) {
			Test.HOLD -> {
				require(algorithm) { "Algorithm is required for Hold Test." }
				completed = runOpMode<Boolean?>(TestsHold(followerFunction!!))!!
			}
			
			Test.LINE -> {
				require(algorithm) { "Algorithm is required for Hold Test." }
				completed = runOpMode<Boolean?>(TestsLine(followerFunction!!, distance.get()!!))!!
			}
			
			Test.CURVED -> {
				require(algorithm) { "Algorithm is required for Hold Test." }
				completed = runOpMode<Boolean?>(TestsCurve(followerFunction!!, distance.get()!!))!!
			}
			
			Test.INTERPOLATION_CURVED -> {
				require(algorithm) { "Algorithm is required for Hold Test." }
				completed =
					runOpMode<Boolean?>(TestsInterpolation(followerFunction!!, distance.get()!!))!!
			}
			
			Test.LOCALIZATION -> {
				require(drivetrain) { "Drivetrain is required for Localization Test." }
				require(localizer) { "Localizer is required for Localization Test." }
				completed = runOpMode<Boolean?>(
					TestsLocalization(
						drivetrainFunction!!,
						localizerFunction!!
					)
				)!!
			}
			
			Test.ODOMETRY -> {
				require(drivetrain) { "Drivetrain is required for Odometry Test." }
				require(localizer) { "Localizer is required for Odometry Test." }
				completed =
					runOpMode<Boolean?>(TestsOdometry(drivetrainFunction!!, localizerFunction!!))!!
				if (!completed) abort("Failed odometry test. Please check your odometry pods and ensure they are functioning correctly.")
			}
			
			Test.POSE -> {
				require(localizer) { "Localizer is required for Pose Test." }
				completed = runOpMode<Boolean?>(TestsPose(localizerFunction!!))!!
			}
			
			Test.DRIVING -> {
				require(drivetrain) { "Drivetrain is required for Driving Test." }
				completed = runOpMode<Boolean?>(TestsDriving(drivetrainFunction!!))!!
			}
			
			null -> TODO()
		}
		
		result("Completed", completed)
	}
}

internal class TestsHold(var followerFunction: Function<HardwareMap?, Follower>) :
	TuningOpMode<Boolean?>("Hold Test", "Tests the Follower's ability to hold a position.", true) {
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val follower = followerFunction.apply(hardwareMap)
		follower.setPose(Pose.zero())
		Thread.sleep(1000)
		waitForStart()
		follower.setPose(Pose.zero())
		follower.update()
		follower.hold(Pose.zero())
		while (opModeIsActive()) {
			follower.update()
		}
		return true
	}
}

internal class TestsLine(
	var followerFunction: Function<HardwareMap?, Follower>,
	var distance: Double,
) : TuningOpMode<Boolean?>("Line Test", "Tests the Follower's ability to follow a line.", true) {
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val follower = followerFunction.apply(hardwareMap)
		follower.setPose(Pose.zero())
		
		var forward = true
		
		val path1 = Paths.line(Pose.zero(), Pose(distance, 0.0, 0.0)).constant(0.0)
		val path2 = Paths.line(Pose(distance, 0.0, 0.0), Pose.zero()).constant(0.0)
		
		Thread.sleep(1000)
		waitForStart()
		follower.setPose(Pose.zero())
		follower.update()
		follower.follow(path1)
		
		while (opModeIsActive()) {
			follower.update()
			if (follower.atParametricEnd()) {
				if (forward) {
					follower.follow(path2)
				} else {
					follower.follow(path1)
				}
				forward = !forward
			}
		}
		return true
	}
}

internal class TestsCurve(
	var followerFunction: Function<HardwareMap?, Follower>,
	var distance: Double,
) : TuningOpMode<Boolean?>("Curve Test", "Tests the Follower's ability to follow a curve.", true) {
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val follower = followerFunction.apply(hardwareMap)
		follower.setPose(Pose.zero())
		
		var forward = true
		
		val path1 =
			Paths.curve(Pose.zero(), Pose(distance + 0, 0.0), Pose(distance, distance)).tangent()
		val path2 =
			Paths.curve(Pose(distance, distance), Pose(distance, 0.0), Pose.zero()).tangent()
		
		Thread.sleep(1000)
		waitForStart()
		follower.setPose(Pose.zero())
		follower.update()
		follower.follow(path1)
		
		while (opModeIsActive()) {
			follower.update()
			if (follower.atParametricEnd()) {
				if (forward) {
					follower.follow(path2)
				} else {
					follower.follow(path1)
				}
				forward = !forward
			}
		}
		return true
	}
}

internal class TestsInterpolation(
	var followerFunction: Function<HardwareMap?, Follower>,
	var distance: Double,
) : TuningOpMode<Boolean?>(
	"Interpolation Curve Test",
	"Tests the Follower's ability to follow a curve with several interpolations.",
	true
) {
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val follower = followerFunction.apply(hardwareMap)
		follower.setPose(Pose.zero())
		
		var forward = true
		
		val path1 =
			Paths.curve(Pose.zero(), Pose(distance + 0, 0.0), Pose(distance, distance)).heading(
				Interpolator { curve: Curve?, t: Double -> Math.PI })
		val path2 = Paths.curve(Pose(distance, distance), Pose(distance, 0.0), Pose.zero()).heading(
			Interpolator.piecewise().until(0.5, Interpolator.tangent)
				.until(1.0, Interpolator.constant(0.0))
		)
		
		Thread.sleep(1000)
		waitForStart()
		follower.setPose(Pose.zero())
		follower.update()
		follower.follow(path1)
		
		while (opModeIsActive()) {
			follower.update()
			if (follower.atParametricEnd()) {
				if (forward) {
					follower.follow(path2)
				} else {
					follower.follow(path1)
				}
				forward = !forward
			}
		}
		return true
	}
}

internal class TestsLocalization(
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	var localizerFunction: Function<HardwareMap?, Localizer>,
) : TuningOpMode<Boolean?>("Localization Test", "Verifies localization and manual control.", true) {
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		
		while (opModeIsActive()) {
			drivetrain.drive(
				DrivePowers(
					-gamepad1.left_stick_y.toDouble(),
					-gamepad1.left_stick_x.toDouble(),
					-gamepad1.right_stick_x.toDouble()
				), true
			)
			localizer.update()
			telemetry.addData("Pose", localizer.pose())
			telemetry.update()
		}
		return true
	}
}

internal class TestsOdometry(
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	var localizerFunction: Function<HardwareMap?, Localizer>,
) : TuningOpMode<Boolean?>("Localization Test", "Verifies localization and manual control.", true) {
	enum class Test {
		FORWARD,
		LEFT,
		TURN,
		IDLE
	}
	
	private var test = Test.FORWARD
	var totalHeading: Double = 0.0
	var prevHeading: Double = 0.0
	
	private val timer = ElapsedTime()
	
	private var passedX = false
	private var passedY = false
	private var passedHeading = false
	
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		
		timer.reset()
		
		while (opModeIsActive()) {
			localizer.update()
			
			if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero()
					.y() || localizer.pose().heading() != Pose.zero().heading()
			) {
				val currentHeading = localizer.pose().heading()
				totalHeading += Angle.normalizeSigned(currentHeading - prevHeading)
				prevHeading = currentHeading
			}
			
			when (test) {
				Test.FORWARD -> {
					drivetrain.drive(DrivePowers(POWER, 0.0, 0.0), false)
					
					if (timer.seconds() > 1) {
						test = Test.LEFT
						timer.reset()
					}
				}
				
				Test.LEFT -> {
					drivetrain.drive(DrivePowers(0.0, POWER, 0.0), false)
					
					if (timer.seconds() > 1) {
						test = Test.TURN
						timer.reset()
						
						prevHeading = localizer.pose().heading()
						totalHeading = 0.0
					}
				}
				
				Test.TURN -> {
					drivetrain.drive(DrivePowers(0.0, 0.0, POWER), false)
					
					if (timer.seconds() > 1) {
						test = Test.IDLE
						timer.reset()
					}
				}
				
				Test.IDLE -> {
					drivetrain.drive(DrivePowers(0.0, 0.0, 0.0), true)
					
					telemetry.addData("Test", "Completed")
					
					val pose = localizer.pose()
					
					if (pose.x() < 0) telemetry.addData("xPod Direction", "Flipped")
					else if (pose.x() < 2) telemetry.addData("xPod Resolution", "Too high")
					else if (pose.x() > 144) telemetry.addData("xPod Resolution", "Too low")
					else {
						telemetry.addData("xPod", "Good")
						passedX = true
					}
					
					if (pose.y() < 0) telemetry.addData("yPod Direction", "Flipped")
					else if (pose.y() < 2) telemetry.addData("yPod Resolution", "Too high")
					else if (pose.y() > 144) telemetry.addData("yPod Resolution", "Too low")
					else {
						telemetry.addData("yPod", "Good")
						passedY = true
					}
					
					if (totalHeading < 0) telemetry.addData("Heading Direction", "Flipped")
					else if (totalHeading < 0.02) telemetry.addData(
						"Heading Resolution",
						"Too high"
					)
					else if (totalHeading > 2 * Math.PI) telemetry.addData(
						"Heading Resolution",
						"Too low"
					)
					else {
						telemetry.addData("Heading", "Good")
						passedHeading = true
					}
				}
			}
			
			telemetry.addData("Pose", localizer.pose())
			telemetry.update()
		}
		return passedX && passedY && passedHeading
	}
	
	companion object {
		var POWER: Double = 0.5
	}
}

internal class TestsDriving(var drivetrainFunction: Function<HardwareMap?, Drivetrain>) :
	TuningOpMode<Boolean?>(
		"Driving Test",
		"Tests raw drivetrain control without localization.",
		true
	) {
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		waitForStart()
		while (opModeIsActive()) {
			drivetrain.drive(
				DrivePowers(
					-gamepad1.left_stick_y.toDouble(),
					-gamepad1.left_stick_x.toDouble(),
					-gamepad1.right_stick_x.toDouble()
				), true
			)
		}
		return true
	}
}

internal class TestsPose(var localizerFunction: Function<HardwareMap?, Localizer>) :
	TuningOpMode<Boolean?>("Pose Test", "Verifies localizer output without a drivetrain.", true) {
	@Throws(InterruptedException::class)
	public override fun runTuningOpMode(): Boolean {
		val localizer = localizerFunction.apply(hardwareMap)
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		while (opModeIsActive()) {
			localizer.update()
			telemetry.addData("Pose", localizer.pose())
			telemetry.update()
		}
		return true
	}
}