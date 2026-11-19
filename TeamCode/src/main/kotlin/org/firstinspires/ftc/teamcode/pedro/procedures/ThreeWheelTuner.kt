package org.firstinspires.ftc.teamcode.pedro.procedures

import android.util.Log
import com.pedropathing.config.Configuration
import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.Encoder
import com.pedropathing.revhub.localizers.ThreeWheelConfig
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer
import com.pedropathing.tuning.autotune.Inputs
import com.pedropathing.tuning.autotune.Procedure
import com.pedropathing.tuning.autotune.TuningOpMode
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.List
import kotlin.math.abs

class ThreeWheelTuner : Procedure("Three Wheel Tuner", "Tune three odometry pods") {
	companion object {
		private const val TAG = "ThreeWheelTuner"
		
		private var leftEncoderName: String? = "frontLeftMotor"
		var rightEncoderName: String? = "backRightMotor"
		var strafeEncoderName: String? = "frontRightMotor"
		var frontLeftName: String? = "frontLeftMotor"
		var backLeftName: String? = "backLeftMotor"
		var frontRightName: String? = "frontRightMotor"
		var backRightName: String? = "backRightMotor"
		
		fun direction(direction: Double): String {
			return if (direction == Encoder.REVERSE) "Encoder.REVERSE" else "Encoder.FORWARD"
		}
		
		fun config(
			left: Boolean, forward: Double, strafe: Double,
			leftDirection: Double, rightDirection: Double, strafeDirection: Double
		): ThreeWheelConfig {
			Log.d(TAG, "Configuring ThreeWheelConfig: left=$left, forwardTicksToInches=$forward, strafeTicksToInches=$strafe")
			return ThreeWheelConfig(Configuration { c: ThreeWheelConfig? ->
				c!!.leftEncoderName.set(leftEncoderName)
				c.rightEncoderName.set(rightEncoderName)
				c.strafeEncoderName.set(strafeEncoderName)
				c.leftPodY.set(if (left) 0.0 else 1.0)
				c.rightPodY.set(if (left) -1.0 else 0.0)
				c.strafePodX.set(0.0)
				c.forwardTicksToInches.set(forward)
				c.strafeTicksToInches.set(strafe)
				c.turnTicksToRadians.set(0.0)
				c.leftEncoderDirection.set(leftDirection)
				c.rightEncoderDirection.set(rightDirection)
				c.strafeEncoderDirection.set(strafeDirection)
			})
		}
		
		fun localizer(map: HardwareMap, config: ThreeWheelConfig): ThreeWheelLocalizer {
			Log.d(TAG, "Initializing ThreeWheelLocalizer...")
			for (hub in map.getAll<LynxModule>(LynxModule::class.java)) {
				hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
			}
			for (name in arrayOf<String?>(
				frontLeftName,
				backLeftName,
				frontRightName,
				backRightName
			)) {
				val motor = map.tryGet<DcMotorEx?>(DcMotorEx::class.java, name)
				if (motor == null) {
					Log.w(TAG, "Drive motor not found in hardware map: $name")
					continue
				}
				motor.setPower(0.0)
				motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
			}
			return ThreeWheelLocalizer(map, config)
		}
	}
	
	@Throws(InterruptedException::class)
	override fun run() {
		Log.i(TAG, "Starting ThreeWheelTuner procedure execution.")
		
		val setup = inputs(
			"Encoder Setup",
			"Set the motor ports that the three odometry encoders are plugged into."
		)
		val leftEncoder: Inputs.Field<String?> = setup.s("Left Encoder Motor Name").withDefault("frontLeftMotor")
		val rightEncoder: Inputs.Field<String?> = setup.s("Right Encoder Motor Name").withDefault("backRightMotor")
		val strafeEncoder: Inputs.Field<String?> = setup.s("Strafe Encoder Motor Name").withDefault("frontRightMotor")
		val frontLeft: Inputs.Field<String?> = setup.s("Front Left Drive Motor Name").withDefault("frontLeftMotor")
		val backLeft: Inputs.Field<String?> = setup.s("Back Left Drive Motor Name").withDefault("backLeftMotor")
		val frontRight: Inputs.Field<String?> = setup.s("Front Right Drive Motor Name").withDefault("frontRightMotor")
		val backRight: Inputs.Field<String?> = setup.s("Back Right Drive Motor Name").withDefault("backRightMotor")
		
		awaitInputs(setup)
		leftEncoderName = leftEncoder.get()
		rightEncoderName = rightEncoder.get()
		strafeEncoderName = strafeEncoder.get()
		frontLeftName = frontLeft.get()
		backLeftName = backLeft.get()
		frontRightName = frontRight.get()
		backRightName = backRight.get()
		
		Log.i(TAG, "Encoder setup received -> Left: $leftEncoderName, Right: $rightEncoderName, Strafe: $strafeEncoderName")
		
		val resolution = inputs(
			"Encoder Resolution Identification",
			"Set a positive push distance in inches. Keep the robot straight during each push."
		)
		val distance: Inputs.Field<Double?> = resolution.d("Distance").withDefault(48.0)
		awaitInputs(resolution)
		
		val targetDistance = distance.get() ?: 48.0
		Log.i(TAG, "Target calibration distance set to: $targetDistance inches")
		
		if (!(targetDistance > 0.0)) {
			Log.e(TAG, "Aborted: Distance must be greater than 0. Received: $targetDistance")
			abort("Enter a positive distance in inches.")
			return
		}
		
		val left = measure("Left", targetDistance)
		if (left == null) {
			Log.e(TAG, "Left encoder measurement failed or returned null.")
			return
		}
		val right = measure("Right", targetDistance)
		if (right == null) {
			Log.e(TAG, "Right encoder measurement failed or returned null.")
			return
		}
		val strafe = measure("Strafe", targetDistance)
		if (strafe == null) {
			Log.e(TAG, "Strafe encoder measurement failed or returned null.")
			return
		}
		
		val leftTicksPerInch: Double = left.get(0)!!
		val rightTicksPerInch: Double = right.get(0)!!
		val strafeTicksPerInch: Double = strafe.get(0)!!
		val forwardTicksPerInch = 2.0 / (1.0 / leftTicksPerInch + 1.0 / rightTicksPerInch)
		
		val forward = 1.0 / forwardTicksPerInch
		val lateral = 1.0 / strafeTicksPerInch
		
		Log.i(TAG, "Calculated Resolution -> Left Ticks/Inch: $leftTicksPerInch, Right Ticks/Inch: $rightTicksPerInch, Strafe Ticks/Inch: $strafeTicksPerInch")
		
		val leftOffsets = runOpMode<MutableList<Double?>?>(
			ThreeWheelOffsets(
				true, forward, lateral, left.get(1)!!, right.get(1)!!, strafe.get(1)!!
			)
		)
		if (leftOffsets == null) {
			Log.e(TAG, "Left offset stage failed (null result).")
			abort("Left stage ended without parallel pod travel. Rotate 180 degrees CCW, then press Stop.")
			return
		}
		Log.i(TAG, "Left offset measured successfully -> Y Offset: ${leftOffsets.get(0)}")
		
		val rightOffsets = runOpMode<MutableList<Double?>?>(
			ThreeWheelOffsets(
				false, forward, lateral, left.get(1)!!, right.get(1)!!, strafe.get(1)!!
			)
		)
		if (rightOffsets == null) {
			Log.e(TAG, "Right offset stage failed (null result).")
			abort("Right stage ended without parallel pod travel. Rotate 180 degrees CCW, then press Stop.")
			return
		}
		Log.i(TAG, "Right offset measured successfully -> Y Offset: ${rightOffsets.get(0)}")
		
		val config: ThreeWheelConfig = Companion.config(
			true, forward, lateral,
			left.get(1)!!, right.get(1)!!, strafe.get(1)!!
		)
		config.leftPodY.set(leftOffsets.get(0))
		config.rightPodY.set(rightOffsets.get(0))
		config.turnTicksToRadians.set(forward)
		
		val turn = runOpMode<Double?>(ThreeWheelTurn(config))
		if (turn == null) {
			Log.e(TAG, "Turn stage failed (null result).")
			abort("Turn stage ended without positive rotation. Rotate 360 degrees CCW, then press Stop.")
			return
		}
		Log.i(TAG, "Turn multiplier measured successfully -> $turn")
		
		val strafeX = ((leftOffsets.get(1)!! + rightOffsets.get(1)!!) / 2.0) / lateral
		Log.i(TAG, "Calculated strafePodX: $strafeX")
		
		result("leftEncoderName", leftEncoderName)
		result("rightEncoderName", rightEncoderName)
		result("strafeEncoderName", strafeEncoderName)
		result("leftPodY", leftOffsets.get(0))
		result("rightPodY", rightOffsets.get(0))
		result("strafePodX", strafeX)
		result("leftTicksPerInch", leftTicksPerInch)
		result("rightTicksPerInch", rightTicksPerInch)
		result("forwardTicksPerInch", forwardTicksPerInch)
		result("strafeTicksPerInch", strafeTicksPerInch)
		result("forwardTicksToInches", forward)
		result("strafeTicksToInches", lateral)
		result("turnTicksToRadians", turn)
		result("leftEncoderDirection", Companion.direction(left.get(1)!!))
		result("rightEncoderDirection", Companion.direction(right.get(1)!!))
		result("strafeEncoderDirection", Companion.direction(strafe.get(1)!!))
		
		code(
			Language.JAVA,
			"public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {\n" +
					"    c.leftEncoderName.set(\"" + leftEncoderName + "\");\n" +
					"    c.rightEncoderName.set(\"" + rightEncoderName + "\");\n" +
					"    c.strafeEncoderName.set(\"" + strafeEncoderName + "\");\n" +
					"    c.leftPodY.set(" + leftOffsets.get(0) + ");\n" +
					"    c.rightPodY.set(" + rightOffsets.get(0) + ");\n" +
					"    c.strafePodX.set(" + strafeX + ");\n" +
					"    c.forwardTicksToInches.set(" + forward + ");\n" +
					"    c.strafeTicksToInches.set(" + lateral + ");\n" +
					"    c.turnTicksToRadians.set(" + turn + ");\n" +
					"    c.leftEncoderDirection.set(" + Companion.direction(left.get(1)!!) + ");\n" +
					"    c.rightEncoderDirection.set(" + Companion.direction(right.get(1)!!) + ");\n" +
					"    c.strafeEncoderDirection.set(" + Companion.direction(strafe.get(1)!!) + ");\n" +
					"});"
		)
		Log.i(TAG, "ThreeWheelTuner procedure completed successfully.")
	}
	
	@Throws(InterruptedException::class)
	private fun measure(pod: String, distance: Double): MutableList<Double?>? {
		Log.d("ThreeWheelTuner.measure", "Starting measurement for pod: $pod over distance: $distance")
		val measured = runOpMode<MutableList<Double?>?>(ThreeWheelResolution(pod, distance))
		if (measured == null) {
			Log.e("ThreeWheelTuner.measure", "Measurement for $pod returned null.")
			abort(pod + " stage ended without a nonzero measurement. Check the displayed ticks, complete the push, then press Stop.")
			return null
		}
		Log.d("ThreeWheelTuner.measure", "$pod measurement successful: ticks/inch = ${measured.get(0)}, direction = ${measured.get(1)}")
		return measured
	}
}

internal class ThreeWheelResolution(var pod: String, var distance: Double) :
	TuningOpMode<MutableList<Double?>?>(
		pod + " Encoder Resolution and Direction",
		"After Start, push the robot " + (if (pod == "Strafe") "left " else "forward ") +
				distance + " inches exactly without turning. Stop moving, press Stop to save this measurement.",
		true
	) {
	override fun runTuningOpMode(): MutableList<Double?>? {
		Log.d("ThreeWheelResolution", "Running resolution tuning for $pod")
		val config: ThreeWheelConfig = ThreeWheelTuner.Companion.config(
			pod != "Right", 1.0, 1.0,
			Encoder.FORWARD, Encoder.FORWARD, Encoder.FORWARD
		)
		val localizer: ThreeWheelLocalizer = ThreeWheelTuner.Companion.localizer(hardwareMap, config)
		localizer.setPose(Pose(0.0, 0.0))
		var position: Pose? = null
		
		waitForStart()
		while (!isStopRequested()) {
			localizer.update()
			position = localizer.pose()
			telemetry.addData("Current X", position.x())
			telemetry.addData("Current Y", position.y())
			telemetry.update()
		}
		
		if (position == null) {
			Log.w("ThreeWheelResolution", "Position was null at stop request for $pod")
			return null
		}
		val movement = if (pod == "Strafe") position.y() else position.x()
		Log.d("ThreeWheelResolution", "$pod movement recorded: $movement over target distance: $distance")
		if (movement == 0.0) {
			Log.w("ThreeWheelResolution", "Movement was exactly 0.0 for $pod")
			return null
		}
		return List.of<Double?>(
			abs(movement / distance),
			if (movement < 0) Encoder.REVERSE else Encoder.FORWARD
		)
	}
}

internal class ThreeWheelOffsets(
	var left: Boolean, var forward: Double, var strafe: Double,
	var leftDirection: Double, var rightDirection: Double, var strafeDirection: Double
) : TuningOpMode<MutableList<Double?>?>(
	(if (left) "Left" else "Right") + " Pod Offset Identification",
	"After Start, rotate exactly 180 degrees counterclockwise about the robot center. " +
			"Keep that center fixed. Stop moving, press Stop to save this measurement.", true
) {
	override fun runTuningOpMode(): MutableList<Double?>? {
		Log.d("ThreeWheelOffsets", "Running offset tuning for side left=$left")
		val config: ThreeWheelConfig = ThreeWheelTuner.Companion.config(
			left, forward, strafe,
			leftDirection, rightDirection, strafeDirection
		)
		val localizer: ThreeWheelLocalizer = ThreeWheelTuner.Companion.localizer(hardwareMap, config)
		localizer.setPose(Pose(0.0, 0.0))
		localizer.update()
		var position: Pose? = null
		
		waitForStart()
		while (!isStopRequested()) {
			localizer.update()
			position = localizer.pose()
			telemetry.addData("X Offset Pos", position.x())
			telemetry.addData("Y Offset Pos", position.y())
			Log.d("ThreeWheelOffsets", "Y=${position.y()}")
			
			telemetry.update()
		}
		
		if (position == null) {
			Log.w("ThreeWheelOffsets", "Position was null at stop request for offset (left=$left)")
			return null
		}
		Log.d("ThreeWheelOffsets", "Final offset position (left=$left) -> X: ${position.x()}, Y: ${position.y()}")
		return List.of<Double?>(-position.x() / Math.PI, position.y() / Math.PI)
	}
}

internal class ThreeWheelTurn(var config: ThreeWheelConfig) : TuningOpMode<Double?>(
	"Turn Multiplier Identification",
	"After Start, rotate exactly 360 degrees counterclockwise. " +
			"Stop moving, press Stop to save this measurement.", true
) {
	override fun runTuningOpMode(): Double? {
		Log.d("ThreeWheelTurn", "Running turn multiplier identification")
		val localizer: ThreeWheelLocalizer = ThreeWheelTuner.Companion.localizer(hardwareMap, config)
		localizer.setPose(Pose(0.0, 0.0))
		localizer.update()
		val startHeading = localizer.getTotalHeading()
		var heading: Double? = null
		
		waitForStart()
		while (!isStopRequested()) {
			localizer.update()
			heading = localizer.getTotalHeading()
			telemetry.addData("Start Heading", startHeading)
			telemetry.addData("Current Heading", heading)
			telemetry.update()
		}
		
		if (heading == null || heading <= startHeading) {
			Log.w("ThreeWheelTurn", "Heading invalid or less than start heading. heading=$heading, start=$startHeading")
			return null
		}
		
		val measuredTurn = config.turnTicksToRadians.get() * (2.0 * Math.PI) / (heading - startHeading)
		Log.d("ThreeWheelTurn", "Calculated turn ticks to radians: $measuredTurn")
		return measuredTurn
	}
}