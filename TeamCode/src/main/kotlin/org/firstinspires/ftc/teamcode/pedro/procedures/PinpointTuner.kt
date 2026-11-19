package org.firstinspires.ftc.teamcode.pedro.procedures

import com.pedropathing.config.Configuration
import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.pedropathing.tuning.autotune.Inputs
import com.pedropathing.tuning.autotune.Procedure
import com.pedropathing.tuning.autotune.TuningOpMode
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.List
import java.util.OptionalDouble
import kotlin.math.abs

class PinpointTuner :
	Procedure("Pinpoint Tuner", "A procedure for tuning the Pinpoint localizer.") {
	internal enum class PodType {
		SWING_ARM,
		FOUR_BAR,
		CUSTOM
	}
	
	@Throws(InterruptedException::class)
	override fun run() {
		val inputs = inputs("Setup", "Set Pinpoint HardwareMap Name and Odometry Pod Type")
		val pinpointName: Inputs.Field<String?> =
			inputs.s("HardwareMap Name").withDefault("pinpoint")
		val podType = inputs.e<PodType>("Odometry Pod Type", PodType::class.java).withDefault(
			PodType.FOUR_BAR
		)
		awaitInputs(inputs)
		
		var customPodScalar = OptionalDouble.empty()
		
		if (podType.get() == PodType.CUSTOM) {
			val inputsCustom = inputs(
				"Custom Scalar Identification Push Distance",
				"Set the distance you will push your robot forward in inches"
			)
			val distance: Inputs.Field<Double?> = inputsCustom.d("Distance").withDefault(48.0)
			awaitInputs(inputsCustom)
			customPodScalar = OptionalDouble.of(
				runOpMode<Double?>(
					PinpointCustomPodScalar(
						distance.get()!!,
						pinpointName.get()
					)
				)!!
			)
		}
		
		val forwardPodReversed: Boolean = runOpMode<Boolean?>(
			PinpointForwardDirection(
				pinpointName.get(),
				podType.get(),
				customPodScalar
			)
		)!!
		val strafePodReversed: Boolean = runOpMode<Boolean?>(
			PinpointStrafeDirection(
				pinpointName.get(),
				podType.get(),
				customPodScalar
			)
		)!!
		
		val offsets: MutableList<Double?> = runOpMode<MutableList<Double?>?>(
			PinpointOffsets(
				pinpointName.get(),
				podType.get(),
				customPodScalar,
				forwardPodReversed,
				strafePodReversed
			)
		)!!
		
		result("name", pinpointName.get())
		
		if (customPodScalar.isPresent()) {
			result("podType", "Custom")
			result("ticksPerUnit", customPodScalar.getAsDouble())
		} else {
			result(
				"podType",
				if (podType.get() == PodType.SWING_ARM) GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD else GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
			)
		}
		
		result(
			"xPodDirection",
			if (forwardPodReversed) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD
		)
		result(
			"yPodDirection",
			if (strafePodReversed) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD
		)
		result("xPodOffset", offsets.get(0))
		result("yPodOffset", offsets.get(1))
		
		code(
			Language.JAVA,
			"public static PinpointConfig localizerConfig = new PinpointConfig(c -> {\n" +
					"    c.name.set(\"" + pinpointName.get() + "\");\n" +
					(if (customPodScalar.isPresent()) "    c.ticksPerUnit.set(OptionalDouble.of(" + customPodScalar.getAsDouble() + "));\n" else "    c.podType.set(" + (if (podType.get() == PodType.SWING_ARM) "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD" else "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD") + ");\n") +
					"    c.xPodOffset.set(" + offsets.get(0) + ");\n" +
					"    c.yPodOffset.set(" + offsets.get(1) + ");\n" +
					"    c.xPodDirection.set(" + (if (forwardPodReversed) "GoBildaPinpointDriver.EncoderDirection.REVERSED" else "GoBildaPinpointDriver.EncoderDirection.FORWARD") + ");\n" +
					"    c.yPodDirection.set(" + (if (strafePodReversed) "GoBildaPinpointDriver.EncoderDirection.REVERSED" else "GoBildaPinpointDriver.EncoderDirection.FORWARD") + ");\n" +
					"    c.globalDistanceUnit.set(DistanceUnit.INCH);\n" +
					"    c.offsetUnits.set(DistanceUnit.INCH);\n" +
					"});"
		)
	}
}

internal class PinpointCustomPodScalar(var distance: Double, var name: String?) :
	TuningOpMode<Double?>(
		"Custom Scalar Identification",
		("Determines the scalar for the custom pods of the Pinpoint localizer. \n"
				+ "Push your robot forward " + distance + " inches exactly and then stop the Opmode"),
		true
	) {
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Double {
		val config = PinpointConfig(Configuration { c: PinpointConfig? ->
			c!!.name.set(name)
			c.ticksPerUnit.set(OptionalDouble.of(1.0))
			c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			c.xPodOffset.set(0.0)
			c.yPodOffset.set(0.0)
		})
		val localizer = PinpointLocalizer(hardwareMap, config)
		localizer.setPose(Pose(0.0, 0.0))
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		while (!isStopRequested()) {
			localizer.update()
		}
		return abs((localizer.pose().x() / distance))
	}
}

internal class PinpointForwardDirection(
	var name: String?,
	var podType: PinpointTuner.PodType?,
	var customPodScalar: OptionalDouble,
) : TuningOpMode<Boolean?>(
	"Forward Direction Identification",
	"Determines if your forward pod needs to be reversed. \n"
			+ "Push your robot forward and then stop the Opmode",
	true
) {
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Boolean {
		val config = PinpointConfig(Configuration { c: PinpointConfig? ->
			c!!.name.set(name)
			c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			c.xPodOffset.set(0.0)
			c.yPodOffset.set(0.0)
			if (customPodScalar.isPresent()) {
				c.encoderResolutionUnit.set(DistanceUnit.INCH)
				c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()))
			} else {
				c.podType.set(if (podType == PinpointTuner.PodType.SWING_ARM) GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD else GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			}
		})
		val localizer = PinpointLocalizer(hardwareMap, config)
		localizer.setPose(Pose(0.0, 0.0))
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		
		// Keep the last reading taken before Stop. After Stop the Pinpoint reports a reset
		// pose (x = 0), so reading it afterwards always returns false (not reversed).
		var x = 0.0
		while (!isStopRequested()) {
			localizer.update()
			if (!isStopRequested()) {
				x = localizer.pose().x()
			}
		}
		
		return x < 0
	}
}

internal class PinpointStrafeDirection(
	var name: String?,
	var podType: PinpointTuner.PodType?,
	var customPodScalar: OptionalDouble,
) : TuningOpMode<Boolean?>(
	"Strafe Direction Identification",
	"Determines if your strafe pod needs to be reversed. \n"
			+ "Push your robot to the left and then stop the Opmode",
	true
) {
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Boolean {
		val config = PinpointConfig(Configuration { c: PinpointConfig? ->
			c!!.name.set(name)
			c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			c.xPodOffset.set(0.0)
			c.yPodOffset.set(0.0)
			if (customPodScalar.isPresent()) {
				c.encoderResolutionUnit.set(DistanceUnit.INCH)
				c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()))
			} else {
				c.podType.set(if (podType == PinpointTuner.PodType.SWING_ARM) GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD else GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			}
		})
		val localizer = PinpointLocalizer(hardwareMap, config)
		localizer.setPose(Pose(0.0, 0.0))
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		// Keep the last reading taken before Stop. After Stop the Pinpoint reports a reset
		// pose (y = 0), so reading it afterwards always returns false (not reversed).
		var y = 0.0
		while (!isStopRequested()) {
			localizer.update()
			if (!isStopRequested()) {
				y = localizer.pose().y()
			}
		}
		
		return y < 0
	}
}

internal class PinpointOffsets(
	var name: String?,
	var podType: PinpointTuner.PodType?,
	customPodScalar: OptionalDouble,
	forwardPodReversed: Boolean,
	strafePodReversed: Boolean,
) : TuningOpMode<MutableList<Double?>>(
	"Offsets Identification",
	"Automatically identifies the offsets for your Pinpoint localizer. \n"
			+ "Spin your robot in place 180 degrees counterclockwise and then stop the Opmode",
	true
) {
	var customPodScalar: OptionalDouble = OptionalDouble.empty()
	var forwardPodReversed: Boolean
	var strafePodReversed: Boolean
	private var previous: Pose = Pose.zero()
	
	init {
		if (customPodScalar.isPresent()) {
			this.customPodScalar = customPodScalar
		}
		this.forwardPodReversed = forwardPodReversed
		this.strafePodReversed = strafePodReversed
	}
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): MutableList<Double?> {
		val config = PinpointConfig(Configuration { c: PinpointConfig? ->
			c!!.name.set(name)
			c.xPodDirection.set(if (forwardPodReversed) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD)
			c.yPodDirection.set(if (strafePodReversed) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD)
			if (customPodScalar.isPresent()) {
				c.encoderResolutionUnit.set(DistanceUnit.INCH)
				c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()))
				c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU)
			} else {
				c.podType.set(if (podType == PinpointTuner.PodType.SWING_ARM) GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD else GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			}
			c.xPodOffset.set(0.0)
			c.yPodOffset.set(0.0)
			c.globalDistanceUnit.set(DistanceUnit.INCH)
			c.offsetUnits.set(DistanceUnit.INCH)
		})
		val localizer = PinpointLocalizer(hardwareMap, config)
		if (customPodScalar.isPresent()) {
			localizer.reset()
		}
		localizer.setPose(Pose.zero())
		localizer.update()
		
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		
		localizer.setPose(Pose.zero())
		
		while (!isStopRequested()) {
			previous = localizer.pose()
			localizer.update()
			
			telemetry.addData("heading", localizer.pose().heading())
			telemetry.addData("pose", localizer.pose())
			telemetry.addData("previous", previous)
			telemetry.update()
		}
		
		if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y()) {
			previous = localizer.pose()
		}
		
		return List.of<Double?>(((-previous.y()) / 2.0), ((-previous.x()) / 2.0))
	}
}