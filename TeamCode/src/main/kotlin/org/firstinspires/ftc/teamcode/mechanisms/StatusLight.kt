package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.Blinker

import dev.nextftc.hardware.*
import dev.nextftc.hardware.actuators.*
import dev.nextftc.robot.*
import dev.nextftc.units.Seconds
import dev.nextftc.units.unittypes.TimeUnit
import org.firstinspires.ftc.teamcode.util.accessField

enum class Status {
	Idle,
	Active,
	Failure,
	
	Okay,
	Warn,
	Error,
}

fun Status.toColor(): NextRGBIndicator.Color = when (this) {
	Status.Idle -> NextRGBIndicator.Color.AZURE
	Status.Active -> NextRGBIndicator.Color.INDIGO
	Status.Failure -> NextRGBIndicator.Color.VIOLET
	
	Status.Okay -> NextRGBIndicator.Color.GREEN
	Status.Warn -> NextRGBIndicator.Color.ORANGE
	Status.Error -> NextRGBIndicator.Color.RED
}

class StatusLight : Mechanism {
//	var indicator: NextRGBIndicator = NextRGBIndicator("status")
	private var currentStatus: Status = Status.Idle
	
	var index = 0.0
	
	fun set(status: Status) {
		this.currentStatus = status
		
		if (status == Status.Failure) {
			val module = RobotController.controlHub.accessField("module") as LynxModule
			
			module.pattern = listOf(Blinker.Step(0x00FF0000, 1, java.util.concurrent.TimeUnit.SECONDS))
		}
		
//		indicator.setColor(currentStatus.toColor())
//		indicator.setBrightness(0.5)
	}
}
