package org.firstinspires.ftc.teamcode.util

import java.lang.reflect.*
import kotlin.reflect.*

class UnsafeAccess {
	companion object {
	
	}
}

fun  Any.accessField(fieldName: String): Any? {
	val field: Field = this::class.java.getDeclaredField(fieldName);
	
	field.isAccessible = true;
	
	val value = field.get(this);
	
	return value;
}