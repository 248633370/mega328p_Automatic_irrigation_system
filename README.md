# mega328p_Automatic_irrigation_system
Yet another automatic irrigation system

System based on atmega328p (Arduino can be used - overkill, but was designed for "on hands" materials).
It measure humidity about each 12 hours by two-needle sensor.

Device powered from ATX PSU +5V standby, and manage PS_ON by optocoupler for start PSU and use +12V through relay for drive water pump.
Device irrigate 10 seconds(hardcoded), after that measure humidity one more time to determine need iriigate once more.
