Gamma_count SPEC
----------------

*Should count pulses as accurately as possible.
*Should use Arduino hardware interrupts.
*Keep interrupt routine as short as possible - just increment a variable, at most. 
*Ensure that you declare the interrupt number for the input with a "#define" statement - see example below.
	"#define gammaInt 2"
	
