# Controller and tool I/O from the real-time report (port 30000), published on "xarm_io_states"
# when the driver parameter io_states.enable is true (firmware >= 2.7.101).
std_msgs/Header header

# controller digital inputs: CI0-CI7 in bits 0-7, DI0-DI7 in bits 8-15
uint16 cgpio_digital_inputs

# controller digital outputs: CO0-CO7 in bits 0-7, DO0-DO7 in bits 8-15
uint16 cgpio_digital_outputs

# controller analog inputs AI0, AI1 (V)
float32[2] cgpio_analog_inputs

# controller analog outputs AO0, AO1 (V)
float32[2] cgpio_analog_outputs

# tool digital inputs, one bit per input as reported by the controller; filled while tool I/O monitoring is on
uint16 tgpio_digital_inputs

# tool digital outputs, one bit per output as reported by the controller (TO0: bit 0, TO1: bit 1);
# filled while tool I/O monitoring is on
uint16 tgpio_digital_outputs

# tool analog inputs (V), filled while tool I/O monitoring is on
float32[2] tgpio_analog_inputs
