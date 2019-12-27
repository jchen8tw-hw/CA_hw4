#/bin/sh
java -jar /opt/verilog-format/verilog-format.jar -f SingleCycleMIPS.v -s ~/.verilog-format.properties
java -jar /opt/verilog-format/verilog-format.jar -f SingleCycleMIPS_FPU.v  ~/.verilog-format.properties
