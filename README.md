Simulink block for online trajectory generation using the Reflexxes Motion Library from github. The block provides instantaneous setpoint generation capabilities for example motion control systems. C++ code can be generated from the block using the Simulink/Embedded Coder. 

Usage instructions:
1. Download the TypeII version of the Reflexxes Motion Library from http://www.reflexxes.com/products/overview-and-download or https://github.com/Reflexxes/RMLTypeII.
2. Unzip the contents to the ReflexxesTypeII folder.
3. Compile the Simulink S-functions using the script make_trajgen.m.
4. Open and run the Simulink model trajgen_pos.slx (position-based) or trajgen_vel.slx (velocity-based).
5. If Simulink/Embedded Coder is available, generate/build C++ code in Simulink by Code->C/C++ Code->Build Model. (Note that C code generation is not supported) 
