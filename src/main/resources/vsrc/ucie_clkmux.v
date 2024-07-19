module ucie_clkmux(
 input in0, in1,
 input mux0_en_0, mux0_en_1,
 input mux1_en_0, mux1_en_1,
 output out, outb
);
 assign out = mux0_en_0 ? in0 : in1;
endmodule
