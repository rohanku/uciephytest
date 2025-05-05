din_width = 32
pu_ctl_width = 40
delay_width = 5
mux_en_width = 8
band_ctrl_width = 2
mix_en_width = 16
nen_out_width = 5
pen_out_width = 5

pd_ctlb_width = pu_ctl_width


code = "class VerilogTxLaneIO extends Bundle {\n"
code += "  val dll_reset = Input(Bool())\n  val dll_resetb = Input(Bool())\n  val ser_resetb = Input(Bool())\n"
code += "  val clkp = Input(Bool())\n  val clkn = Input(Bool())\n"
for i in range(din_width):
    code += f"  val din_{i} = Input(Bool())\n"
code += f"  val dout = Output(Bool())\n"
for i in range(pu_ctl_width):
    code += f"  val pu_ctl_{i} = Input(Bool())\n"
for i in range(pd_ctlb_width):
    code += f"  val pd_ctlb_{i} = Input(Bool())\n"
code += f"  val dll_en = Input(Bool())\n"
code += f"  val ocl = Input(Bool())\n"
for i in range(delay_width):
    code += f"  val delay_{i} = Input(Bool())\n"
for i in range(delay_width):
    code += f"  val delayb_{i} = Input(Bool())\n"
for i in range(mux_en_width):
    code += f"  val mux_en_{i} = Input(Bool())\n"
for i in range(mux_en_width):
    code += f"  val mux_enb_{i} = Input(Bool())\n"
for i in range(band_ctrl_width):
    code += f"  val band_ctrl_{i} = Input(Bool())\n"
for i in range(band_ctrl_width):
    code += f"  val band_ctrlb_{i} = Input(Bool())\n"
for i in range(mix_en_width):
    code += f"  val mix_en_{i} = Input(Bool())\n"
for i in range(mix_en_width):
    code += f"  val mix_enb_{i} = Input(Bool())\n"
for i in range(nen_out_width):
    code += f"  val nen_out_{i} = Input(Bool())\n"
for i in range(nen_out_width):
    code += f"  val nen_outb_{i} = Input(Bool())\n"
for i in range(pen_out_width):
    code += f"  val pen_out_{i} = Input(Bool())\n"
for i in range(pen_out_width):
    code += f"  val pen_outb_{i} = Input(Bool())\n"
code += "}\n"

print(code)
