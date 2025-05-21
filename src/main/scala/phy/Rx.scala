package uciephytest.phy

import chisel3._
import chisel3.util._
import chisel3.experimental.noPrefix

class RxLaneIO extends Bundle {
  val din = Input(Bool())
  val dout = new RxDoutIO
  val clk = Input(Clock())
  val clkb = Input(Clock())
  val divclk = Output(Clock())
  val resetb = Input(Bool())
  val zctl = Input(UInt(6.W))
  val vref_ctl = Input(UInt(7.W))
}

class RxLane(sim: Boolean = false) extends RawModule {
  val io = IO(new RxLaneIO)

  val verilogBlackBox = Module(new VerilogRxLane(sim))
  verilogBlackBox.io.din := io.din
  io.dout.dout_0 := verilogBlackBox.io.dout_0 
  io.dout.dout_1 := verilogBlackBox.io.dout_1 
  io.dout.dout_2 := verilogBlackBox.io.dout_2 
  io.dout.dout_3 := verilogBlackBox.io.dout_3 
  io.dout.dout_4 := verilogBlackBox.io.dout_4 
  io.dout.dout_5 := verilogBlackBox.io.dout_5 
  io.dout.dout_6 := verilogBlackBox.io.dout_6 
  io.dout.dout_7 := verilogBlackBox.io.dout_7 
  io.dout.dout_8 := verilogBlackBox.io.dout_8 
  io.dout.dout_9 := verilogBlackBox.io.dout_9 
  io.dout.dout_10 := verilogBlackBox.io.dout_10 
  io.dout.dout_11 := verilogBlackBox.io.dout_11 
  io.dout.dout_12 := verilogBlackBox.io.dout_12 
  io.dout.dout_13 := verilogBlackBox.io.dout_13 
  io.dout.dout_14 := verilogBlackBox.io.dout_14 
  io.dout.dout_15 := verilogBlackBox.io.dout_15 
  verilogBlackBox.io.clk := io.clk
  verilogBlackBox.io.clkb := io.clkb
  io.divclk := verilogBlackBox.io.divclk
  verilogBlackBox.io.resetb := io.resetb
  val zctlTherm = Wire(UInt(64.W))
  zctlTherm := (1.U << io.zctl) - 1.U
  verilogBlackBox.io.zctl_0 := zctlTherm(0)
  verilogBlackBox.io.zctl_1 := zctlTherm(1)
  verilogBlackBox.io.zctl_2 := zctlTherm(2)
  verilogBlackBox.io.zctl_3 := zctlTherm(3)
  verilogBlackBox.io.zctl_4 := zctlTherm(4)
  verilogBlackBox.io.zctl_5 := zctlTherm(5)
  verilogBlackBox.io.zctl_6 := zctlTherm(6)
  verilogBlackBox.io.zctl_7 := zctlTherm(7)
  verilogBlackBox.io.zctl_8 := zctlTherm(8)
  verilogBlackBox.io.zctl_9 := zctlTherm(9)
  verilogBlackBox.io.zctl_10 := zctlTherm(10)
  verilogBlackBox.io.zctl_11 := zctlTherm(11)
  verilogBlackBox.io.zctl_12 := zctlTherm(12)
  verilogBlackBox.io.zctl_13 := zctlTherm(13)
  verilogBlackBox.io.zctl_14 := zctlTherm(14)
  verilogBlackBox.io.zctl_15 := zctlTherm(15)
  verilogBlackBox.io.zctl_16 := zctlTherm(16)
  verilogBlackBox.io.zctl_17 := zctlTherm(17)
  verilogBlackBox.io.zctl_18 := zctlTherm(18)
  verilogBlackBox.io.zctl_19 := zctlTherm(19)
  verilogBlackBox.io.zctl_20 := zctlTherm(20)
  verilogBlackBox.io.zctl_21 := zctlTherm(21)
  verilogBlackBox.io.zctl_22 := zctlTherm(22)
  verilogBlackBox.io.zctl_23 := zctlTherm(23)
  verilogBlackBox.io.zctl_24 := zctlTherm(24)
  verilogBlackBox.io.zctl_25 := zctlTherm(25)
  verilogBlackBox.io.zctl_26 := zctlTherm(26)
  verilogBlackBox.io.zctl_27 := zctlTherm(27)
  verilogBlackBox.io.zctl_28 := zctlTherm(28)
  verilogBlackBox.io.zctl_29 := zctlTherm(29)
  verilogBlackBox.io.zctl_30 := zctlTherm(30)
  verilogBlackBox.io.zctl_31 := zctlTherm(31)
  verilogBlackBox.io.zctl_32 := zctlTherm(32)
  verilogBlackBox.io.zctl_33 := zctlTherm(33)
  verilogBlackBox.io.zctl_34 := zctlTherm(34)
  verilogBlackBox.io.zctl_35 := zctlTherm(35)
  verilogBlackBox.io.zctl_36 := zctlTherm(36)
  verilogBlackBox.io.zctl_37 := zctlTherm(37)
  verilogBlackBox.io.zctl_38 := zctlTherm(38)
  verilogBlackBox.io.zctl_39 := zctlTherm(39)
  verilogBlackBox.io.zctl_40 := zctlTherm(40)
  verilogBlackBox.io.zctl_41 := zctlTherm(41)
  verilogBlackBox.io.zctl_42 := zctlTherm(42)
  verilogBlackBox.io.zctl_43 := zctlTherm(43)
  verilogBlackBox.io.zctl_44 := zctlTherm(44)
  verilogBlackBox.io.zctl_45 := zctlTherm(45)
  verilogBlackBox.io.zctl_46 := zctlTherm(46)
  verilogBlackBox.io.zctl_47 := zctlTherm(47)
  verilogBlackBox.io.zctl_48 := zctlTherm(48)
  verilogBlackBox.io.zctl_49 := zctlTherm(49)
  verilogBlackBox.io.zctl_50 := zctlTherm(50)
  verilogBlackBox.io.zctl_51 := zctlTherm(51)
  verilogBlackBox.io.zctl_52 := zctlTherm(52)
  verilogBlackBox.io.zctl_53 := zctlTherm(53)
  verilogBlackBox.io.zctl_54 := zctlTherm(54)
  verilogBlackBox.io.zctl_55 := zctlTherm(55)
  verilogBlackBox.io.zctl_56 := zctlTherm(56)
  verilogBlackBox.io.zctl_57 := zctlTherm(57)
  verilogBlackBox.io.zctl_58 := zctlTherm(58)
  verilogBlackBox.io.zctl_59 := zctlTherm(59)
  verilogBlackBox.io.zctl_60 := zctlTherm(60)
  verilogBlackBox.io.zctl_61 := zctlTherm(61)
  verilogBlackBox.io.zctl_62 := zctlTherm(62)
  verilogBlackBox.io.zctl_63 := zctlTherm(63)
  verilogBlackBox.io.vref_sel_0 := io.vref_ctl(0)
  verilogBlackBox.io.vref_sel_1 := io.vref_ctl(1)
  verilogBlackBox.io.vref_sel_2 := io.vref_ctl(2)
  verilogBlackBox.io.vref_sel_3 := io.vref_ctl(3)
  verilogBlackBox.io.vref_sel_4 := io.vref_ctl(4)
  verilogBlackBox.io.vref_sel_5 := io.vref_ctl(5)
  verilogBlackBox.io.vref_sel_6 := io.vref_ctl(6)
}

class VerilogRxLaneIO extends Bundle {
  val din = Input(Bool())
  val dout_0 = Output(Bool())
  val dout_1 = Output(Bool())
  val dout_2 = Output(Bool())
  val dout_3 = Output(Bool())
  val dout_4 = Output(Bool())
  val dout_5 = Output(Bool())
  val dout_6 = Output(Bool())
  val dout_7 = Output(Bool())
  val dout_8 = Output(Bool())
  val dout_9 = Output(Bool())
  val dout_10 = Output(Bool())
  val dout_11 = Output(Bool())
  val dout_12 = Output(Bool())
  val dout_13 = Output(Bool())
  val dout_14 = Output(Bool())
  val dout_15 = Output(Bool())
  val clk = Input(Clock())
  val clkb = Input(Clock())
  val divclk = Output(Clock())
  val resetb = Input(Bool())
  val zctl_0 = Input(Bool())
  val zctl_1 = Input(Bool())
  val zctl_2 = Input(Bool())
  val zctl_3 = Input(Bool())
  val zctl_4 = Input(Bool())
  val zctl_5 = Input(Bool())
  val zctl_6 = Input(Bool())
  val zctl_7 = Input(Bool())
  val zctl_8 = Input(Bool())
  val zctl_9 = Input(Bool())
  val zctl_10 = Input(Bool())
  val zctl_11 = Input(Bool())
  val zctl_12 = Input(Bool())
  val zctl_13 = Input(Bool())
  val zctl_14 = Input(Bool())
  val zctl_15 = Input(Bool())
  val zctl_16 = Input(Bool())
  val zctl_17 = Input(Bool())
  val zctl_18 = Input(Bool())
  val zctl_19 = Input(Bool())
  val zctl_20 = Input(Bool())
  val zctl_21 = Input(Bool())
  val zctl_22 = Input(Bool())
  val zctl_23 = Input(Bool())
  val zctl_24 = Input(Bool())
  val zctl_25 = Input(Bool())
  val zctl_26 = Input(Bool())
  val zctl_27 = Input(Bool())
  val zctl_28 = Input(Bool())
  val zctl_29 = Input(Bool())
  val zctl_30 = Input(Bool())
  val zctl_31 = Input(Bool())
  val zctl_32 = Input(Bool())
  val zctl_33 = Input(Bool())
  val zctl_34 = Input(Bool())
  val zctl_35 = Input(Bool())
  val zctl_36 = Input(Bool())
  val zctl_37 = Input(Bool())
  val zctl_38 = Input(Bool())
  val zctl_39 = Input(Bool())
  val zctl_40 = Input(Bool())
  val zctl_41 = Input(Bool())
  val zctl_42 = Input(Bool())
  val zctl_43 = Input(Bool())
  val zctl_44 = Input(Bool())
  val zctl_45 = Input(Bool())
  val zctl_46 = Input(Bool())
  val zctl_47 = Input(Bool())
  val zctl_48 = Input(Bool())
  val zctl_49 = Input(Bool())
  val zctl_50 = Input(Bool())
  val zctl_51 = Input(Bool())
  val zctl_52 = Input(Bool())
  val zctl_53 = Input(Bool())
  val zctl_54 = Input(Bool())
  val zctl_55 = Input(Bool())
  val zctl_56 = Input(Bool())
  val zctl_57 = Input(Bool())
  val zctl_58 = Input(Bool())
  val zctl_59 = Input(Bool())
  val zctl_60 = Input(Bool())
  val zctl_61 = Input(Bool())
  val zctl_62 = Input(Bool())
  val zctl_63 = Input(Bool())
  val vref_sel_0 = Input(Bool())
  val vref_sel_1 = Input(Bool())
  val vref_sel_2 = Input(Bool())
  val vref_sel_3 = Input(Bool())
  val vref_sel_4 = Input(Bool())
  val vref_sel_5 = Input(Bool())
  val vref_sel_6 = Input(Bool())
}

class VerilogRxLane(sim: Boolean = false) extends BlackBox with HasBlackBoxInline {
  val io = IO(new VerilogRxLaneIO)

  override val desiredName = "rxtile"

  if (sim) {
    setInline(
      "rxtile.v",
      """
module rxtile (
  inout vdd,
   inout vss,
   input din,
   output dout_0,
   output dout_1,
   output dout_2,
   output dout_3,
   output dout_4,
   output dout_5,
   output dout_6,
   output dout_7,
   output dout_8,
   output dout_9,
   output dout_10,
   output dout_11,
   output dout_12,
   output dout_13,
   output dout_14,
   output dout_15,
   input clk,
   input clkb,
   output divclk,
   input resetb,
   input vref_sel_0,
   input vref_sel_1,
   input vref_sel_2,
   input vref_sel_3,
   input vref_sel_4,
   input vref_sel_5,
   input vref_sel_6,
   input zctl_0,
   input zctl_1,
   input zctl_2,
   input zctl_3,
   input zctl_4,
   input zctl_5,
   input zctl_6,
   input zctl_7,
   input zctl_8,
   input zctl_9,
   input zctl_10,
   input zctl_11,
   input zctl_12,
   input zctl_13,
   input zctl_14,
   input zctl_15,
   input zctl_16,
   input zctl_17,
   input zctl_18,
   input zctl_19,
   input zctl_20,
   input zctl_21,
   input zctl_22,
   input zctl_23,
   input zctl_24,
   input zctl_25,
   input zctl_26,
   input zctl_27,
   input zctl_28,
   input zctl_29,
   input zctl_30,
   input zctl_31,
   input zctl_32,
   input zctl_33,
   input zctl_34,
   input zctl_35,
   input zctl_36,
   input zctl_37,
   input zctl_38,
   input zctl_39,
   input zctl_40,
   input zctl_41,
   input zctl_42,
   input zctl_43,
   input zctl_44,
   input zctl_45,
   input zctl_46,
   input zctl_47,
   input zctl_48,
   input zctl_49,
   input zctl_50,
   input zctl_51,
   input zctl_52,
   input zctl_53,
   input zctl_54,
   input zctl_55,
   input zctl_56,
   input zctl_57,
   input zctl_58,
   input zctl_59,
   input zctl_60,
   input zctl_61,
   input zctl_62,
   input zctl_63
);
  reg rstbSync;
  always @(negedge resetb) begin
    rstbSync <= resetb;
  end
  always @(posedge clk) begin
    rstbSync <= resetb;
  end
  reg [1:0] ctr;
  reg divClock;
  reg [15:0] shiftReg;
  reg [15:0] outputReg;
  always @(posedge clk) begin
    if (rstbSync) begin
      ctr <= ctr + 1'b1;
      shiftReg <= (shiftReg << 1'b1) | din;
      if (ctr == 2'b0) begin
        divClock <= ~divClock;
      end
    end else begin
      divClock <= 1'b0;
      ctr <= 2'b1;
      shiftReg <= 16'b0;
    end
  end
  always @(negedge clk) begin
    shiftReg <= (shiftReg << 1'b1) | din;
  end
  always @(posedge divClock) begin
    outputReg <= shiftReg;
  end
  assign dout_0  = outputReg[15];
  assign dout_1  = outputReg[14];
  assign dout_2  = outputReg[13];
  assign dout_3  = outputReg[12];
  assign dout_4  = outputReg[11];
  assign dout_5  = outputReg[10];
  assign dout_6  = outputReg[9];
  assign dout_7  = outputReg[8];
  assign dout_8  = outputReg[7];
  assign dout_9  = outputReg[6];
  assign dout_10 = outputReg[5];
  assign dout_11 = outputReg[4];
  assign dout_12 = outputReg[3];
  assign dout_13 = outputReg[2];
  assign dout_14 = outputReg[1];
  assign dout_15 = outputReg[0];
  assign divclk = divClock;
endmodule
      """
    )
  }
}

class RxClkIO extends Bundle {
  val clkin = Input(Bool())
  val clkout = Output(Bool())
  val zctl = Input(UInt(6.W))
}

class RxClk(sim: Boolean = false) extends RawModule {
  val io = IO(new RxClkIO)

  if (sim) {
    io.clkout := io.clkin
  } else {
    val verilogBlackBox = Module(new VerilogRxClk)
    verilogBlackBox.io.clkin := io.clkin
    io.clkout := verilogBlackBox.io.clkout
    val zctlTherm = Wire(UInt(64.W))
    zctlTherm := (1.U << io.zctl) - 1.U
    verilogBlackBox.io.zctl_0 := zctlTherm(0)
    verilogBlackBox.io.zctl_1 := zctlTherm(1)
    verilogBlackBox.io.zctl_2 := zctlTherm(2)
    verilogBlackBox.io.zctl_3 := zctlTherm(3)
    verilogBlackBox.io.zctl_4 := zctlTherm(4)
    verilogBlackBox.io.zctl_5 := zctlTherm(5)
    verilogBlackBox.io.zctl_6 := zctlTherm(6)
    verilogBlackBox.io.zctl_7 := zctlTherm(7)
    verilogBlackBox.io.zctl_8 := zctlTherm(8)
    verilogBlackBox.io.zctl_9 := zctlTherm(9)
    verilogBlackBox.io.zctl_10 := zctlTherm(10)
    verilogBlackBox.io.zctl_11 := zctlTherm(11)
    verilogBlackBox.io.zctl_12 := zctlTherm(12)
    verilogBlackBox.io.zctl_13 := zctlTherm(13)
    verilogBlackBox.io.zctl_14 := zctlTherm(14)
    verilogBlackBox.io.zctl_15 := zctlTherm(15)
    verilogBlackBox.io.zctl_16 := zctlTherm(16)
    verilogBlackBox.io.zctl_17 := zctlTherm(17)
    verilogBlackBox.io.zctl_18 := zctlTherm(18)
    verilogBlackBox.io.zctl_19 := zctlTherm(19)
    verilogBlackBox.io.zctl_20 := zctlTherm(20)
    verilogBlackBox.io.zctl_21 := zctlTherm(21)
    verilogBlackBox.io.zctl_22 := zctlTherm(22)
    verilogBlackBox.io.zctl_23 := zctlTherm(23)
    verilogBlackBox.io.zctl_24 := zctlTherm(24)
    verilogBlackBox.io.zctl_25 := zctlTherm(25)
    verilogBlackBox.io.zctl_26 := zctlTherm(26)
    verilogBlackBox.io.zctl_27 := zctlTherm(27)
    verilogBlackBox.io.zctl_28 := zctlTherm(28)
    verilogBlackBox.io.zctl_29 := zctlTherm(29)
    verilogBlackBox.io.zctl_30 := zctlTherm(30)
    verilogBlackBox.io.zctl_31 := zctlTherm(31)
    verilogBlackBox.io.zctl_32 := zctlTherm(32)
    verilogBlackBox.io.zctl_33 := zctlTherm(33)
    verilogBlackBox.io.zctl_34 := zctlTherm(34)
    verilogBlackBox.io.zctl_35 := zctlTherm(35)
    verilogBlackBox.io.zctl_36 := zctlTherm(36)
    verilogBlackBox.io.zctl_37 := zctlTherm(37)
    verilogBlackBox.io.zctl_38 := zctlTherm(38)
    verilogBlackBox.io.zctl_39 := zctlTherm(39)
    verilogBlackBox.io.zctl_40 := zctlTherm(40)
    verilogBlackBox.io.zctl_41 := zctlTherm(41)
    verilogBlackBox.io.zctl_42 := zctlTherm(42)
    verilogBlackBox.io.zctl_43 := zctlTherm(43)
    verilogBlackBox.io.zctl_44 := zctlTherm(44)
    verilogBlackBox.io.zctl_45 := zctlTherm(45)
    verilogBlackBox.io.zctl_46 := zctlTherm(46)
    verilogBlackBox.io.zctl_47 := zctlTherm(47)
    verilogBlackBox.io.zctl_48 := zctlTherm(48)
    verilogBlackBox.io.zctl_49 := zctlTherm(49)
    verilogBlackBox.io.zctl_50 := zctlTherm(50)
    verilogBlackBox.io.zctl_51 := zctlTherm(51)
    verilogBlackBox.io.zctl_52 := zctlTherm(52)
    verilogBlackBox.io.zctl_53 := zctlTherm(53)
    verilogBlackBox.io.zctl_54 := zctlTherm(54)
    verilogBlackBox.io.zctl_55 := zctlTherm(55)
    verilogBlackBox.io.zctl_56 := zctlTherm(56)
    verilogBlackBox.io.zctl_57 := zctlTherm(57)
    verilogBlackBox.io.zctl_58 := zctlTherm(58)
    verilogBlackBox.io.zctl_59 := zctlTherm(59)
    verilogBlackBox.io.zctl_60 := zctlTherm(60)
    verilogBlackBox.io.zctl_61 := zctlTherm(61)
    verilogBlackBox.io.zctl_62 := zctlTherm(62)
    verilogBlackBox.io.zctl_63 := zctlTherm(63)
  }
}

class VerilogRxClkIO extends Bundle {
  val clkin = Input(Bool())
  val clkout = Output(Bool())
  val zctl_0 = Input(Bool())
  val zctl_1 = Input(Bool())
  val zctl_2 = Input(Bool())
  val zctl_3 = Input(Bool())
  val zctl_4 = Input(Bool())
  val zctl_5 = Input(Bool())
  val zctl_6 = Input(Bool())
  val zctl_7 = Input(Bool())
  val zctl_8 = Input(Bool())
  val zctl_9 = Input(Bool())
  val zctl_10 = Input(Bool())
  val zctl_11 = Input(Bool())
  val zctl_12 = Input(Bool())
  val zctl_13 = Input(Bool())
  val zctl_14 = Input(Bool())
  val zctl_15 = Input(Bool())
  val zctl_16 = Input(Bool())
  val zctl_17 = Input(Bool())
  val zctl_18 = Input(Bool())
  val zctl_19 = Input(Bool())
  val zctl_20 = Input(Bool())
  val zctl_21 = Input(Bool())
  val zctl_22 = Input(Bool())
  val zctl_23 = Input(Bool())
  val zctl_24 = Input(Bool())
  val zctl_25 = Input(Bool())
  val zctl_26 = Input(Bool())
  val zctl_27 = Input(Bool())
  val zctl_28 = Input(Bool())
  val zctl_29 = Input(Bool())
  val zctl_30 = Input(Bool())
  val zctl_31 = Input(Bool())
  val zctl_32 = Input(Bool())
  val zctl_33 = Input(Bool())
  val zctl_34 = Input(Bool())
  val zctl_35 = Input(Bool())
  val zctl_36 = Input(Bool())
  val zctl_37 = Input(Bool())
  val zctl_38 = Input(Bool())
  val zctl_39 = Input(Bool())
  val zctl_40 = Input(Bool())
  val zctl_41 = Input(Bool())
  val zctl_42 = Input(Bool())
  val zctl_43 = Input(Bool())
  val zctl_44 = Input(Bool())
  val zctl_45 = Input(Bool())
  val zctl_46 = Input(Bool())
  val zctl_47 = Input(Bool())
  val zctl_48 = Input(Bool())
  val zctl_49 = Input(Bool())
  val zctl_50 = Input(Bool())
  val zctl_51 = Input(Bool())
  val zctl_52 = Input(Bool())
  val zctl_53 = Input(Bool())
  val zctl_54 = Input(Bool())
  val zctl_55 = Input(Bool())
  val zctl_56 = Input(Bool())
  val zctl_57 = Input(Bool())
  val zctl_58 = Input(Bool())
  val zctl_59 = Input(Bool())
  val zctl_60 = Input(Bool())
  val zctl_61 = Input(Bool())
  val zctl_62 = Input(Bool())
  val zctl_63 = Input(Bool())
}

class VerilogRxClk extends BlackBox {
  val io = IO(new VerilogRxClkIO)

  override val desiredName = "rxclk_with_esd"
}
