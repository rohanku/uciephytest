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
  val zctl = new TerminationControlIO
  val vref_ctl = Input(UInt(7.W))
}

class RxLane extends RawModule {
  val io = IO(new RxLaneIO)

  val verilogBlackBox = Module(new VerilogRxLane)
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
  verilogBlackBox.io.zctl_0 := io.zctl.zctl_0
  verilogBlackBox.io.zctl_1 := io.zctl.zctl_1
  verilogBlackBox.io.zctl_2 := io.zctl.zctl_2
  verilogBlackBox.io.zctl_3 := io.zctl.zctl_3
  verilogBlackBox.io.zctl_4 := io.zctl.zctl_4
  verilogBlackBox.io.zctl_5 := io.zctl.zctl_5
  verilogBlackBox.io.zctl_6 := io.zctl.zctl_6
  verilogBlackBox.io.zctl_7 := io.zctl.zctl_7
  verilogBlackBox.io.zctl_8 := io.zctl.zctl_8
  verilogBlackBox.io.zctl_9 := io.zctl.zctl_9
  verilogBlackBox.io.zctl_10 := io.zctl.zctl_10
  verilogBlackBox.io.zctl_11 := io.zctl.zctl_11
  verilogBlackBox.io.zctl_12 := io.zctl.zctl_12
  verilogBlackBox.io.zctl_13 := io.zctl.zctl_13
  verilogBlackBox.io.zctl_14 := io.zctl.zctl_14
  verilogBlackBox.io.zctl_15 := io.zctl.zctl_15
  verilogBlackBox.io.zctl_16 := io.zctl.zctl_16
  verilogBlackBox.io.zctl_17 := io.zctl.zctl_17
  verilogBlackBox.io.zctl_18 := io.zctl.zctl_18
  verilogBlackBox.io.zctl_19 := io.zctl.zctl_19
  verilogBlackBox.io.zctl_20 := io.zctl.zctl_20
  verilogBlackBox.io.zctl_21 := io.zctl.zctl_21
  verilogBlackBox.io.zctl_22 := io.zctl.zctl_22
  verilogBlackBox.io.zctl_23 := io.zctl.zctl_23
  verilogBlackBox.io.zctl_24 := io.zctl.zctl_24
  verilogBlackBox.io.zctl_25 := io.zctl.zctl_25
  verilogBlackBox.io.zctl_26 := io.zctl.zctl_26
  verilogBlackBox.io.zctl_27 := io.zctl.zctl_27
  verilogBlackBox.io.zctl_28 := io.zctl.zctl_28
  verilogBlackBox.io.zctl_29 := io.zctl.zctl_29
  verilogBlackBox.io.zctl_30 := io.zctl.zctl_30
  verilogBlackBox.io.zctl_31 := io.zctl.zctl_31
  verilogBlackBox.io.zctl_32 := io.zctl.zctl_32
  verilogBlackBox.io.zctl_33 := io.zctl.zctl_33
  verilogBlackBox.io.zctl_34 := io.zctl.zctl_34
  verilogBlackBox.io.zctl_35 := io.zctl.zctl_35
  verilogBlackBox.io.zctl_36 := io.zctl.zctl_36
  verilogBlackBox.io.zctl_37 := io.zctl.zctl_37
  verilogBlackBox.io.zctl_38 := io.zctl.zctl_38
  verilogBlackBox.io.zctl_39 := io.zctl.zctl_39
  verilogBlackBox.io.zctl_40 := io.zctl.zctl_40
  verilogBlackBox.io.zctl_41 := io.zctl.zctl_41
  verilogBlackBox.io.zctl_42 := io.zctl.zctl_42
  verilogBlackBox.io.zctl_43 := io.zctl.zctl_43
  verilogBlackBox.io.zctl_44 := io.zctl.zctl_44
  verilogBlackBox.io.zctl_45 := io.zctl.zctl_45
  verilogBlackBox.io.zctl_46 := io.zctl.zctl_46
  verilogBlackBox.io.zctl_47 := io.zctl.zctl_47
  verilogBlackBox.io.zctl_48 := io.zctl.zctl_48
  verilogBlackBox.io.zctl_49 := io.zctl.zctl_49
  verilogBlackBox.io.zctl_50 := io.zctl.zctl_50
  verilogBlackBox.io.zctl_51 := io.zctl.zctl_51
  verilogBlackBox.io.zctl_52 := io.zctl.zctl_52
  verilogBlackBox.io.zctl_53 := io.zctl.zctl_53
  verilogBlackBox.io.zctl_54 := io.zctl.zctl_54
  verilogBlackBox.io.zctl_55 := io.zctl.zctl_55
  verilogBlackBox.io.zctl_56 := io.zctl.zctl_56
  verilogBlackBox.io.zctl_57 := io.zctl.zctl_57
  verilogBlackBox.io.zctl_58 := io.zctl.zctl_58
  verilogBlackBox.io.zctl_59 := io.zctl.zctl_59
  verilogBlackBox.io.zctl_60 := io.zctl.zctl_60
  verilogBlackBox.io.zctl_61 := io.zctl.zctl_61
  verilogBlackBox.io.zctl_62 := io.zctl.zctl_62
  verilogBlackBox.io.zctl_63 := io.zctl.zctl_63
  verilogBlackBox.io.vref_sel_0 := io.vref_ctl(0)
  verilogBlackBox.io.vref_sel_1 := io.vref_ctl(1)
  verilogBlackBox.io.vref_sel_2 := io.vref_ctl(2)
  verilogBlackBox.io.vref_sel_3 := io.vref_ctl(3)
  verilogBlackBox.io.vref_sel_4 := io.vref_ctl(4)
  verilogBlackBox.io.vref_sel_5 := io.vref_ctl(5)
  verilogBlackBox.io.vref_sel_6 := io.vref_ctl(6)
  // val ctr = withClockAndReset(io.clk, !io.resetb) { RegInit(7.U((log2Ceil(Phy.SerdesRatio) - 1).W)) }
  // ctr := ctr + 1.U

  // val divClock = withClockAndReset(io.clk, !io.resetb) { RegInit(true.B) }
  // when (ctr === 0.U) {
  //   divClock := !divClock
  // }

  // val shiftReg = withClockAndReset(io.clk, !io.resetb) { RegInit(0.U(Phy.SerdesRatio.W)) }
  // shiftReg := shiftReg << 1.U | io.din.asUInt

  // val outputReg = withClock(divClock.asClock) {
  //   RegNext(Reverse(shiftReg))
  // }

  // io.divclk := divClock.asClock
  // io.dout := outputReg.asTypeOf(io.dout)
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
  val out0 = Output(Bool())
}

class VerilogRxLane extends BlackBox {
  val io = IO(new VerilogRxLaneIO)

  override val desiredName = "rxtile"
}

class RxClkIO extends Bundle {
  val clkin = Input(Bool())
  val clkout = Output(Bool())
  val zctl = new TerminationControlIO
}

class RxClk extends RawModule {
  val io = IO(new RxClkIO)

  val verilogBlackBox = Module(new VerilogRxClk)
  verilogBlackBox.io.clkin := io.clkin
  io.clkout := verilogBlackBox.io.clkout
  verilogBlackBox.io.zctl_0 := io.zctl.zctl_0
  verilogBlackBox.io.zctl_1 := io.zctl.zctl_1
  verilogBlackBox.io.zctl_2 := io.zctl.zctl_2
  verilogBlackBox.io.zctl_3 := io.zctl.zctl_3
  verilogBlackBox.io.zctl_4 := io.zctl.zctl_4
  verilogBlackBox.io.zctl_5 := io.zctl.zctl_5
  verilogBlackBox.io.zctl_6 := io.zctl.zctl_6
  verilogBlackBox.io.zctl_7 := io.zctl.zctl_7
  verilogBlackBox.io.zctl_8 := io.zctl.zctl_8
  verilogBlackBox.io.zctl_9 := io.zctl.zctl_9
  verilogBlackBox.io.zctl_10 := io.zctl.zctl_10
  verilogBlackBox.io.zctl_11 := io.zctl.zctl_11
  verilogBlackBox.io.zctl_12 := io.zctl.zctl_12
  verilogBlackBox.io.zctl_13 := io.zctl.zctl_13
  verilogBlackBox.io.zctl_14 := io.zctl.zctl_14
  verilogBlackBox.io.zctl_15 := io.zctl.zctl_15
  verilogBlackBox.io.zctl_16 := io.zctl.zctl_16
  verilogBlackBox.io.zctl_17 := io.zctl.zctl_17
  verilogBlackBox.io.zctl_18 := io.zctl.zctl_18
  verilogBlackBox.io.zctl_19 := io.zctl.zctl_19
  verilogBlackBox.io.zctl_20 := io.zctl.zctl_20
  verilogBlackBox.io.zctl_21 := io.zctl.zctl_21
  verilogBlackBox.io.zctl_22 := io.zctl.zctl_22
  verilogBlackBox.io.zctl_23 := io.zctl.zctl_23
  verilogBlackBox.io.zctl_24 := io.zctl.zctl_24
  verilogBlackBox.io.zctl_25 := io.zctl.zctl_25
  verilogBlackBox.io.zctl_26 := io.zctl.zctl_26
  verilogBlackBox.io.zctl_27 := io.zctl.zctl_27
  verilogBlackBox.io.zctl_28 := io.zctl.zctl_28
  verilogBlackBox.io.zctl_29 := io.zctl.zctl_29
  verilogBlackBox.io.zctl_30 := io.zctl.zctl_30
  verilogBlackBox.io.zctl_31 := io.zctl.zctl_31
  verilogBlackBox.io.zctl_32 := io.zctl.zctl_32
  verilogBlackBox.io.zctl_33 := io.zctl.zctl_33
  verilogBlackBox.io.zctl_34 := io.zctl.zctl_34
  verilogBlackBox.io.zctl_35 := io.zctl.zctl_35
  verilogBlackBox.io.zctl_36 := io.zctl.zctl_36
  verilogBlackBox.io.zctl_37 := io.zctl.zctl_37
  verilogBlackBox.io.zctl_38 := io.zctl.zctl_38
  verilogBlackBox.io.zctl_39 := io.zctl.zctl_39
  verilogBlackBox.io.zctl_40 := io.zctl.zctl_40
  verilogBlackBox.io.zctl_41 := io.zctl.zctl_41
  verilogBlackBox.io.zctl_42 := io.zctl.zctl_42
  verilogBlackBox.io.zctl_43 := io.zctl.zctl_43
  verilogBlackBox.io.zctl_44 := io.zctl.zctl_44
  verilogBlackBox.io.zctl_45 := io.zctl.zctl_45
  verilogBlackBox.io.zctl_46 := io.zctl.zctl_46
  verilogBlackBox.io.zctl_47 := io.zctl.zctl_47
  verilogBlackBox.io.zctl_48 := io.zctl.zctl_48
  verilogBlackBox.io.zctl_49 := io.zctl.zctl_49
  verilogBlackBox.io.zctl_50 := io.zctl.zctl_50
  verilogBlackBox.io.zctl_51 := io.zctl.zctl_51
  verilogBlackBox.io.zctl_52 := io.zctl.zctl_52
  verilogBlackBox.io.zctl_53 := io.zctl.zctl_53
  verilogBlackBox.io.zctl_54 := io.zctl.zctl_54
  verilogBlackBox.io.zctl_55 := io.zctl.zctl_55
  verilogBlackBox.io.zctl_56 := io.zctl.zctl_56
  verilogBlackBox.io.zctl_57 := io.zctl.zctl_57
  verilogBlackBox.io.zctl_58 := io.zctl.zctl_58
  verilogBlackBox.io.zctl_59 := io.zctl.zctl_59
  verilogBlackBox.io.zctl_60 := io.zctl.zctl_60
  verilogBlackBox.io.zctl_61 := io.zctl.zctl_61
  verilogBlackBox.io.zctl_62 := io.zctl.zctl_62
  verilogBlackBox.io.zctl_63 := io.zctl.zctl_63

  // io.clkout := io.clkin
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

  override val desiredName = "rxclk"
}
