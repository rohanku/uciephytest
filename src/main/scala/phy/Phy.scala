package uciephytest.phy

import chisel3._
import chisel3.util._
import chisel3.experimental.dataview._
import freechips.rocketchip.util.{AsyncQueue, AsyncQueueParams}

object Phy {
  val SerdesRatio = 32
  val QueueParams = AsyncQueueParams(depth = 32)
}

class TxIO(numLanes: Int = 16) extends Bundle  {
  val data = Vec(numLanes, Bits(Phy.SerdesRatio.W))
  val valid = Bits(Phy.SerdesRatio.W)
  val clkp = Bits(Phy.SerdesRatio.W)
  val clkn = Bits(Phy.SerdesRatio.W)
  val track = Bits(Phy.SerdesRatio.W)
}

class RxIO(numLanes: Int = 16) extends Bundle  {
  val data = Vec(numLanes, Bits(Phy.SerdesRatio.W))
  val valid = Bits(Phy.SerdesRatio.W)
  val track = Bits(Phy.SerdesRatio.W)
}

class PhyToTestIO(numLanes: Int = 16) extends Bundle {
  val tx = Flipped(DecoupledIO(new TxIO(numLanes)))
  val rx = DecoupledIO(new RxIO(numLanes))
  val txRst = Input(Bool())
  val rxRst = Input(Bool())
}

class SidebandIO extends Bundle {
  val txClk = Input(Bool())
  val txData = Input(Bool())
  val rxClk = Output(Bool())
  val rxData = Output(Bool())
}

class RefClkRxIO extends Bundle {
  val vip = Input(Bool())
  val vin = Input(Bool())
  val vop = Output(Bool())
  val von = Output(Bool())
}

class RefClkRx(sim: Boolean = false) extends BlackBox with HasBlackBoxInline {
  val io = IO(new RefClkRxIO)

  override val desiredName = "refclkrx"

  if (sim) {
    setInline("refclkrx.v",
      """module refclkrx(
      | input vip, vin,
      | output vop, von
      |);
      | assign vop = vip;
      | assign von = vin;
      |endmodule
      """.stripMargin)
  }
}

class ClkMuxIO extends Bundle {
  val in0 = Input(Bool())
  val in1 = Input(Bool())
  val mux0_en_0 = Input(Bool())
  val mux0_en_1 = Input(Bool())
  val mux1_en_0 = Input(Bool())
  val mux1_en_1 = Input(Bool())
  val out = Output(Bool())
  val outb = Output(Bool())
}

class ClkMux(sim: Boolean = false) extends BlackBox with HasBlackBoxInline {
  val io = IO(new ClkMuxIO)

  override val desiredName = "ucie_clkmux"

  if (sim) {
    setInline("ucie_clkmux.v",
      """module ucie_clkmux(
      | input in0, in1,
      | input mux0_en_0, mux0_en_1,
      | input mux1_en_0, mux1_en_1,
      | output out, outb
      |);
      | assign out = mux0_en_0 ? in0 : in1;
      |endmodule
      """.stripMargin)
  }
}

class RstSyncIO extends Bundle {
  val clk = Input(Bool())
  val rstbAsync = Input(Bool())
  val rstbSync = Output(Bool())
}

class RstSync(sim: Boolean = true) extends BlackBox with HasBlackBoxInline {
  val io = IO(new RstSyncIO)

  if (sim) {
    setInline("RstSync.v",
      """module RstSync(
      | input clk,
      | input rstbAsync,
      | output reg rstbSync
      |);
      | always @(posedge clk) begin
      |   rstbSync <= rstbAsync;
      | end
      |endmodule
      """.stripMargin)
  } else {
    setInline("RstSync.v",
      """module RstSync(
      | input clk,
      | input rstbAsync,
      | output rstbSync
      |);
      | wire rstb_int0, rstb_int1, rstb_int2;
      | b15fqn003hn1n04x5 xinst0(.clk(clk), .d(1'b1), .o(rstb_int0), .rb(rstbAsync));
      | b15lyn083hn1n04x5 xinst1(.clkb(clk), .d(rstb_int0), .o(rstb_int1), .rb(rstbAsync));
      | b15lyn003hn1n16x5 xinst2(.clk(clk), .d(rstb_int1), .o(rstb_int2), .rb(rstbAsync));
      | b15bfn000an1n80a5 xinst3(.a(rstb_int2), .o(rstbSync));
      |endmodule
      """.stripMargin)
  }
}

class Esd(sim: Boolean = false) extends BlackBox with HasBlackBoxInline {
  val io = IO(new Bundle {
    val term = Input(Bool())
  })

  override val desiredName = "ucie_esd"

  if (sim) {
    setInline("ucie_esd.v",
      """module ucie_esd(
      | input term
      |);
      |endmodule
      """.stripMargin)
  }
}

class EsdRoutable(sim: Boolean = false) extends BlackBox with HasBlackBoxInline {
  val io = IO(new Bundle {
    val term = Input(Bool())
  })

  override val desiredName = "ucie_esd_routable"

  if (sim) {
    setInline("ucie_esd_routable.v",
      """module ucie_esd_routable(
      | input term
      |);
      |endmodule
      """.stripMargin)
  }
}

class Shuffler32 extends RawModule {
  val io = IO(new Bundle {
    val din = Input(UInt(32.W))
    val dout = Output(UInt(32.W))
    val permutation = Input(Vec(32, UInt(5.W)))
  })

  io.dout := VecInit((0 until 32).map(i =>
    io.din(io.permutation(i))
  )).asUInt
}

class TxLaneDigitalCtlIO extends Bundle {
  val driver = new DriverControlIO
  val skew = new TxSkewControlIO
  val shuffler = Vec(32, UInt(5.W))
}

class PhyIO(numLanes: Int = 16) extends Bundle {
  // TX CONTROL
  // Lane control (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 1 track lane).
  val txctl = Input(Vec(numLanes + 4, new TxLaneDigitalCtlIO))
  val dllCode = Output(Vec(numLanes + 4, UInt(5.W)))
  val pllCtl = Input(new UciePllCtlIO)
  val pllOutput = Output(new UciePllDebugOutIO)
  val testPllCtl = Input(new UciePllCtlIO)
  val testPllOutput = Output(new UciePllDebugOutIO)

  // RX CONTROL
  // Termination impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 1 track lane).
  val rxctl = Input(Vec(numLanes + 4, new RxLaneCtlIO))

  // If 1, PHY uses bypass clk. If 0, PHY uses PLL clk.
  val pllBypassEn = Input(Bool())

  // TEST INTERFACE
  // =====================
  val test = new PhyToTestIO(numLanes)

  // SIDEBAND INTERFACE
  // =====================
  val sideband = new SidebandIO

  // TOP INTERFACE
  // =====================
  val top = new uciephytest.UciephyTopIO(numLanes)
}

class Phy(numLanes: Int = 16, sim: Boolean = false) extends Module {
  val io = IO(new PhyIO(numLanes))

  // TODO do we need to set pu/pd ctl to 0 when driver en is low?

  // Set up sideband
  val sbTxClk = Module(new TxDriver(sim))
  sbTxClk.io.din := io.sideband.txClk
  io.top.sbTxClk := sbTxClk.io.dout.asClock
  sbTxClk.io.ctl.pu_ctl := 63.U
  sbTxClk.io.ctl.pd_ctl := 63.U
  sbTxClk.io.ctl.en := true.B
  sbTxClk.io.ctl.en_b := false.B
  val sbTxData = Module(new TxDriver(sim))
  sbTxData.io.din := io.sideband.txData
  io.top.sbTxData := sbTxData.io.dout
  sbTxData.io.ctl.pu_ctl := 63.U
  sbTxData.io.ctl.pd_ctl := 63.U
  sbTxData.io.ctl.en := true.B
  sbTxData.io.ctl.en_b := false.B
  val ESD_sbRxClk = Module(new EsdRoutable(sim))
  val ESD_sbRxData = Module(new EsdRoutable(sim))
  ESD_sbRxClk.io.term := io.top.sbRxClk.asBool
  ESD_sbRxData.io.term := io.top.sbRxData.asBool
  io.sideband.rxClk := io.top.sbRxClk.asBool
  io.sideband.rxData := io.top.sbRxData

  // Set up clocking
  val rxClkP = Module(new RxClkLane(sim))
  rxClkP.io.clkin := io.top.rxClkP.asBool
  rxClkP.io.ctl := io.rxctl(numLanes + 1)
  val rxClkN = Module(new RxClkLane(sim))
  rxClkN.io.clkin := io.top.rxClkN.asBool
  rxClkN.io.ctl := io.rxctl(numLanes + 2)

  val pll = Module(new UciePll(sim))
  pll.io.vclk_ref := io.top.refClkP.asBool
  pll.io.vclk_refb := io.top.refClkN.asBool
  pll.io.dref_low := io.pllCtl.dref_low
  pll.io.dref_high := io.pllCtl.dref_high
  pll.io.vrdac_ref := io.top.pllRdacVref
  pll.io.dcoarse := io.pllCtl.dcoarse
  pll.io.dvco_reset := io.pllCtl.vco_reset
  pll.io.dvco_resetn := !io.pllCtl.vco_reset
  pll.io.d_digital_reset := io.pllCtl.digital_reset
  pll.io.d_accumulator_reset := io.pllCtl.d_accumulator_reset
  pll.io.d_kp := io.pllCtl.d_kp
  pll.io.d_ki := io.pllCtl.d_ki
  pll.io.d_clol := io.pllCtl.d_clol
  pll.io.d_ol_fcw := io.pllCtl.d_ol_fcw
  io.top.debug.pllClkP := pll.io.vp_out
  io.top.debug.pllClkN := pll.io.vn_out
  io.pllOutput.d_fcw_debug := pll.io.d_fcw_debug
  io.pllOutput.d_sar_debug := pll.io.d_sar_debug

  val testPll = Module(new UciePll(sim))
  testPll.io.vclk_ref := io.top.refClkP.asBool
  testPll.io.vclk_refb := io.top.refClkN.asBool
  testPll.io.dref_low := io.testPllCtl.dref_low
  testPll.io.dref_high := io.testPllCtl.dref_high
  testPll.io.vrdac_ref := io.top.pllRdacVref
  testPll.io.dcoarse := io.testPllCtl.dcoarse
  testPll.io.dvco_reset := io.testPllCtl.vco_reset
  testPll.io.dvco_resetn := !io.testPllCtl.vco_reset
  testPll.io.d_digital_reset := io.testPllCtl.digital_reset
  testPll.io.d_accumulator_reset := io.testPllCtl.d_accumulator_reset
  testPll.io.d_kp := io.testPllCtl.d_kp
  testPll.io.d_ki := io.testPllCtl.d_ki
  testPll.io.d_clol := io.testPllCtl.d_clol
  testPll.io.d_ol_fcw := io.testPllCtl.d_ol_fcw
  io.top.debug.testPllClkP := testPll.io.vp_out
  io.top.debug.testPllClkN := testPll.io.vn_out
  io.testPllOutput.d_fcw_debug := testPll.io.d_fcw_debug
  io.testPllOutput.d_sar_debug := testPll.io.d_sar_debug
  io.top.debug.testPllClkP := testPll.io.vp_out
  io.top.debug.testPllClkN := testPll.io.vn_out

  val clkMuxP = Module(new ClkMux(sim))
  clkMuxP.io.in0 := pll.io.vp_out
  clkMuxP.io.in1 := io.top.bypassClkP.asBool
  clkMuxP.io.mux0_en_0 := !io.pllBypassEn
  clkMuxP.io.mux0_en_1 := io.pllBypassEn
  clkMuxP.io.mux1_en_0 := false.B
  clkMuxP.io.mux1_en_1 := false.B
  val clkMuxN = Module(new ClkMux)
  clkMuxN.io.in0 := pll.io.vn_out
  clkMuxN.io.in1 := io.top.bypassClkN.asBool
  clkMuxN.io.mux0_en_0 := !io.pllBypassEn
  clkMuxN.io.mux0_en_1 := io.pllBypassEn
  clkMuxN.io.mux1_en_0 := false.B
  clkMuxN.io.mux1_en_1 := false.B
  val txClkP_wire = Wire(Bool())
  val txClkN_wire = Wire(Bool())
  txClkP_wire := clkMuxP.io.out
  txClkN_wire := clkMuxN.io.out
  
  val txClkDiv = Module(new ClkDiv4(sim))
  txClkDiv.io.clk := txClkP_wire
  txClkDiv.io.resetb := !io.test.txRst.asBool
  val rstSyncTx = Module(new RstSync(sim))
  rstSyncTx.io.rstbAsync := !io.test.txRst.asBool
  rstSyncTx.io.clk := txClkDiv.io.clkout_3

  val txFifo = Module(new AsyncQueue(new TxIO(numLanes), Phy.QueueParams))
  txFifo.io.enq.bits.data := io.test.tx.bits.data
  txFifo.io.enq.bits.valid := io.test.tx.bits.valid
  txFifo.io.enq.bits.clkp := io.test.tx.bits.clkp
  txFifo.io.enq.bits.clkn := io.test.tx.bits.clkn
  txFifo.io.enq.bits.track := io.test.tx.bits.track
  txFifo.io.enq.valid := io.test.tx.valid
  txFifo.io.deq_clock := txClkDiv.io.clkout_3.asClock
  io.test.tx.ready := txFifo.io.enq.ready
  txFifo.io.enq_clock := clock
  txFifo.io.enq_reset := io.test.txRst
  txFifo.io.deq_reset := !rstSyncTx.io.rstbSync.asBool
  txFifo.io.deq.ready := true.B
  
  val rxClkDiv = Module(new ClkDiv4(sim))
  rxClkDiv.io.clk := rxClkP.io.clkout
  rxClkDiv.io.resetb := !io.test.rxRst.asBool
  val rstSyncRx = Module(new RstSync(sim))
  rstSyncRx.io.rstbAsync := !io.test.rxRst.asBool
  rstSyncRx.io.clk := rxClkDiv.io.clkout_3

  val rxFifo = Module(new AsyncQueue(new RxIO(numLanes), Phy.QueueParams))
  rxFifo.io.enq.valid := true.B
  rxFifo.io.enq_reset := !rstSyncRx.io.rstbAsync.asBool
  rxFifo.io.deq_clock := clock
  rxFifo.io.enq_clock := rxClkDiv.io.clkout_3.asClock
  rxFifo.io.deq_reset := io.test.rxRst
  rxFifo.io.deq.ready := io.test.rx.ready
  io.test.rx.valid := rxFifo.io.deq.valid
  io.test.rx.bits.data := rxFifo.io.deq.bits.data
  io.test.rx.bits.valid := rxFifo.io.deq.bits.valid
  io.test.rx.bits.track := rxFifo.io.deq.bits.track

  // TODO: separate clock divider for async FIFO and its reset synchronizer
  
  for (lane <- 0 to numLanes + 3) {
    val shuffler = Module(new Shuffler32)
    when (txFifo.io.deq.valid) {
      shuffler.io.din := { if (lane < numLanes) { 
      txFifo.io.deq.bits.data(lane)
      } else if (lane == numLanes) {

      txFifo.io.deq.bits.valid
      } else if (lane == numLanes + 1) {

      txFifo.io.deq.bits.clkp
      } else if (lane == numLanes + 2) {

      txFifo.io.deq.bits.clkn
      }
      else {
      txFifo.io.deq.bits.track
      } }
    } .otherwise {
      shuffler.io.din := 0.U
    }
    shuffler.io.permutation := io.txctl(lane).shuffler

    val txLane = Module(new TxLane(sim));
    txLane.suggestName( if (lane < numLanes) {
      s"txdata$lane"
    } else if (lane == numLanes) {
      "txvalid"
    } else if (lane == numLanes + 1) {
      "txclkp"
    } else if (lane == numLanes + 2) {
      "txclkn"
    } else {
      "txtrack"
    });
    txLane.io.dll_reset := reset.asBool
    txLane.io.dll_resetb := !reset.asBool
    txLane.io.ser_resetb := !reset.asBool
    txLane.io.clkp := txClkP_wire
    txLane.io.clkn := txClkN_wire
    txLane.io.din := shuffler.io.dout.asTypeOf(txLane.io.din)
    if (lane < numLanes) {
      io.top.txData(lane) := txLane.io.dout
    } else if (lane == numLanes) {
      io.top.txValid := txLane.io.dout
    } else if (lane == numLanes + 1) {
      io.top.txClkP := txLane.io.dout.asClock
    } else if (lane == numLanes + 2) {
      io.top.txClkN := txLane.io.dout.asClock
    } else {
      io.top.txTrack := txLane.io.dout
    }
    txLane.io.ctl.driver := io.txctl(lane).driver
    txLane.io.ctl.skew := io.txctl(lane).skew
    io.dllCode(lane) := txLane.io.dll_code
  }

  // TODO async FIFO clock and reset

  for (lane <- 0 to numLanes + 1) {
    val rxLane = Module(new RxDataLane(sim))
    if (lane < numLanes) {
      rxLane.suggestName(s"rxdata$lane")
      rxLane.io.din := io.top.rxData(lane)
      rxFifo.io.enq.bits.data(lane) := rxLane.io.dout
      rxLane.io.ctl := io.rxctl(lane)
    } else if (lane == numLanes) {
      rxLane.suggestName(s"rxvalid")
      rxLane.io.din := io.top.rxValid
      rxFifo.io.enq.bits.valid := rxLane.io.dout
      rxLane.io.ctl := io.rxctl(lane)
    } else {
      rxLane.suggestName(s"rxtrack")
      rxLane.io.din := io.top.rxTrack
      rxFifo.io.enq.bits.track := rxLane.io.dout
      rxLane.io.ctl := io.rxctl(numLanes + 3)
    }
    rxLane.io.clk := rxClkP.io.clkout.asClock
    rxLane.io.resetb := !reset.asBool
  }
}

class ClkDiv4IO extends Bundle {
  val clk = Input(Bool())
  val resetb = Input(Bool())
  val clkout_0 = Output(Bool())
  val clkout_1 = Output(Bool())
  val clkout_2 = Output(Bool())
  val clkout_3 = Output(Bool())
}

class ClkDiv4(sim: Boolean = false) extends BlackBox with HasBlackBoxInline {
  val io = IO(new ClkDiv4IO)

  override val desiredName = "clock_div_4_with_rst_sync"

  if (sim) {
    setInline("clock_div_4_with_rst_sync.v",
      """module clock_div_4_with_rst_sync(
      |  input clk, resetb,
      |  output reg clkout_0, clkout_1, clkout_2, clkout_3
      |);
      |  always @(posedge clk) begin
      |    if (resetb) begin
      |      clkout_0 <= 1'b0;
      |      clkout_1 <= 1'b0;
      |      clkout_2 <= 1'b0;
      |      clkout_3 <= 1'b0;
      |    end else begin
      |    	clkout_0 <= ~clkout_0;
      |    end
      |  end
      |  always @(posedge clkout_0) begin
      |    clkout_1 <= ~clkout_1;
      |  end
      |  always @(posedge clkout_1) begin
      |    clkout_2 <= ~clkout_2;
      |  end
      |  always @(posedge clkout_2) begin
      |    clkout_3 <= ~clkout_3;
      |  end
      |endmodule
      """.stripMargin)
  }
}
