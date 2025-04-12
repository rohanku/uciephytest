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
  val shuffler = Input(Vec(32, UInt(5.W)))
}

class PhyIO(numLanes: Int = 16) extends Bundle {
  // TX CONTROL
  // Lane control (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 1 track lane).
  val txctl = Input(Vec(numLanes + 4, new TxLaneDigitalCtlIO))

  // RX CONTROL
  // Termination impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes).
  val rxctl = Input(Vec(numLanes + 3, new RxLaneCtlIO))

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

  val connectDriverCtl = (driver_ctl: DriverControlIO, lane: Int) => {
    when (io.driverEn(lane)) {
      driver_ctl.pu_ctl := io.driverPuCtl(lane).asTypeOf(driver_ctl.pu_ctl)
      driver_ctl.pd_ctl := io.driverPdCtl(lane).asTypeOf(driver_ctl.pd_ctl)
      driver_ctl.en := true.B
      driver_ctl.en_b := false.B
    } .otherwise {
      driver_ctl.pu_ctl := 0.U(6.W).asTypeOf(driver_ctl.pu_ctl)
      driver_ctl.pd_ctl := 0.U(6.W).asTypeOf(driver_ctl.pd_ctl)
      driver_ctl.en := false.B
      driver_ctl.en_b := true.B
    }
  }

  val connectClockingCtl = (clocking_ctl: ClockingControlDllIO, lane: Int) => {
    clocking_ctl.pi := io.clockingPiCtl(lane).asTypeOf(clocking_ctl.pi)
    clocking_ctl.misc := io.clockingMiscCtl(lane).asTypeOf(clocking_ctl.misc)
  }

  // Set up sideband
  val sbTxClk = Module(new TxDriver(sim))
  sbTxClk.io.din := io.sideband.txClk
  io.top.sbTxClk := sbTxClk.io.dout.asClock
  sbTxClk.io.driver_ctl.pu_ctl := 63.U
  sbTxClk.io.driver_ctl.pd_ctl := 63.U
  sbTxClk.io.driver_ctl.en := true.B
  sbTxClk.io.driver_ctl.en_b := false.B
  val sbTxData = Module(new TxDriver(sim))
  sbTxData.io.din := io.sideband.txData
  io.top.sbTxData := sbTxData.io.dout
  sbTxData.io.driver_ctl.pu_ctl := 63.U
  sbTxData.io.driver_ctl.pd_ctl := 63.U
  sbTxData.io.driver_ctl.en := true.B
  sbTxData.io.driver_ctl.en_b := false.B
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
  val rxClkN = Module(new RxClk(sim))
  rxClkN.io.clkin := io.top.rxClkN.asBool
  rxClkN.io.ctl := io.rxctl(numLanes + 2)
  val refClkRx = Module(new RefClkRx(sim))
  refClkRx.io.vip := io.top.refClkP.asBool
  refClkRx.io.vin := io.top.refClkN.asBool
  val ESD_refClkP = Module(new Esd(sim))
  val ESD_refClkN = Module(new Esd(sim))
  ESD_refClkP.io.term := io.top.refClkP.asBool
  ESD_refClkN.io.term := io.top.refClkN.asBool

  val pll = Module(new SsdpllSingleSampling)
  pll.vclk_ref := refClkRx.io.vop
  pll.vclk_refb := refClkRx.io.von

  val clkMuxP = Module(new ClkMux(sim))
  clkMuxP.io.in0 := pll.io.vp_out
  clkMuxP.io.in1 := false.B
  clkMuxP.io.mux0_en_0 := true.B
  clkMuxP.io.mux0_en_1 := false.B
  clkMuxP.io.mux1_en_0 := false.B
  clkMuxP.io.mux1_en_1 := false.B
  val clkMuxN = Module(new ClkMux)
  clkMuxN.io.in0 := pll.io.vn_out
  clkMuxN.io.in1 := false.B
  clkMuxN.io.mux0_en_0 := true.B
  clkMuxN.io.mux0_en_1 := false.B
  clkMuxN.io.mux1_en_0 := false.B
  clkMuxN.io.mux1_en_1 := false.B
  val txClkP_wire = Wire(Bool())
  val txClkN_wire = Wire(Bool())
  txClkP_wire := clkMuxP.io.out
  txClkN_wire := clkMuxN.io.out
  val txClkP = Module(new TxDriver(sim))
  txClkP.io.din := txClkP_wire
  io.top.txClkP := txClkP.io.dout.asClock
  txClkP.io.ctl.driver := io.txctl(numLanes + 1).driver
  txClkP.io.ctl.skew := io.txctl(numLanes + 1).skew
  val txClkN = Module(new TxDriver(sim))
  txClkN.io.din := txClkN_wire
  io.top.txClkN := txClkN.io.dout.asClock
  txClkN.io.ctl.driver := io.txctl(numLanes + 2).driver
  txClkN.io.ctl.skew := io.txctl(numLanes + 2).skew
  
  val rstSyncTx = Module(new RstSync(sim))
  rstSyncTx.io.rstbAsync := !io.test.txRst.asBool

  val txFifo = Module(new AsyncQueue(new TxIO(numLanes), Phy.QueueParams))
  txFifo.io.enq.bits.data := io.test.tx.bits.data
  txFifo.io.enq.bits.valid := io.test.tx.bits.valid
  txFifo.io.enq.valid := io.test.tx.valid
  io.test.tx.ready := txFifo.io.enq.ready
  txFifo.io.enq_clock := clock
  txFifo.io.enq_reset := io.test.txRst
  txFifo.io.deq_reset := !rstSyncTx.io.rstbSync.asBool
  txFifo.io.deq.ready := true.B
  
  val rstSyncRx = Module(new RstSync(sim))
  rstSyncRx.io.rstbAsync := !io.test.rxRst.asBool

  val rxFifo = Module(new AsyncQueue(new RxIO(numLanes), Phy.QueueParams))
  rxFifo.io.enq.valid := true.B
  rxFifo.io.enq_reset := !rstSyncRx.io.rstbAsync.asBool
  rxFifo.io.deq_clock := clock
  rxFifo.io.deq_reset := io.test.rxRst
  rxFifo.io.deq.ready := io.test.rx.ready
  io.test.rx.valid := rxFifo.io.deq.valid
  io.test.rx.bits.data := rxFifo.io.deq.bits.data
  io.test.rx.bits.valid := rxFifo.io.deq.bits.valid

  // TODO: separate clock divider for async FIFO and its reset synchronizer
  
  for (lane <- 0 to numLanes) {
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
      io.top.txClkP := txLane.io.dout
    } else if (lane == numLanes + 2) {
      io.top.txClkN := txLane.io.dout
    } else {
      io.top.txTrack := txLane.io.dout
    }
    txLane.io.ctl.driver := io.txctl(lane).driver
    txLane.io.ctl.skew := io.txctl(lane).skew
  }

  // TODO async FIFO clock and reset

  for (lane <- 0 to numLanes) {
    val rxLane = Module(new RxDataLane(sim))
    if (lane < numLanes) {
      rxLane.suggestName(s"rxdata$lane")
      rxLane.io.din := io.top.rxData(lane)
      rxFifo.io.enq.bits.data(lane) := rxLane.io.dout
    } else {
      rxLane.suggestName(s"rxvalid")
      rxLane.io.din := io.top.rxValid
      rxFifo.io.enq.bits.valid := rxLane.io.dout
    }
    rxLane.io.clk := rxClkP.io.clkout
    rxLane.io.resetb := !reset.asBool
    rxLane.io.ctl := io.rxctl(lane)
  }
}
