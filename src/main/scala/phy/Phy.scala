package uciephytest.phy

import chisel3._
import chisel3.util._
import chisel3.experimental.dataview._
import freechips.rocketchip.util.{AsyncQueue, AsyncQueueParams}

object Phy {
  val DigitalToPhyBitsRatio = 2
  val SerdesRatio = 16
  val DigitalBitsPerCycle = DigitalToPhyBitsRatio * SerdesRatio
  val QueueParams = AsyncQueueParams(depth = 32)
}

class MainbandIO(numLanes: Int = 2) extends Bundle  {
  val data = Vec(numLanes, UInt(Phy.DigitalBitsPerCycle.W))
  val valid = UInt(Phy.DigitalBitsPerCycle.W)
}

class PhyToTestIO(numLanes: Int = 2) extends Bundle {
  val tx = Flipped(DecoupledIO(new MainbandIO(numLanes)))
  val rx = DecoupledIO(new MainbandIO(numLanes))
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

class Shuffler16 extends RawModule {
  val io = IO(new Bundle {
    val din = Input(UInt(16.W))
    val dout = Output(UInt(16.W))
    val permutation = Input(Vec(16, UInt(4.W)))
  })

  io.dout := VecInit((0 until 16).map(i =>
    io.din(io.permutation(i))
  )).asUInt
}

class Ser32to16 extends Module {
  val io = IO(new Bundle {
    val din = Input(UInt(32.W))
    val dout = Output(UInt(16.W))
    val divClock = Output(Bool())
  })
  val divClock = RegInit(false.B)
  divClock := !divClock
  io.divClock := divClock

  val shiftReg = RegInit(0.U(32.W))
  shiftReg := shiftReg >> 16.U
  when (divClock) {
    shiftReg := io.din
  }

  io.divClock := divClock
  io.dout := shiftReg(15, 0)
}

class Des16to32 extends Module {
  val io = IO(new Bundle {
    val din = Input(UInt(16.W))
    val dout = Output(UInt(32.W))
    val divClock = Output(Bool())
  })
  val divClock = RegInit(true.B)
  divClock := !divClock

  val shiftReg = RegInit(0.U(32.W))
  shiftReg := shiftReg << 16.U | Reverse(io.din)

  val outputReg = withClock(divClock.asClock) {
    RegNext(Reverse(shiftReg))
  }

  io.divClock := divClock
  io.dout := outputReg
}

class PhyIO(numLanes: Int = 2) extends Bundle {
  // TX CONTROL
  // =====================
  // Pull-up impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 2 ref clock out).
  val driverPuCtl = Input(Vec(numLanes + 5, UInt(6.W))) 
  // Pull-down impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 2 ref clock out).
  val driverPdCtl = Input(Vec(numLanes + 5, UInt(6.W))) 
  // Driver enable signal per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 2 ref clock out). 
  // When low, the driver enters a high-Z state.
  val driverEn = Input(Vec(numLanes + 5, Bool()))
  // Misc clocking control per lane (`numLanes` data lanes, 1 valid lane). 
  val clockingPiCtl = Input(Vec(numLanes + 1, UInt(52.W)))
  // Misc clocking control per lane (`numLanes` data lanes, 1 valid lane). 
  val clockingMiscCtl = Input(Vec(numLanes + 1, UInt(28.W)))
  /// Control for TX shuffler.
  val shufflerCtl = Input(Vec(numLanes + 1, Vec(16, UInt(4.W))))

  // RX CONTROL
  // =====================
  // Termination impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes).
  val terminationCtl = Input(Vec(numLanes + 3, UInt(6.W))) 
  // Reference voltage control.
  val vrefCtl = Input(Vec(numLanes + 1, UInt(7.W))) 

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

class Phy(numLanes: Int = 2, sim: Boolean = false) extends Module {
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
  val rxClkP = Module(new RxClk(sim))
  rxClkP.io.clkin := io.top.rxClkP.asBool
  rxClkP.io.zctl := io.terminationCtl(numLanes + 1).asTypeOf(rxClkP.io.zctl)
  val rxClkN = Module(new RxClk(sim))
  rxClkN.io.clkin := io.top.rxClkN.asBool
  rxClkN.io.zctl := io.terminationCtl(numLanes + 2).asTypeOf(rxClkN.io.zctl)
  val refClkRx = Module(new RefClkRx(sim))
  refClkRx.io.vip := io.top.refClkP.asBool
  refClkRx.io.vin := io.top.refClkN.asBool
  val ESD_refClkP = Module(new Esd(sim))
  val ESD_refClkN = Module(new Esd(sim))
  ESD_refClkP.io.term := io.top.refClkP.asBool
  ESD_refClkN.io.term := io.top.refClkN.asBool
  val clkMuxP = Module(new ClkMux(sim))
  clkMuxP.io.in0 := refClkRx.io.vop
  clkMuxP.io.in1 := false.B
  clkMuxP.io.mux0_en_0 := true.B
  clkMuxP.io.mux0_en_1 := false.B
  clkMuxP.io.mux1_en_0 := false.B
  clkMuxP.io.mux1_en_1 := false.B
  val clkMuxN = Module(new ClkMux)
  clkMuxN.io.in0 := refClkRx.io.von
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
  connectDriverCtl(txClkP.io.driver_ctl, numLanes + 1)
  val txClkN = Module(new TxDriver(sim))
  txClkN.io.din := txClkN_wire
  io.top.txClkN := txClkN.io.dout.asClock
  connectDriverCtl(txClkN.io.driver_ctl, numLanes + 2)
  
  val rstSyncTx = Module(new RstSync(sim))
  rstSyncTx.io.rstbAsync := !io.test.txRst.asBool

  val txFifo = Module(new AsyncQueue(new MainbandIO(numLanes), Phy.QueueParams))
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

  val rxFifo = Module(new AsyncQueue(new MainbandIO(numLanes), Phy.QueueParams))
  rxFifo.io.enq.valid := true.B
  rxFifo.io.enq_reset := !rstSyncRx.io.rstbAsync.asBool
  rxFifo.io.deq_clock := clock
  rxFifo.io.deq_reset := io.test.rxRst
  rxFifo.io.deq.ready := io.test.rx.ready
  io.test.rx.valid := rxFifo.io.deq.valid
  io.test.rx.bits.data := rxFifo.io.deq.bits.data
  io.test.rx.bits.valid := rxFifo.io.deq.bits.valid

  for (lane <- 0 to numLanes) {
    val txLane = Module(new TxLaneDll(sim))
    txLane.io.vinp := txClkP_wire
    txLane.io.vinn := txClkN_wire
    txLane.io.reset := reset.asBool
    connectDriverCtl(txLane.io.driver_ctl, lane)
    connectClockingCtl(txLane.io.clocking_ctl, lane)

    val serializer = withClockAndReset(txLane.io.divclk, reset.asAsyncReset) { Module(new Ser32to16) }
    when (txFifo.io.deq.valid) {
      serializer.io.din := { if (lane < numLanes) { txFifo.io.deq.bits.data(lane) } else { txFifo.io.deq.bits.valid } }
    } .otherwise {
      serializer.io.din := 0.U
    }
    val shuffler = Module(new Shuffler16)
    shuffler.io.din := serializer.io.dout
    shuffler.io.permutation := io.shufflerCtl(lane).asTypeOf(shuffler.io.permutation)
    txLane.io.din := shuffler.io.dout.asTypeOf(txLane.io.din)
    // Use the first 32:16 serializer's divided clock to clock the async FIFO and its reset synchronizer.
    if (lane == 0) {
      rstSyncTx.io.clk := serializer.io.divClock
      txFifo.io.deq_clock := serializer.io.divClock.asClock
    }

    if (lane < numLanes) {
      io.top.txData(lane) := txLane.io.dout
    } else {
      io.top.txValid := txLane.io.dout
    }

    val rxLane = Module(new RxLane(sim))
    rxLane.io.clk := rxClkP.io.clkout.asClock
    rxLane.io.clkb := rxClkN.io.clkout.asClock
    rxLane.io.zctl := io.terminationCtl(lane).asTypeOf(rxLane.io.zctl)
    rxLane.io.resetb := !reset.asBool
    rxLane.io.vref_ctl := io.vrefCtl(lane)

    val deserializer = withClockAndReset(rxLane.io.divclk, reset.asAsyncReset) { Module(new Des16to32) }
    if (lane < numLanes) { 
      rxFifo.io.enq.bits.data(lane) := deserializer.io.dout
    } else { 
      rxFifo.io.enq.bits.valid := deserializer.io.dout
    }
    deserializer.io.din := rxLane.io.dout.asTypeOf(deserializer.io.din)
    // Use the first 16:32 deserializer's divided clock to clock the async FIFO and its reset synchronizer.
    if (lane == 0) {
      rstSyncRx.io.clk := deserializer.io.divClock
      rxFifo.io.enq_clock := deserializer.io.divClock.asClock
    }

    if (lane < numLanes) {
      rxLane.io.din := io.top.rxData(lane)
    } else {
      rxLane.io.din := io.top.rxValid
    }
  }
}
