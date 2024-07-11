package uciephytest.phy

import chisel3._
import chisel3.util._
import chisel3.experimental.dataview._
import freechips.rocketchip.util.{AsyncQueue, AsyncQueueParams}

object Phy {
  val DigitalToPhyBitsRatio = 2
  val SerdesRatio = 16
  val DigitalBitsPerCycle = DigitalToPhyBitsRatio * SerdesRatio
  val QueueParams = AsyncQueueParams()
}

class PhyToTestIO(numLanes: Int = 2) extends Bundle {
  // Data passed in by the test RTL this cycle.
  val txTransmitData = Vec(numLanes, Flipped(DecoupledIO(UInt(Phy.DigitalBitsPerCycle.W))))
  // Valid passed in by the test RTL this cycle.
  val txTransmitValid = Flipped(DecoupledIO(UInt(Phy.DigitalBitsPerCycle.W)))
  // Data to pass to the test RTL this cycle.
  val rxReceiveData = Vec(numLanes, DecoupledIO(UInt(Phy.DigitalBitsPerCycle.W)))
  // Valid to pass to the test RTL this cycle.
  val rxReceiveValid = DecoupledIO(UInt(Phy.DigitalBitsPerCycle.W))
}

class RefClkRxIO extends Bundle {
  val vip = Input(Bool())
  val vin = Input(Bool())
  val vop = Output(Bool())
  val von = Output(Bool())
}

class RefClkRx extends BlackBox {
  val io = IO(new RefClkRxIO)

  override val desiredName = "refclkrx"

  // io.vop := io.vin
  // io.von := io.vip
}

class ClkMuxIO extends Bundle {
  val in0 = Input(Bool())
  val in1 = Input(Bool())
  val mux0_en0 = Input(Bool())
  val mux0_en1 = Input(Bool())
  val mux1_en0 = Input(Bool())
  val mux1_en1 = Input(Bool())
  val out = Output(Bool())
  val outb = Output(Bool())
}

class ClkMux extends BlackBox with HasBlackBoxInline {
  val io = IO(new ClkMuxIO)

  override val desiredName = "clkmux_wrapper"

  setInline("clkmux_wrapper.v",
    """module clkmux_wrapper (
      |    input  in0,
      |    input  in1,
      |    input mux0_en0,
      |    input mux0_en1,
      |    input mux1_en0, 
      |    input mux1_en1, 
      |    output out
      |    output outb
      |);
      |    clkmux clkmux_inner(
      |       .\in<0> (in0),
      |       .\in<1> (in1),
      |       .\mux0_en<0> (mux0_en0),
      |       .\mux0_en<1> (mux0_en1),
      |       .\mux1_en<0> (mux1_en0),
      |       .\mux1_en<1> (mux1_en1),
      |       .out (out),
      |       .outb (outb)
      |    );
      |endmodule
    """.stripMargin)
}

class RstSyncIO extends Bundle {
  val rstbSync = Output(Reset())
}

class RstSync extends Module {
  val io = IO(new RstSyncIO)

  // TODO: Fix behavioral model
  io.rstbSync := reset
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
  val clockingMiscCtl = Input(Vec(numLanes + 1, UInt(26.W)))

  // RX CONTROL
  // =====================
  // Termination impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes).
  val terminationCtl = Input(Vec(numLanes + 3, UInt(6.W))) 
  // Reference voltage control.
  val vrefCtl = Input(Vec(numLanes + 1, UInt(7.W))) 

  // TEST INTERFACE
  // =====================
  val test = new PhyToTestIO(numLanes)

  // TOP INTERFACE
  // =====================
  val top = new uciephytest.UciephyTopIO(numLanes)
}

class Phy(numLanes: Int = 2) extends Module {
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

  // Set up clocking
  val rxClkP = Module(new RxClk)
  rxClkP.io.clkin := io.top.rxClkP.asBool
  rxClkP.io.zctl := io.terminationCtl(numLanes + 1).asTypeOf(rxClkP.io.zctl)
  val rxClkN = Module(new RxClk)
  rxClkN.io.clkin := io.top.rxClkN.asBool
  rxClkN.io.zctl := io.terminationCtl(numLanes + 2).asTypeOf(rxClkN.io.zctl)
  val refClkRx = Module(new RefClkRx)
  refClkRx.io.vip := io.top.refClkP.asBool
  refClkRx.io.vin := io.top.refClkN.asBool
  val clkMuxP = Module(new ClkMux)
  clkMuxP.io.in0 := refClkRx.io.vop
  clkMuxP.io.in1 := false.B
  clkMuxP.io.mux0_en0 := true.B
  clkMuxP.io.mux0_en1 := false.B
  clkMuxP.io.mux1_en0 := false.B
  clkMuxP.io.mux1_en1 := false.B
  val clkMuxN = Module(new ClkMux)
  clkMuxN.io.in0 := refClkRx.io.von
  clkMuxN.io.in1 := false.B
  clkMuxN.io.mux0_en0 := true.B
  clkMuxN.io.mux0_en1 := false.B
  clkMuxN.io.mux1_en0 := false.B
  clkMuxN.io.mux1_en1 := false.B
  val txClkP_wire = Wire(Bool())
  val txClkN_wire = Wire(Bool())
  txClkP_wire := clkMuxP.io.out
  txClkN_wire := clkMuxN.io.out
  val txClkP = Module(new TxDriver)
  txClkP.io.din := txClkP_wire
  io.top.txClkP := txClkP.io.dout.asClock
  connectDriverCtl(txClkP.io.driver_ctl, numLanes + 1)
  val txClkN = Module(new TxDriver)
  txClkN.io.din := txClkN_wire
  io.top.txClkN := txClkN.io.dout.asClock
  connectDriverCtl(txClkN.io.driver_ctl, numLanes + 2)

  for (lane <- 0 to numLanes) {
    val txLane = Module(new TxLaneDll)
    txLane.io.vinp := txClkP_wire
    txLane.io.vinn := txClkN_wire
    txLane.io.resetb := !reset.asBool
    connectDriverCtl(txLane.io.driver_ctl, lane)
    connectClockingCtl(txLane.io.clocking_ctl, lane)

    // TODO: double check reset sense.
    val rstSyncTx = withClockAndReset(txLane.io.divclk, !reset.asBool) {
      Module(new RstSync)
    }
    val currentFifoTx  = withClockAndReset(txLane.io.divclk, reset.asAsyncReset) {
      RegInit(0.U(log2Ceil(Phy.DigitalToPhyBitsRatio).W))
    }

    txLane.io.din := (0.U).asTypeOf(txLane.io.din)

    val txDigitalLane = if (lane < numLanes) { io.test.txTransmitData(lane) } else { io.test.txTransmitValid }
    val txFifos = (0 until Phy.DigitalToPhyBitsRatio).map((i: Int) => {
      val fifo = Module(new AsyncQueue(UInt(Phy.SerdesRatio.W), Phy.QueueParams))
      fifo.io.enq.bits := txDigitalLane.bits(Phy.SerdesRatio*(i+1) - 1, Phy.SerdesRatio*i)
      fifo.io.enq.valid := txDigitalLane.valid && txDigitalLane.ready
      fifo.io.enq_clock := clock
      fifo.io.enq_reset := reset
      fifo.io.deq_clock := txLane.io.divclk
      fifo.io.deq_reset := !rstSyncTx.io.rstbSync.asBool
      fifo.io.deq.ready := currentFifoTx === i.U
      when (fifo.io.deq.ready && fifo.io.deq.valid) {
        txLane.io.din := fifo.io.deq.bits.asTypeOf(txLane.io.din)
        currentFifoTx := (currentFifoTx + 1.U) % Phy.DigitalToPhyBitsRatio.U
      }
      fifo
    })
    txDigitalLane.ready := txFifos.map(_.io.enq.ready).reduce(_ && _)


    if (lane < numLanes) {
      io.top.txData(lane) := txLane.io.dout
    } else {
      io.top.txValid := txLane.io.dout
    }

    // TODO: Change to use top-level clkp and clkn
    val rxLane = Module(new RxLane)
    rxLane.io.clk := rxClkP.io.clkout.asClock
    rxLane.io.clkb := rxClkN.io.clkout.asClock
    rxLane.io.zctl := io.terminationCtl(lane).asTypeOf(rxLane.io.zctl)
    rxLane.io.resetb := !reset.asBool
    rxLane.io.vref_ctl := io.vrefCtl(lane)

    // TODO: double check reset sense.
    val rstSyncRx = withClockAndReset(rxLane.io.divclk, !reset.asBool) {
      Module(new RstSync)
    }
    val currentFifoRx  = withClockAndReset(rxLane.io.divclk, reset.asAsyncReset) {
      RegInit(0.U(log2Ceil(Phy.DigitalToPhyBitsRatio).W))
    }

    val rxDigitalLane = if (lane < numLanes) { io.test.rxReceiveData(lane) } else { io.test.rxReceiveValid }
    val rxDigitalLaneBits = Wire(Vec(Phy.DigitalToPhyBitsRatio, UInt(Phy.SerdesRatio.W)))
    val rxFifos = (0 until Phy.DigitalToPhyBitsRatio).map((i: Int) => {
      val fifo = Module(new AsyncQueue(UInt(Phy.SerdesRatio.W), Phy.QueueParams))
      rxDigitalLaneBits(i) := fifo.io.deq.bits
      fifo.io.deq.ready := rxDigitalLane.valid && rxDigitalLane.ready
      fifo.io.deq_clock := clock
      fifo.io.deq_reset := reset
      fifo.io.enq_clock := rxLane.io.divclk
      fifo.io.enq_reset := !rstSyncRx.io.rstbSync.asBool
      fifo.io.enq.bits := rxLane.io.dout.asTypeOf(UInt(Phy.SerdesRatio.W))
      fifo.io.enq.valid := currentFifoRx === i.U
      when (currentFifoRx === i.U && fifo.io.enq.ready) {
        currentFifoRx := (currentFifoRx + 1.U) % Phy.DigitalToPhyBitsRatio.U
      } 
      fifo
    })
    rxDigitalLane.bits := rxDigitalLaneBits.asTypeOf(rxDigitalLane.bits)
    rxDigitalLane.valid := rxFifos.map(_.io.deq.valid).reduce(_ && _)

    if (lane < numLanes) {
      rxLane.io.din := io.top.rxData(lane)
    } else {
      rxLane.io.din := io.top.rxValid
    }
  }
}
