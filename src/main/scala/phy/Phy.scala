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

class RefClkRx extends RawModule {
  val io = IO(new RefClkRxIO)

  override val desiredName = "refclkrx"
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
  val driverPuCtl = Input(Vec(numLanes + 5, UInt(48.W))) 
  // Pull-down impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 2 ref clock out).
  val driverPdCtl = Input(Vec(numLanes + 5, UInt(48.W))) 
  // Driver enable signal per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 2 ref clock out). 
  // When low, the driver enters a high-Z state.
  val driverEn = Input(Vec(numLanes + 5, Bool()))
  // Phase control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes, 2 ref clock out). 
  val phaseCtl = Input(Vec(numLanes + 5, UInt(8.W)))

  // RX CONTROL
  // =====================
  // Termination impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes).
  val terminationCtl = Input(Vec(numLanes + 3, UInt(64.W))) 

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
      driver_ctl.pd_ctlb := (~io.driverPdCtl(lane)).asTypeOf(driver_ctl.pd_ctlb)
      driver_ctl.driver_en := true.B
      driver_ctl.driver_en_b := false.B
    } .otherwise {
      driver_ctl.pu_ctl := 0.U(48.W).asTypeOf(driver_ctl.pu_ctl)
      driver_ctl.pd_ctlb := (~0.U(48.W)).asTypeOf(driver_ctl.pd_ctlb)
      driver_ctl.driver_en := false.B
      driver_ctl.driver_en_b := true.B
    }
  }

  // Set up clocking
  val rxClkP = Module(new RxClk)
  rxClkP.io.clkin := io.top.rxClkP
  rxClkP.io.zctl := io.terminationCtl(numLanes + 1).asTypeOf(rxClkP.io.zctl)
  val rxClkN = Module(new RxClk)
  rxClkN.io.clkin := io.top.rxClkN
  rxClkN.io.zctl := io.terminationCtl(numLanes + 2).asTypeOf(rxClkN.io.zctl)
  val refClkRx = Module(new RefClkRx)
  refClkRx.io.vip := io.top.refClkP
  refClkRx.io.vin := io.top.refClkN
  val txClk = Module(new TxClk)
  txClk.io.clk := refClkRx.io.vop
  txClk.io.clkb := refClkRx.io.von
  io.top.txClkP := txClk.io.clkout
  io.top.txClkN := txClk.io.clkoutb
  for (i <- 0 to 1) {
    connectDriverCtl(txClk.io.driver_ctl(i), numLanes + 1 + i)
  }
  // Connect reference clock to pad drivers
  val refClkPDriver = Module(new TxDriver)
  refClkPDriver.io.din := refClkRx.io.vop
  io.top.clkRxOutP := refClkPDriver.io.dout
  val refClkNDriver = Module(new TxDriver)
  refClkNDriver.io.din := refClkRx.io.von
  io.top.clkRxOutN := refClkNDriver.io.dout
  for (i <- 0 to 1) {
    val lane = numLanes + 3 + i
    val driver = if (i == 0) { refClkPDriver } else { refClkNDriver }
    connectDriverCtl(driver.io.driver_ctl, lane)
  }

  for (lane <- 0 to numLanes) {
    val txLane = Module(new TxLane)
    txLane.io.clk := refClkRx.io.vop
    txLane.io.clkb := refClkRx.io.von
    txLane.io.resetb := !reset.asBool
    connectDriverCtl(txLane.io.driver_ctl, lane)

    // TODO: double check reset sense.
    val rstSyncTx = withClockAndReset(txLane.io.divclk, !reset.asBool) {
      Module(new RstSync)
    }
    val currentFifoTx  = withClockAndReset(txLane.io.divclk, reset.asAsyncReset) {
      RegInit(0.U(log2Ceil(Phy.DigitalToPhyBitsRatio).W))
    }

    txLane.io.din := 0.U

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
        txLane.io.din := fifo.io.deq.bits
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
    rxLane.io.clk := rxClkP.io.clkout
    rxLane.io.clkb := rxClkN.io.clkout
    rxLane.io.zctl := io.terminationCtl(lane).asTypeOf(rxLane.io.zctl)
    rxLane.io.resetb := !reset.asBool

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
