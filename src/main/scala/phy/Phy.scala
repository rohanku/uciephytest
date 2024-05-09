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
  // Pull-up impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes).
  val driverPuCtl = Input(Vec(numLanes + 3, UInt(8.W))) 
  // Pull-down impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes).
  val driverPdCtl = Input(Vec(numLanes + 3, UInt(8.W))) 
  // Driver enable signal per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes). 
  // When low, the driver enters a high-Z state.
  val driverEn = Input(Vec(numLanes + 3, Bool()))
  // Phase control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes). 
  val phaseCtl = Input(Vec(numLanes + 3, UInt(8.W)))

  // RX CONTROL
  // =====================
  // Termination impedance control per lane (`numLanes` data lanes, 1 valid lane, 2 clock lanes).
  val terminationCtl = Input(Vec(numLanes + 3, UInt(8.W))) 

  // TEST INTERFACE
  // =====================
  val test = new PhyToTestIO(numLanes)

  // TOP INTERFACE
  // =====================
  val top = new uciephytest.UciephyTopIO(numLanes)
}

class Phy(numLanes: Int = 2) extends Module {
  val io = IO(new PhyIO(numLanes))


  val terminationCtl = Wire(Vec(numLanes + 3, UInt(64.W)))
  for (lane <- 0 until numLanes + 3) {
    val decoder = Module(new Decoder(64))
    decoder.io.binary := io.terminationCtl(lane)
    terminationCtl(lane) := decoder.io.thermometer
  }

  // Set up clocking
  val rxClkP = Module(new RxClk)
  rxClkP.io.clkin := io.top.rxClkP
  rxClkP.io.zctl := terminationCtl(numLanes + 1).asTypeOf(new TerminationControlIO)

  for (lane <- 0 to numLanes) {

    val txLane = withClock(io.top.refClkP) { Module(new TxLane) }
    txLane.io.driverPuCtl := io.driverPuCtl(lane)
    txLane.io.driverPdCtl := io.driverPdCtl(lane)
    txLane.io.driverEn := io.driverEn(lane)
    txLane.io.phaseCtl := io.phaseCtl(lane)

    // TODO: double check reset sense.
    val rstSyncTx = withClockAndReset(txLane.io.divClock, !reset.asBool) {
      Module(new RstSync)
    }
    val currentFifoTx  = withClockAndReset(txLane.io.divClock, reset.asAsyncReset) {
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
      fifo.io.deq_clock := txLane.io.divClock
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

    // TODO: double check reset sense.
    val rstSyncRx = withClockAndReset(rxLane.io.divClock, !reset.asBool) {
      Module(new RstSync)
    }
    val currentFifoRx  = withClockAndReset(rxLane.io.divClock, reset.asAsyncReset) {
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
      fifo.io.enq_clock := rxLane.io.divClock
      fifo.io.enq_reset := !rstSyncRx.io.rstbSync.asBool
      fifo.io.enq.bits := rxLane.io.dout
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
