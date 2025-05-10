package uciephytest

import chisel3._
import chisel3.util._
import chisel3.util.random._
import chisel3.experimental.BundleLiterals._
import chisel3.experimental.VecLiterals._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{
  BaseSubsystem,
  PBUS,
  SBUS,
  CacheBlockBytes,
  TLBusWrapperLocation
}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{
  HasRegMap,
  RegField,
  RegWriteFn,
  RegReadFn,
  RegFieldDesc
}
import freechips.rocketchip.tilelink._
import uciephytest.phy.{
  TxDriver,
  TxLane,
  ClkDiv4,
  UcieRstSync,
  Shuffler32,
  ClkRx,
  DriverControlIO,
  Phy,
  TxLaneDigitalCtlIO,
  DiffBuffer,
  SingleEndedBuffer
}
import freechips.rocketchip.util.{AsyncQueue}
import testchipip.soc.{OBUS}
import edu.berkeley.cs.ucie.digital.tilelink._
import edu.berkeley.cs.ucie.digital.interfaces.{FdiParams, RdiParams, AfeParams}
import edu.berkeley.cs.ucie.digital.protocol.{ProtocolLayerParams}
import edu.berkeley.cs.ucie.digital.sideband.{SidebandParams}
import edu.berkeley.cs.ucie.digital.logphy.{
  LinkTrainingParams,
  TransmitPattern,
  RegisterRWIO,
  RegisterRW
}

case class UciephyCommonParams(
    bitCounterWidth: Int = 64,
    address: BigInt = 0x4000,
    managerWhere: TLBusWrapperLocation = PBUS,
    sim: Boolean = false
)

case object UciephyCommonKey
    extends Field[Option[Seq[UciephyCommonParams]]](None)

class PhyDebugBumpsIO extends Bundle {
  val pllClkP = Output(Bool())
  val pllClkN = Output(Bool())
  val testPllClkP = Output(Bool())
  val testPllClkN = Output(Bool())
  val rxClk = Output(Bool())
  val rxClkDivided = Output(Bool())
}

class DebugBumpsIO extends Bundle {
  val phy = new PhyDebugBumpsIO
  val txData = Output(Bool())
}

class CommonPhyBumpsIO extends Bundle {
  val refClkP = Input(Bool())
  val refClkN = Input(Bool())
  val bypassClkP = Input(Bool())
  val bypassClkN = Input(Bool())
  val pllRdacVref = Input(Bool())
}

class CommonBumpsIO extends Bundle {
  val phy = new CommonPhyBumpsIO
  val debug = new DebugBumpsIO
}

class UciephyCommonIO(bitCounterWidth: Int = 64) extends Bundle {
  val debug = Flipped(new PhyDebugBumpsIO)
  val phy = Flipped(new CommonPhyBumpsIO)
  val top = new CommonBumpsIO
  val driverctl = Input(Vec(6, new DriverControlIO))
  val txctl = Input(new TxLaneDigitalCtlIO)
  val txTestMode = Input(TxTestMode())
  val txDataMode = Input(DataMode())
  val txLfsrSeed = Input(UInt(64.W))
  val txFsmRst = Input(Bool())
  val txExecute = Input(Bool())
  val txManualRepeatPeriod = Input(UInt(6.W))
  val txPacketsToSend = Input(UInt(bitCounterWidth.W))
  val data = Input(Vec(16, UInt(64.W)))
  val txState = Output(TxTestState())
  val txPacketsEnqueued = Output(UInt(bitCounterWidth.W))
  val dllCode = Output(UInt(5.W))
}

class UciephyCommon(bitCounterWidth: Int = 64, sim: Boolean = false) extends Module {
  val io = IO(new UciephyCommonIO(bitCounterWidth))

  io.phy.pllRdacVref := io.top.phy.pllRdacVref

  val txReset = io.txFsmRst || reset.asBool
  val txState = withReset(txReset) { RegInit(TxTestState.idle) }
  io.txState := txState
  val txPacketsEnqueued = withReset(txReset) {
    RegInit(0.U(bitCounterWidth.W))
  }
  io.txPacketsEnqueued := txPacketsEnqueued
  val inputBufferAddrReg = withReset(txReset) { RegInit(0.U(5.W)) }
  val maxPackets = 32.U
  val txManualRepeatPeriodFinal = Mux(
    io.txManualRepeatPeriod === 0.U || io.txManualRepeatPeriod > maxPackets,
    maxPackets,
    io.txManualRepeatPeriod
  )
  val txLfsr = Module(
    new FibonacciLFSR(
      2 * Phy.SerdesRatio,
      taps = LFSR.tapsMaxPeriod.get(2 * Phy.SerdesRatio).get.head,
      step = Phy.SerdesRatio
    )
  )
  txLfsr.io.seed.bits := io.txLfsrSeed.asTypeOf(txLfsr.io.seed.bits)
  txLfsr.io.seed.valid := txReset
  txLfsr.io.increment := false.B

  val txFifo = Module(new AsyncQueue(UInt(32.W), Phy.QueueParams))
  val shuffler = Module(new Shuffler32)
  val txLane = Module(new TxLane(sim))
  val rstSync = Module(new UcieRstSync(sim))
  rstSync.io.rstbAsync := !reset.asBool
  rstSync.io.clk := txLane.io.divclk
  txFifo.io.enq.bits := 0.U
  txFifo.io.enq.valid := false.B
  txFifo.io.enq_clock := clock
  txFifo.io.enq_reset := reset
  txFifo.io.deq_clock := txLane.io.divclk.asClock
  txFifo.io.deq_reset := !rstSync.io.rstbSync.asBool
  txFifo.io.deq.ready := true.B

  when(txFifo.io.deq.valid) {
    shuffler.io.din := txFifo.io.deq.bits
  }.otherwise {
    shuffler.io.din := 0.U
  }
  shuffler.io.permutation := io.txctl.shuffler

  txLane.io.dll_reset := io.txctl.dll_reset
  txLane.io.dll_resetb := !io.txctl.dll_reset
  txLane.io.ser_resetb := !reset.asBool
  txLane.io.din := shuffler.io.dout.asTypeOf(txLane.io.din)
  io.top.debug.txData := txLane.io.dout
  txLane.io.ctl.driver := io.txctl.driver
  txLane.io.ctl.skew := io.txctl.skew
  io.dllCode := txLane.io.dll_code

  // TX logic
  switch(txState) {
    is(TxTestState.idle) {
      when(io.txExecute) {
        txState := TxTestState.run
      }
    }
    is(TxTestState.run) {
      switch(io.txTestMode) {
        is(TxTestMode.manual) {
          // Only send the next packet if we still need to send more bits.
          switch(io.txDataMode) {
            is(DataMode.finite) {
              txFifo.io.enq.valid := txPacketsEnqueued < io.txPacketsToSend
            }
            is(DataMode.infinite) {
              txFifo.io.enq.valid := true.B
            }
          }
        }
        is(TxTestMode.lfsr) {
          switch(io.txDataMode) {
            is(DataMode.finite) {
              txFifo.io.enq.valid := txPacketsEnqueued < io.txPacketsToSend
            }
            is(DataMode.infinite) {
              txFifo.io.enq.valid := true.B
            }
          }
        }
      }
      when(txFifo.io.enq.valid) {
        switch(io.txTestMode) {
          is(TxTestMode.manual) {
            txFifo.io.enq.bits := io.data.asTypeOf(Vec(32, UInt(32.W)))(
              inputBufferAddrReg
            )
          }
          is(TxTestMode.lfsr) {
            txFifo.io.enq.bits := Reverse(txLfsr.io.out.asUInt)(31, 0)
          }
        }
      }

      when(txFifo.io.enq.valid && txFifo.io.enq.ready) {
        txPacketsEnqueued := Mux(
          txPacketsEnqueued < VecInit(
            Seq.fill(txPacketsEnqueued.getWidth)(true.B)
          ).asUInt,
          txPacketsEnqueued + 1.U,
          txPacketsEnqueued
        )
        inputBufferAddrReg := (inputBufferAddrReg + 1.U) % txManualRepeatPeriodFinal
        when(io.txTestMode === TxTestMode.lfsr) {
          txLfsr.io.increment := true.B
        }
      }

      when(!txFifo.io.enq.valid) {
        txState := TxTestState.done
      }
    }
    is(TxTestState.done) {}
  }

  val refclkrx = Module(new ClkRx(sim))
  refclkrx.io.vip := io.top.phy.refClkP
  refclkrx.io.vin := io.top.phy.refClkN
  val refclkbuf = Module(new DiffBuffer(sim))
  refclkbuf.io.vinp := refclkrx.io.vop
  refclkbuf.io.vinn := refclkrx.io.von
  io.phy.refClkP := refclkbuf.io.voutp
  io.phy.refClkN := refclkbuf.io.voutn

  val bpclkrx = Module(new ClkRx(sim))
  bpclkrx.io.vip := io.top.phy.bypassClkP
  bpclkrx.io.vin := io.top.phy.bypassClkN
  val bpclkbuf = Module(new DiffBuffer(sim))
  bpclkbuf.io.vinp := bpclkrx.io.vop
  bpclkbuf.io.vinn := bpclkrx.io.von
  io.phy.bypassClkP := bpclkbuf.io.voutp
  io.phy.bypassClkN := bpclkbuf.io.voutn

  val testPllClkPBuf0 = Module(new SingleEndedBuffer(sim))
  val testPllClkPBuf1 = Module(new SingleEndedBuffer(sim))
  val testPllClkPBuf2 = Module(new SingleEndedBuffer(sim))
  val testPllClkNBuf0 = Module(new SingleEndedBuffer(sim))
  val testPllClkNBuf1 = Module(new SingleEndedBuffer(sim))
  val testPllClkNBuf2 = Module(new SingleEndedBuffer(sim))
  testPllClkPBuf0.io.vin := io.debug.testPllClkP
  testPllClkPBuf1.io.vin := testPllClkPBuf0.io.vout
  testPllClkPBuf2.io.vin := testPllClkPBuf1.io.vout
  testPllClkNBuf0.io.vin := io.debug.testPllClkN
  testPllClkNBuf1.io.vin := testPllClkNBuf0.io.vout
  testPllClkNBuf2.io.vin := testPllClkNBuf1.io.vout

  val uciePllClkBuf0 = Module(new DiffBuffer(sim))
  val uciePllClkBuf1 = Module(new DiffBuffer(sim))
  uciePllClkBuf0.io.vinp := io.debug.pllClkP
  uciePllClkBuf0.io.vinn := io.debug.pllClkN
  uciePllClkBuf1.io.vinp := uciePllClkBuf0.io.voutp
  uciePllClkBuf1.io.vinn := uciePllClkBuf0.io.voutn
  txLane.io.clkp := uciePllClkBuf1.io.voutp
  txLane.io.clkn := uciePllClkBuf1.io.voutn

  val pllClkNDiv = Module(new ClkDiv4(sim))
  pllClkNDiv.io.clk := uciePllClkBuf0.io.voutn
  pllClkNDiv.io.resetb := !reset.asBool

  val rxClkPBuf0 = Module(new SingleEndedBuffer(sim))
  val rxClkPBuf1 = Module(new SingleEndedBuffer(sim))
  val rxClkPBuf2 = Module(new SingleEndedBuffer(sim))
  val rxClkPBuf3 = Module(new SingleEndedBuffer(sim))
  val rxClkNBuf0 = Module(new SingleEndedBuffer(sim))
  val rxClkNBuf1 = Module(new SingleEndedBuffer(sim))
  val rxClkNBuf2 = Module(new SingleEndedBuffer(sim))
  val rxClkNBuf3 = Module(new SingleEndedBuffer(sim))
  rxClkPBuf0.io.vin := io.debug.rxClk
  rxClkPBuf1.io.vin := rxClkPBuf0.io.vout
  rxClkPBuf2.io.vin := rxClkPBuf1.io.vout
  rxClkPBuf3.io.vin := rxClkPBuf2.io.vout
  rxClkNBuf0.io.vin := io.debug.rxClkDivided
  rxClkNBuf1.io.vin := rxClkNBuf0.io.vout
  rxClkNBuf2.io.vin := rxClkNBuf1.io.vout
  rxClkNBuf3.io.vin := rxClkNBuf2.io.vout

  val drivers = Seq(
    (testPllClkPBuf2.io.vout, io.top.debug.phy.testPllClkP, "testPllClkPDriver"),
    (testPllClkNBuf2.io.vout, io.top.debug.phy.testPllClkN, "testPllClkNDriver"),
    (uciePllClkBuf0.io.voutp, io.top.debug.phy.pllClkP, "pllClkPDriver"),
    (pllClkNDiv.io.clkout_2, io.top.debug.phy.pllClkN, "pllClkNDriver"),
    (rxClkPBuf3.io.vout, io.top.debug.phy.rxClk, "rxClkDriver"),
    (rxClkNBuf3.io.vout, io.top.debug.phy.rxClkDivided, "rxClkDivDriver")
  ).zipWithIndex
  for (((input, output, name), i) <- drivers) {
    val driver = Module(new TxDriver(sim)).suggestName(name)
    driver.io.din := input
    output := driver.io.dout
    driver.io.ctl := io.driverctl(i)
  }
}
