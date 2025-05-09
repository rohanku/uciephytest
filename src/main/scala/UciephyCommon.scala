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
  RstSync,
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

class UciephyCommonTLIO extends Bundle {
  val refClkInP = Input(Bool())
  val refClkInN = Input(Bool())
  val refClkOutP = Output(Bool())
  val refClkOutN = Output(Bool())
  val bypassClkInP = Input(Bool())
  val bypassClkInN = Input(Bool())
  val bypassClkOutP = Output(Bool())
  val bypassClkOutN = Output(Bool())
  val pllClkInP = Input(Bool())
  val pllClkInN = Input(Bool())
  val pllClkOutP = Output(Bool())
  val pllClkOutN = Output(Bool())
  val testPllClkInP = Input(Bool())
  val testPllClkInN = Input(Bool())
  val testPllClkOutP = Output(Bool())
  val testPllClkOutN = Output(Bool())
  val rxClkIn = Input(Bool())
  val rxClkInDivided = Input(Bool())
  val rxClkOut = Output(Bool())
  val rxClkOutDivided = Output(Bool())
  val txDataDebug = Output(Bool())
}

class UciephyCommonTL(params: UciephyCommonParams, beatBytes: Int)(implicit
    p: Parameters
) extends ClockSinkDomain(ClockSinkParameters())(p) {
  def toRegFieldRw[T <: Data](r: T, name: String): RegField = {
    RegField(
      r.getWidth,
      r.asUInt,
      RegWriteFn((valid, data) => {
        when(valid) {
          r := data.asTypeOf(r)
        }
        true.B
      }),
      Some(RegFieldDesc(name, ""))
    )
  }
  def toRegFieldR[T <: Data](r: T, name: String): RegField = {
    RegField.r(r.getWidth, r.asUInt, RegFieldDesc(name, ""))
  }
  override lazy val desiredName = "UciephyCommonTL"
  val device = new SimpleDevice("uciephycommon", Seq("ucbbar,uciephycommon"))
  val node = TLRegisterNode(
    Seq(AddressSet(params.address, 4096 - 1)),
    device,
    "reg/control",
    beatBytes = beatBytes
  )

  val topIO = BundleBridgeSource(() => new UciephyCommonTLIO)

  override lazy val module = new UciephyCommonImpl
  class UciephyCommonImpl extends Impl {
    val io = IO(new Bundle {})
    withClockAndReset(clock, reset) {
      val io = topIO.out(0)._1
      // MMIO registers.
      // Test PLL P/N, UCIe PLL P/N, RX CLK P/N
      val driverctl = RegInit(VecInit(Seq.fill(6)({
        val w = Wire(new DriverControlIO)
        w.pu_ctl := 0.U
        w.pd_ctl := 0.U
        w.en := false.B
        w.en_b := true.B
        w
      })))
      val txctl = RegInit({
        val w = Wire(new TxLaneDigitalCtlIO)
        w.dll_reset := true.B
        w.driver.pu_ctl := 0.U
        w.driver.pd_ctl := 0.U
        w.driver.en := false.B
        w.driver.en_b := true.B
        w.skew.dll_en := false.B
        w.skew.ocl := false.B
        w.skew.delay := 0.U
        w.skew.mux_en := "b00000011".U
        w.skew.band_ctrl := "b01".U
        w.skew.mix_en := 0.U
        w.skew.nen_out := 20.U
        w.skew.pen_out := 22.U
        for (i <- 0 until 32) {
          w.shuffler(i) := i.U(5.W)
        }
        w.sample_negedge := false.B
        w.delay := 0.U
        w
      })

      val txTestMode = RegInit(TxTestMode.manual)
      val txDataMode = RegInit(DataMode.finite)
      val txLfsrSeed = RegInit(1.U(64.W))
      val txFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val txExecute = Wire(DecoupledIO(UInt(1.W)))
      txFsmRst.ready := true.B
      txExecute.ready := true.B
      val txManualRepeatPeriod = RegInit(0.U(6.W))
      val txPacketsToSend = RegInit(0.U(params.bitCounterWidth.W))

      val txReset = txFsmRst.valid || reset.asBool
      val txState = withReset(txReset) { RegInit(TxTestState.idle) }
      val txPacketsEnqueued = withReset(txReset) {
        RegInit(0.U(params.bitCounterWidth.W))
      }
      val inputBufferAddrReg = withReset(txReset) { RegInit(0.U(5.W)) }
      val maxPackets = 32.U
      val txManualRepeatPeriodFinal = Mux(
        txManualRepeatPeriod === 0.U || txManualRepeatPeriod > maxPackets,
        maxPackets,
        txManualRepeatPeriod
      )
      val txLfsr = Module(
        new FibonacciLFSR(
          2 * Phy.SerdesRatio,
          taps = LFSR.tapsMaxPeriod.get(2 * Phy.SerdesRatio).get.head,
          step = Phy.SerdesRatio
        )
      )
      txLfsr.io.seed.bits := txLfsrSeed.asTypeOf(txLfsr.io.seed.bits)
      txLfsr.io.seed.valid := txReset
      txLfsr.io.increment := false.B
      val data = RegInit(VecInit(Seq.fill(16)(0.U(64.W))))

      val txFifo = Module(new AsyncQueue(UInt(32.W), Phy.QueueParams))
      val shuffler = Module(new Shuffler32)
      val txLane = Module(new TxLane(params.sim))
      val rstSync = Module(new RstSync(params.sim))
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
      shuffler.io.permutation := txctl.shuffler

      txLane.io.dll_reset := txctl.dll_reset
      txLane.io.dll_resetb := !txctl.dll_reset
      txLane.io.ser_resetb := !reset.asBool
      txLane.io.din := shuffler.io.dout.asTypeOf(txLane.io.din)
      io.txDataDebug := txLane.io.dout
      txLane.io.ctl.driver := txctl.driver
      txLane.io.ctl.skew := txctl.skew

      // TX logic
      switch(txState) {
        is(TxTestState.idle) {
          when(txExecute.valid) {
            txState := TxTestState.run
          }
        }
        is(TxTestState.run) {
          switch(txTestMode) {
            is(TxTestMode.manual) {
              // Only send the next packet if we still need to send more bits.
              switch(txDataMode) {
                is(DataMode.finite) {
                  txFifo.io.enq.valid := txPacketsEnqueued < txPacketsToSend
                }
                is(DataMode.infinite) {
                  txFifo.io.enq.valid := true.B
                }
              }
            }
            is(TxTestMode.lfsr) {
              switch(txDataMode) {
                is(DataMode.finite) {
                  txFifo.io.enq.valid := txPacketsEnqueued < txPacketsToSend
                }
                is(DataMode.infinite) {
                  txFifo.io.enq.valid := true.B
                }
              }
            }
          }
          when(txFifo.io.enq.valid) {
            switch(txTestMode) {
              is(TxTestMode.manual) {
                txFifo.io.enq.bits := data.asTypeOf(Vec(32, UInt(32.W)))(
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
            when(txTestMode === TxTestMode.lfsr) {
              txLfsr.io.increment := true.B
            }
          }

          when(!txFifo.io.enq.valid) {
            txState := TxTestState.done
          }
        }
        is(TxTestState.done) {}
      }

      val refclkrx = Module(new ClkRx(params.sim))
      refclkrx.io.vip := io.refClkInP
      refclkrx.io.vin := io.refClkInN
      val refclkbuf = Module(new DiffBuffer(params.sim))
      refclkbuf.io.vinp := refclkrx.io.vop
      refclkbuf.io.vinn := refclkrx.io.von
      io.refClkOutP := refclkbuf.io.voutp
      io.refClkOutN := refclkbuf.io.voutn

      val bpclkrx = Module(new ClkRx(params.sim))
      bpclkrx.io.vip := io.bypassClkInP
      bpclkrx.io.vin := io.bypassClkInN
      val bpclkbuf = Module(new DiffBuffer(params.sim))
      bpclkbuf.io.vinp := bpclkrx.io.vop
      bpclkbuf.io.vinn := bpclkrx.io.von
      io.bypassClkOutP := bpclkbuf.io.voutp
      io.bypassClkOutN := bpclkbuf.io.voutn

      val testPllClkPBuf0 = Module(new SingleEndedBuffer(params.sim))
      val testPllClkPBuf1 = Module(new SingleEndedBuffer(params.sim))
      val testPllClkPBuf2 = Module(new SingleEndedBuffer(params.sim))
      val testPllClkNBuf0 = Module(new SingleEndedBuffer(params.sim))
      val testPllClkNBuf1 = Module(new SingleEndedBuffer(params.sim))
      val testPllClkNBuf2 = Module(new SingleEndedBuffer(params.sim))
      testPllClkPBuf0.io.vin := io.testPllClkInP
      testPllClkPBuf1.io.vin := testPllClkPBuf0.io.vout
      testPllClkPBuf2.io.vin := testPllClkPBuf1.io.vout
      testPllClkNBuf0.io.vin := io.testPllClkInN
      testPllClkNBuf1.io.vin := testPllClkNBuf0.io.vout
      testPllClkNBuf2.io.vin := testPllClkNBuf1.io.vout

      val uciePllClkBuf0 = Module(new DiffBuffer(params.sim))
      val uciePllClkBuf1 = Module(new DiffBuffer(params.sim))
      uciePllClkBuf0.io.vinp := io.pllClkInP
      uciePllClkBuf0.io.vinn := io.pllClkInN
      uciePllClkBuf1.io.vinp := uciePllClkBuf0.io.voutp
      uciePllClkBuf1.io.vinn := uciePllClkBuf0.io.voutn
      txLane.io.clkp := uciePllClkBuf1.io.voutp
      txLane.io.clkn := uciePllClkBuf1.io.voutn

      val pllClkNDiv = Module(new ClkDiv4(params.sim))
      pllClkNDiv.io.clk := uciePllClkBuf0.io.voutn
      pllClkNDiv.io.resetb := !reset.asBool

      val rxClkPBuf0 = Module(new SingleEndedBuffer(params.sim))
      val rxClkPBuf1 = Module(new SingleEndedBuffer(params.sim))
      val rxClkPBuf2 = Module(new SingleEndedBuffer(params.sim))
      val rxClkPBuf3 = Module(new SingleEndedBuffer(params.sim))
      val rxClkNBuf0 = Module(new SingleEndedBuffer(params.sim))
      val rxClkNBuf1 = Module(new SingleEndedBuffer(params.sim))
      val rxClkNBuf2 = Module(new SingleEndedBuffer(params.sim))
      val rxClkNBuf3 = Module(new SingleEndedBuffer(params.sim))
      rxClkPBuf0.io.vin := io.rxClkIn
      rxClkPBuf1.io.vin := rxClkPBuf0.io.vout
      rxClkPBuf2.io.vin := rxClkPBuf1.io.vout
      rxClkPBuf3.io.vin := rxClkPBuf2.io.vout
      rxClkNBuf0.io.vin := io.rxClkInDivided
      rxClkNBuf1.io.vin := rxClkNBuf0.io.vout
      rxClkNBuf2.io.vin := rxClkNBuf1.io.vout
      rxClkNBuf3.io.vin := rxClkNBuf2.io.vout

      val drivers = Seq(
        (testPllClkPBuf2.io.vout, io.testPllClkOutP, "testPllClkPDriver"),
        (testPllClkNBuf2.io.vout, io.testPllClkOutN, "testPllClkNDriver"),
        (uciePllClkBuf0.io.voutp, io.pllClkOutP, "pllClkPDriver"),
        (pllClkNDiv.io.clkout_2, io.pllClkOutN, "pllClkNDriver"),
        (rxClkPBuf3.io.vout, io.rxClkOut, "rxClkDriver"),
        (rxClkNBuf3.io.vout, io.rxClkOutDivided, "rxClkDivDriver")
      ).zipWithIndex
      for (((input, output, name), i) <- drivers) {
        val driver = Module(new TxDriver(params.sim)).suggestName(name)
        driver.io.din := input
        output := driver.io.dout
        driver.io.ctl := driverctl(i)
      }

      var mmioRegs = Seq(
        toRegFieldRw(txTestMode, "txTestMode"),
        toRegFieldRw(txDataMode, "txDataMode"),
        toRegFieldRw(txLfsrSeed, s"txLfsrSeed"),
        RegField.w(1, txFsmRst, RegFieldDesc("txFsmRst", "")),
        RegField.w(1, txExecute, RegFieldDesc("txExecute", "")),
        toRegFieldR(txPacketsEnqueued, "txPacketsSent"),
        toRegFieldRw(txManualRepeatPeriod, "txManualRepeatPeriod"),
        toRegFieldRw(txPacketsToSend, "txPacketsToSend"),
        toRegFieldR(txState, "txTestState")
      ) ++ (0 until driverctl.length).map((i: Int) => {
        toRegFieldRw(driverctl(i), s"driverctl_${i}")
      }) ++ Seq(
        toRegFieldRw(txctl.dll_reset, s"dll_reset"),
        toRegFieldRw(txctl.driver, s"txctl_driver"),
        toRegFieldRw(txctl.skew, s"txctl_skew")
      ) ++ (0 until 32).map((j: Int) =>
        toRegFieldRw(txctl.shuffler(j), s"txctl_shuffler_$j")
      ) ++ Seq(
        toRegFieldR(txLane.io.dll_code, s"dllCode")
      )

      node.regmap(mmioRegs.zipWithIndex.map({
        case (f, i) => {
          i * 8 -> Seq(f)
        }
      }): _*)
    }
  }
}

trait CanHavePeripheryUciephyCommon { this: BaseSubsystem =>
  private val portName = "uciephycommon"

  private val pbus = locateTLBusWrapper(PBUS)
  private val sbus = locateTLBusWrapper(SBUS)

  val uciephy_common = p(UciephyCommonKey) match {
    case Some(params) => {
      val uciephy_common =
        params.map(x => LazyModule(new UciephyCommonTL(x, sbus.beatBytes)(p)))

      lazy val uciephy_tlbus =
        params.map(x => locateTLBusWrapper(x.managerWhere))

      for (
        (((ucie, ucie_params), tlbus), n) <- uciephy_common
          .zip(params)
          .zip(uciephy_tlbus)
          .zipWithIndex
      ) {
        ucie.clockNode := sbus.fixedClockNode
        sbus.coupleTo(s"uciephycommon{$n}") {
          ucie.node := TLBuffer() := TLFragmenter(
            sbus.beatBytes,
            sbus.blockBytes
          ) := TLBuffer() := _
        }
      }
      Some(uciephy_common)
    }
    case None => None
  }
}

class WithUciephyCommon(params: Seq[UciephyCommonParams])
    extends Config((site, here, up) => { case UciephyCommonKey =>
      Some(params)
    })

class WithUciephyCommonSim
    extends Config((site, here, up) => { case UciephyCommonKey =>
      up(UciephyCommonKey, site).map(u => u.map(_.copy(sim = true)))
    })
