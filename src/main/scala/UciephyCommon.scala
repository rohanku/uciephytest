package uciephytest

import chisel3._
import chisel3.util._
import chisel3.util.random._
import chisel3.experimental.BundleLiterals._
import chisel3.experimental.VecLiterals._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS, SBUS, CacheBlockBytes, TLBusWrapperLocation}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField, RegWriteFn, RegReadFn, RegFieldDesc}
import freechips.rocketchip.tilelink._
import uciephytest.phy.{Phy, ClkRx, PhyToTestIO, TxLaneDigitalCtlIO, RxLaneDigitalCtlIO, RxLaneCtlIO, DriverControlIO, TxSkewControlIO, RxAfeIO}
import freechips.rocketchip.util.{AsyncQueueParams}
import testchipip.soc.{OBUS}
import edu.berkeley.cs.ucie.digital.tilelink._
import edu.berkeley.cs.ucie.digital.interfaces.{FdiParams, RdiParams, AfeParams}
import edu.berkeley.cs.ucie.digital.protocol.{ProtocolLayerParams}
import edu.berkeley.cs.ucie.digital.sideband.{SidebandParams}
import edu.berkeley.cs.ucie.digital.logphy.{LinkTrainingParams, TransmitPattern, RegisterRWIO, RegisterRW}

class UciephyCommonIO extends Bundle {
}

class UciephyCommon(sim: Boolean = false) extends RawModule {
  val io = IO(new UciephyCommonIO)
}

case class UciephyCommonParams(
  address: BigInt = 0x4000,
  managerWhere: TLBusWrapperLocation = PBUS,
  sim: Boolean = false
)

case object UciephyCommonKey extends Field[Option[Seq[UciephyCommonParams]]](None)

class UciephyCommonTLIO extends Bundle {
  val refClkInP = Input(Bool())
  val refClkInN = Input(Bool())
  val refClkOutP = Output(Bool())
  val refClkOutN = Output(Bool())
  val bypassClkInP = Input(Bool())
  val bypassClkInN = Input(Bool())
  val bypassClkOutP = Output(Bool())
  val bypassClkOutN = Output(Bool())
  val testPllClkInP = Input(Bool())
  val testPllClkInN = Input(Bool())
  val testPllClkOutP = Output(Bool())
  val testPllClkOutN = Output(Bool())
}

class UciephyCommonTL(params: UciephyCommonParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  def toRegFieldRw[T <: Data](r: T, name: String): RegField = {
        RegField(r.getWidth, r.asUInt, RegWriteFn((valid, data) => {
          when (valid) {
            r := data.asTypeOf(r)
          }
          true.B
        }), Some(RegFieldDesc(name, "")))
      }
  def toRegFieldR[T <: Data](r: T, name: String): RegField = {
        RegField.r(r.getWidth, r.asUInt, RegFieldDesc(name, ""))
      }
  override lazy val desiredName = "UciephyCommonTL"
  val device = new SimpleDevice("uciephycommon", Seq("ucbbar,uciephycommon"))
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  val topIO = BundleBridgeSource(() => new UciephyCommonTLIO)

  override lazy val module = new UciephyCommonImpl
  class UciephyCommonImpl extends Impl {
    val io = IO(new Bundle {})
    withClockAndReset(clock, reset) {
      // MMIO registers.
      val driverctl = RegInit(VecInit(Seq.fill(2)({
        val w = Wire(new DriverCtlIO)
        w.driver.pu_ctl := 0.U
        w.driver.pd_ctl := 0.U
        w.driver.en := false.B
        w.driver.en_b := true.B
        w
      })))

      val refclkrx = Module(new ClkRx(params.sim))
      refclkrx.io.vip := io.refClkInP
      refclkrx.io.vin := io.refClkInN
      io.refClkOutP := refclkrx.io.vop
      io.refClkOutN := refclkrx.io.von

      val bpclkrx = Module(new ClkRx(params.sim))
      bpclkrx.io.vip := io.bypassClkInP
      bpclkrx.io.vin := io.bypassClkInN
      io.bypassClkOutP := bpclkrx.io.vop
      io.bypassClkOutN := bpclkrx.io.von

      val testPllClkP = Module(new TxDriver(params.sim))
      testPllClkP.io.din := io.testPllClkInP
      io.testPllClkOutP := testPllClkP.io.dout
      testPllClkP.io.ctl := driverctl(0)

      val testPllClkN = Module(new TxDriver(params.sim))
      testPllClkN.io.din := io.testPllClkInN
      io.testPllClkOutN := testPllClkN.io.dout
      testPllClkN.io.ctl := driverctl(1)

      var mmioRegs = driverctl.zipWithIndex.map((i, ctl) => {
          toRegFieldRw(ctl, s"driverctl_${i}")
      })

      node.regmap(mmioRegs.zipWithIndex.map({ case (f, i) => {
      i * 8 -> Seq(f)} }): _*)
    }
  }
}


trait CanHavePeripheryUciephyCommon { this: BaseSubsystem =>
  private val portName = "uciephycommon"

  private val pbus = locateTLBusWrapper(PBUS)
  private val sbus = locateTLBusWrapper(SBUS)

  val uciephy_common = p(UciephyCommonKey) match {
    case Some(params) => {
      val uciephy_common = params.map(x => LazyModule(new UciephyCommonTL(x, sbus.beatBytes)(p)))

      lazy val uciephy_tlbus = params.map(x => locateTLBusWrapper(x.managerWhere))

      for ((((ucie, ucie_params), tlbus), n) <- uciephy_common.zip(params).zip(uciephy_tlbus).zipWithIndex){
        ucie.clockNode := sbus.fixedClockNode
        sbus.coupleTo(s"uciephycommon{$n}") { ucie.node := TLBuffer() := TLFragmenter(sbus.beatBytes, sbus.blockBytes) := TLBuffer() := _ }
      }
      Some(uciephy_common)
    }
    case None => None
  }
}

class WithUciephyCommon(params: Seq[UciephyCommonParams]) extends Config((site, here, up) => {
  case UciephyCommonKey => Some(params)
})

class WithUciephyCommonSim extends Config((site, here, up) => {
  case UciephyCommonKey => up(UciephyCommonKey, site).map(u => u.map(_.copy(sim = true)))
})
