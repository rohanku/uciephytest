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
  val refClkInP = Input(Bool())
  val refClkInN = Input(Bool())
  val refClkOutP = Output(Bool())
  val refClkOutN = Output(Bool())
  val bypassClkInP = Input(Bool())
  val bypassClkInN = Input(Bool())
  val bypassClkOutP = Output(Bool())
  val bypassClkOutN = Output(Bool())
}

class UciephyCommon(sim: Boolean = false) extends RawModule {
  val io = IO(new UciephyCommonIO)

  val refclkrx = Module(new ClkRx(sim))
  refclkrx.io.vip := io.refClkInP
  refclkrx.io.vin := io.refClkInN
  io.refClkOutP := refclkrx.io.vop
  io.refClkOutN := refclkrx.io.von

  val bpclkrx = Module(new ClkRx(sim))
  bpclkrx.io.vip := io.bypassClkInP
  bpclkrx.io.vin := io.bypassClkInN
  io.bypassClkOutP := bpclkrx.io.vop
  io.bypassClkOutN := bpclkrx.io.von
}
