package uciephytest

import chisel3._
import chisel3.util._
import chiseltest._
import org.chipsalliance.cde.config.{Field, Parameters, Config}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.prci._
import freechips.rocketchip.diplomacy._
import chisel3.experimental.BundleLiterals._

// import freechips.rocketchip.unittest._
import org.scalatest.flatspec.AnyFlatSpec
import edu.berkeley.cs.ucie.digital.tilelink._
import edu.berkeley.cs.ucie.digital.interfaces.{FdiParams, RdiParams, AfeParams}
import edu.berkeley.cs.ucie.digital.protocol.{ProtocolLayerParams}
import edu.berkeley.cs.ucie.digital.sideband.{SidebandParams}
import edu.berkeley.cs.ucie.digital.logphy.{LinkTrainingParams, TransmitPattern, RegisterRWIO, RegisterRW}

import uciephytest.phy._

import java.util.ResourceBundle

class RxAfeCtlTest extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "RxAfeCtl"
  it should "behave correctly in FSM mode and bypass mode" in {
    test(new RxAfeCtl()).withAnnotations(
      Seq(VcsBackendAnnotation, WriteFsdbAnnotation),
    ) { c =>
      c.reset.poke(true.B)
      c.io.bypass.poke(false.B)
      c.io.opCycles.poke(32.U)
      c.io.overlapCycles.poke(16.U)
      c.io.afe.aEn.expect(false.B)
      c.io.afe.aPc.expect(true.B)
      c.io.afe.bEn.expect(false.B)
      c.io.afe.bPc.expect(true.B)
      c.clock.step(4)
      c.reset.poke(false.B)

      for (iter <- 0 to 8) {
        for (i <- 0 to 32) {
          c.io.afe.aEn.expect(true.B)
          c.io.afe.aPc.expect(false.B)
          c.io.afe.bEn.expect(false.B)
          c.io.afe.bPc.expect(true.B)
          c.io.afe.selA.expect(true.B)
          c.clock.step(timeout)
        }
        for (i <- 0 to 16) {
          c.io.afe.aEn.expect(true.B)
          c.io.afe.aPc.expect(false.B)
          c.io.afe.bEn.expect(true.B)
          c.io.afe.bPc.expect(false.B)
          c.io.afe.selA.expect(true.B)
          c.clock.step(timeout)
        }
        c.io.afe.aEn.expect(true.B)
        c.io.afe.aPc.expect(false.B)
        c.io.afe.bEn.expect(true.B)
        c.io.afe.bPc.expect(false.B)
        c.io.afe.selA.expect(false.B)
        c.clock.step(timeout)
        for (i <- 0 to 32) {
          c.io.afe.aEn.expect(false.B)
          c.io.afe.aPc.expect(true.B)
          c.io.afe.bEn.expect(true.B)
          c.io.afe.bPc.expect(false.B)
          c.io.afe.selA.expect(false.B)
          c.clock.step(timeout)
        }
        for (i <- 0 to 16) {
          c.io.afe.aEn.expect(true.B)
          c.io.afe.aPc.expect(false.B)
          c.io.afe.bEn.expect(true.B)
          c.io.afe.bPc.expect(false.B)
          c.io.afe.selA.expect(false.B)
          c.clock.step(timeout)
        }
        c.io.afe.aEn.expect(true.B)
        c.io.afe.aPc.expect(false.B)
        c.io.afe.bEn.expect(true.B)
        c.io.afe.bPc.expect(false.B)
        c.io.afe.selA.expect(true.B)
        c.clock.step(timeout)
      }

      c.io.bypass.poke(true.B)
      c.io.afeBypass.aEn.poke(true.B)
      c.io.afe.aEn.expect(true.B)
      c.io.afeBypass.aEn.poke(false.B)
      c.io.afe.aEn.expect(false.B)
      c.io.afeBypass.aPc.poke(true.B)
      c.io.afe.aPc.expect(true.B)
      c.io.afeBypass.aPc.poke(false.B)
      c.io.afe.aPc.expect(false.B)
      c.io.afeBypass.bEn.poke(true.B)
      c.io.afe.bEn.expect(true.B)
      c.io.afeBypass.bEn.poke(false.B)
      c.io.afe.bEn.expect(false.B)
      c.io.afeBypass.bPc.poke(true.B)
      c.io.afe.bPc.expect(true.B)
      c.io.afeBypass.bPc.poke(false.B)
      c.io.afe.bPc.expect(false.B)
      c.io.afeBypass.selA.poke(true.B)
      c.io.afe.selA.expect(true.B)
      c.io.afeBypass.selA.poke(false.B)
      c.io.afe.selA.expect(false.B)
      c.io.bypass.poke(false.B)

      for (iter <- 0 to 8) {
        for (i <- 0 to 32) {
          c.io.afe.aEn.expect(true.B)
          c.io.afe.aPc.expect(false.B)
          c.io.afe.bEn.expect(false.B)
          c.io.afe.bPc.expect(true.B)
          c.io.afe.selA.expect(true.B)
          c.clock.step(timeout)
        }
        for (i <- 0 to 16) {
          c.io.afe.aEn.expect(true.B)
          c.io.afe.aPc.expect(false.B)
          c.io.afe.bEn.expect(true.B)
          c.io.afe.bPc.expect(false.B)
          c.io.afe.selA.expect(true.B)
          c.clock.step(timeout)
        }
        for (i <- 0 to 32) {
          c.io.afe.aEn.expect(false.B)
          c.io.afe.aPc.expect(true.B)
          c.io.afe.bEn.expect(true.B)
          c.io.afe.bPc.expect(false.B)
          c.io.afe.selA.expect(false.B)
          c.clock.step(timeout)
        }
        for (i <- 0 to 16) {
          c.io.afe.aEn.expect(true.B)
          c.io.afe.aPc.expect(false.B)
          c.io.afe.bEn.expect(true.B)
          c.io.afe.bPc.expect(false.B)
          c.io.afe.selA.expect(false.B)
          c.clock.step(timeout)
        }
      }

      c.reset.poke(true.B)
      c.io.afe.aEn.expect(false.B)
      c.io.afe.aPc.expect(true.B)
      c.io.afe.bEn.expect(false.B)
      c.io.afe.bPc.expect(true.B)
    }
  }
}
