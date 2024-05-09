package uciephytest.phy

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

class DecoderSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "Decoder"
  it should "work" in {
    test(new Decoder(outBits = 256)) { c =>
      c.io.binary.poke(0.U)
      c.clock.step()
      c.io.thermometer.expect(0.U)
      c.io.binary.poke(5.U)
      c.clock.step()
      c.io.thermometer.expect("b011111".U)
      c.io.binary.poke(16.U)
      c.clock.step()
      c.io.thermometer.expect("h0ffff".U)
      c.io.binary.poke(126.U)
      c.clock.step()
      c.io.thermometer.expect("h3fffffffffffffffffffffffffffffff".U)
      c.io.binary.poke(256.U)
      c.clock.step()
      c.io.thermometer.expect("hffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".U)
    }
  }
}
