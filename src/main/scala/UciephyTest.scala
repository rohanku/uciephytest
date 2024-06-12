package uciephytest

import chisel3._
import chisel3.util._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField, RegWriteFn}
import freechips.rocketchip.tilelink._
import uciephytest.phy.{Phy, PhyToTestIO}

case class UciephyTestParams(
  address: BigInt = 0x4000,
  bufferDepthPerLane: Int = 10,
  numLanes: Int = 2,
)

case object UciephyTestKey extends Field[Option[UciephyTestParams]](None)


// TX test modes.
object TxTestMode extends ChiselEnum {
  // Data to send is provided manually via `txDataOffset` and `txDataChunkIn`.
  val manual = Value(0.U(1.W))
  // Data to send is derived from an LFSR.
  val lfsr = Value(1.U(1.W))
}

// TX valid framing modes.
object TxValidFramingMode extends ChiselEnum {
  // Valid framing is done according to the UCIe spec (high for 4 UI, low for 4 UI).
  val ucie = Value(0.U(1.W))
  // Valid is high while valid data is being transmitted, and low when data is no longer valid.
  val simple = Value(1.U(1.W))
}

// State of the TX test FSM.
object TxTestState extends ChiselEnum {
  // Awaiting configuration and start of transmission.
  val idle = Value(0.U(2.W))
  // Test is currently being run.
  val run = Value(1.U(2.W))
  // Test is complete.
  val done = Value(2.U(2.W))
}

class UciephyTopIO(numLanes: Int = 2) extends Bundle {
  val txData = Output(Vec(numLanes, Bool()))
  val txValid = Output(Bool())
  val refClkP = Input(Clock())
  val refClkN = Input(Clock())
  val txClkP = Output(Clock())
  val txClkN = Output(Clock())
  val rxData = Input(Vec(numLanes, Bool()))
  val rxValid = Input(Bool())
  val rxClkP = Input(Clock())
  val rxClkN = Input(Clock())
  val pllIref = Input(Bool())
}

class UciephyTestMMIO(bufferDepthPerLane: Int = 10, numLanes: Int = 2) extends Bundle {
  // TODO: add additional control signals
  // TX CONTROL
  // =====================
  // The test mode of the TX.
  val txTestMode = Input(TxTestMode())
  // The valid framing mode of the TX.
  val txValidFramingMode = Input(TxValidFramingMode())
  // Seed of the TX LFSR.
  val txLfsrSeed = Input(Vec(numLanes, UInt(Phy.SerdesRatio.W)))
  // Repeats the TX manual transmission indefinitely. Useful for sending a long custom pattern
  // and verifying the `rxSignature`.
  val txManualRepeat = Input(Bool())
  // Resets the TX FSM (i.e. resetting the number of bits sent to 0, reseeding the LFSR,
  // and stopping any in-progress transmissions).
  val txFsmRst = Input(Bool())
  // Starts a transmission starting from the beginning of the input buffer (`TxTestMode.manual`) or from
  // the current state of the LFSR (`TxTestMode.lsfr`). Does not do anything if a transmission is in progress.
  val txExecute = Input(Bool())
  // The number of bits sent since the last FSM reset.
  val txBitsSent = Output(UInt(bufferDepthPerLane.W))
  // The number of bits to send during transmission. Set to 0 to send the entire buffer (`TxTestMode.manual`)
  // or continuously (`TxTestMode.lsfr`). Numbers greater than the buffer length will send the entire buffer
  // in `TxTestMode.manual`.
  val txBitsToSend = Input(UInt(32.W))
  // Data chunk lane in input buffer.
  val txDataLane = Input(UInt(log2Ceil(numLanes).W))
  // Data chunk offset in input buffer.
  val txDataOffset = Input(UInt((bufferDepthPerLane - 6).W))
  // 64-bit data chunk to write.
  val txDataChunkIn = Flipped(DecoupledIO(UInt(64.W)))
  // Data chunk at the given chunk offset for inspecting the data to be sent.
  val txDataChunkOut = Output(UInt(64.W))
  // Permutation of data fed into the serializer, in case serializer timing is off.
  val txPermute = Input(Vec(Phy.SerdesRatio, UInt(log2Ceil(Phy.SerdesRatio).W)))
  // State of the TX test FSM.
  val txTestState = Output(TxTestState())

  // RX CONTROL
  // ====================
  // Seed of the RX LFSR used for detecting bit errors. Should be the same as the TX seed of the transmitting chiplet.
  val rxLfsrSeed = Input(Vec(numLanes, UInt(Phy.SerdesRatio.W)))
  // The number of bit periods that valid must be high for the transmission to be considered valid. By default,
  // this is 4 since UCIe specifies asserting valid for 4 bit periods then de-asserting valid for 4 bit periods
  // during transmission. If the valid transmission is buggy, can set this to 1 such that data is received as long
  // as valid goes high for at least 1 bit period.
  val rxValidStartThreshold = Input(UInt(log2Ceil(Phy.DigitalBitsPerCycle).W))
  // The number of bit periods that valid must be low for the transmission to be considered invalid. By default,
  // this is 4 since UCIe specifies asserting valid for 4 bit periods then de-asserting valid for 4 bit periods
  // during transmission. If the valid transmission is buggy, can set this to 0 so that
  // data starts being received as soon as valid goes high and ignores any glitches afterwards.
  val rxValidStopThreshold = Input(UInt(log2Ceil(Phy.DigitalBitsPerCycle).W))
  // Resets the RX FSM (i.e. resetting the number of bits received and the offset within the output
  // buffer to 0).
  val rxFsmRst = Input(Bool())
  // The number of bits received since the last FSM reset. Only the first 2^bufferDepthPerLane bits received 
  // per lane are stored in the output buffer.
  val rxBitsReceived = Output(UInt(bufferDepthPerLane.W))
  // The number of bit errors since the last FSM reset. Only applicable in `TxTestMode.lsfr`.
  val rxBitErrors = Output(UInt(bufferDepthPerLane.W))
  // A MISR derived from the bits received since the last FSM reset.
  val rxSignature = Output(UInt(32.W))
  // Data chunk lane in output buffer.
  val rxDataLane = Input(UInt(log2Ceil(numLanes).W))
  // Data chunk offset in output buffer.
  val rxDataOffset = Input(UInt((bufferDepthPerLane - 6).W))
  // Data chunk at the given chunk offset for inspect the received data.
  val rxDataChunk = Output(UInt(64.W))
  // Valid chunk at the given chunk offset for inspect the valid signals corresponding to the received data.
  // In a correct UCIe implementation, there should be 4 1-bits followed by 4 0-bits repeated across the entire 
  // transmission.
  val rxValidChunk = Output(UInt(64.W))
  // Permutation of data read from the deserializer, in case deserializer timing is off.
  val rxPermute = Input(Vec(Phy.SerdesRatio, UInt(log2Ceil(Phy.SerdesRatio).W)))
}

class UciephyTestIO(bufferDepthPerLane: Int = 10, numLanes: Int = 2) extends Bundle {
  val mmio = new UciephyTestMMIO(bufferDepthPerLane, numLanes)

  // PHY INTERFACE
  // ====================
  val phy = Flipped(new PhyToTestIO(numLanes))
}

class UciephyTest(bufferDepthPerLane: Int = 10, numLanes: Int = 2) extends Module {
  val io = IO(new UciephyTestIO(bufferDepthPerLane, numLanes))

  // TX registers
  val txReset = io.mmio.txFsmRst || reset.asBool
  val txState = withReset(txReset) { RegInit(TxTestState.idle) }
  val packetsEnqueued = withReset(txReset) { RegInit(VecInit(Seq.fill(numLanes + 1)(0.U(bufferDepthPerLane.W)))) }

  // RX registers
  val rxReset = io.mmio.rxFsmRst || reset.asBool
  val rxBitsReceived = withReset(rxReset) { RegInit(0.U((bufferDepthPerLane + 1).W)) }
  val rxBitErrors = withReset(rxReset) { RegInit(0.U((bufferDepthPerLane + 1).W)) }
  val rxSignature = withReset(rxReset) { RegInit(0.U(32.W)) }

  val totalChunks = numLanes * (1 << (bufferDepthPerLane - 6))
  val addrBits = log2Ceil(totalChunks)

  val inputBuffer = Reg(Vec(numLanes, Vec(1 << (bufferDepthPerLane - 6), UInt(64.W))))
  val outputDataBuffer = Reg(Vec(numLanes, Vec(1 << (bufferDepthPerLane - 6), UInt(64.W))))
  val outputValidBuffer = Reg(Vec(1 << (bufferDepthPerLane - 6), UInt(64.W)))

  // Use valid lane as reference for how many bits were sent.
  io.mmio.txBitsSent := packetsEnqueued(numLanes) << log2Ceil(Phy.DigitalBitsPerCycle)
  io.mmio.txDataChunkIn.ready := txState === TxTestState.idle
  io.mmio.txDataChunkOut := inputBuffer(io.mmio.txDataLane)(io.mmio.txDataOffset)
  io.mmio.txTestState := txState

  io.mmio.rxBitsReceived := rxBitsReceived
  io.mmio.rxBitErrors := rxBitErrors
  io.mmio.rxSignature := rxSignature
  io.mmio.rxDataChunk := outputDataBuffer(io.mmio.rxDataLane)(io.mmio.rxDataOffset)
  io.mmio.rxValidChunk := outputValidBuffer(io.mmio.rxDataOffset)

  for (lane <- 0 until numLanes) {
    io.phy.txTransmitData(lane).bits := 0.U
    io.phy.txTransmitData(lane).valid := false.B
    io.phy.rxReceiveData(lane).ready := false.B
  }
  io.phy.txTransmitValid.bits := 0.U
  io.phy.txTransmitValid.valid := false.B
  io.phy.rxReceiveValid.ready := false.B

  // TX logic
  switch(txState) {
    is(TxTestState.idle) {
      when (io.mmio.txDataChunkIn.valid) {
        inputBuffer(io.mmio.txDataLane)(io.mmio.txDataOffset) := io.mmio.txDataChunkIn.bits
      }

      when (io.mmio.txExecute) {
        txState := TxTestState.run
      }
    }
    is(TxTestState.run) {
      val dividedInputBuffer = inputBuffer.asTypeOf(Vec(numLanes, Vec((1 << bufferDepthPerLane) / Phy.DigitalBitsPerCycle, UInt(Phy.DigitalBitsPerCycle.W))))
      for (lane <- 0 until numLanes) {
        io.phy.txTransmitData(lane).bits := dividedInputBuffer(lane)(packetsEnqueued(lane))
        io.phy.txTransmitData(lane).valid := (packetsEnqueued(lane) << log2Ceil(Phy.DigitalBitsPerCycle)) < io.mmio.txBitsToSend
      }
      switch(io.mmio.txValidFramingMode) {
        is (TxValidFramingMode.ucie) {
          io.phy.txTransmitValid.bits := VecInit((0 until Phy.DigitalBitsPerCycle/8).flatMap(_ => Seq.fill(4)(true.B) ++ Seq.fill(4)(false.B))).asUInt
        }
        is (TxValidFramingMode.simple) {
          io.phy.txTransmitValid.bits := VecInit(Seq.fill(Phy.DigitalBitsPerCycle)(true.B)).asUInt
        }
      }
      io.phy.txTransmitValid.valid := (packetsEnqueued(numLanes) << log2Ceil(Phy.DigitalBitsPerCycle)) < io.mmio.txBitsToSend
      
      for (lane <- 0 to numLanes) {
        val digitalLane = if (lane < numLanes) { io.phy.txTransmitData(lane) } else { io.phy.txTransmitValid }
        when (digitalLane.valid && digitalLane.ready) {
          packetsEnqueued(lane) := packetsEnqueued(lane) + 1.U
        }
      }

      when ((0 to numLanes).map(lane => {
          if (lane < numLanes) { !io.phy.txTransmitData(lane).valid } else { !io.phy.txTransmitValid.valid }
        }).reduce(_ && _)) {
        txState := TxTestState.done
      }
    }
    is(TxTestState.done) {
    }
  }

  // RX logic
  
  // Only dequeue if all lanes can be dequeued simultaneously.
  val ready = (0 to numLanes).map(lane => {
          if (lane < numLanes) { io.phy.rxReceiveData(lane).valid } else { io.phy.rxReceiveValid.valid }
        }).reduce(_ && _)

  for (lane <- 0 until numLanes) {
    io.phy.rxReceiveData(lane).ready := ready
  }
  io.phy.rxReceiveValid.ready := ready

  // Dumb RX logic (only sets threshold to start recording)
  val recordingStarted = withReset(rxReset) { RegInit(false.B) }
  val validHighStreak = withReset(rxReset) { RegInit(0.U(log2Ceil(Phy.DigitalBitsPerCycle).W)) }
  val prevDataBits = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes)(0.U(Phy.DigitalBitsPerCycle.W)))) }

  val startRecording = Wire(Bool())
  val startIdx = Wire(UInt(log2Ceil(Phy.DigitalBitsPerCycle).W))
  startRecording := false.B
  startIdx := 0.U

  // Check valid streak after each packet is dequeued.
  when (io.phy.rxReceiveValid.ready & io.phy.rxReceiveValid.valid) {
    // Store data in a register
    for (lane <- 0 until numLanes) {
      prevDataBits(lane) := io.phy.rxReceiveData(lane).bits
    }

    val mask = Wire(UInt(Phy.DigitalBitsPerCycle.W))
    mask := (1.U << io.mmio.rxValidStartThreshold) - 1.U
    // Find correct start index if recording hasn't started already.
    for (i <- Phy.DigitalBitsPerCycle - 1 to 0 by -1) {
      val shouldStartRecording = ((io.phy.rxReceiveValid.bits >> i.U) & mask) === mask
      when(!recordingStarted && shouldStartRecording) {
        startRecording := true.B
        startIdx := i.U
      }
    }

    for (i <- Phy.DigitalBitsPerCycle to 0 by -1) {
      val mask = Wire(UInt(Phy.DigitalBitsPerCycle.W))
      mask := ~((1.U << i.U) - 1.U)
      when ((io.phy.rxReceiveValid.bits & mask) === mask) {
        validHighStreak := (Phy.DigitalBitsPerCycle - i).U
      }
    }

    recordingStarted := recordingStarted || startRecording


    val rxBitsReceivedOffset = rxBitsReceived % 64.U
    val rxBlock0 = rxBitsReceived >> 6.U
    val rxBlock1 = rxBlock0 + 1.U
    // Insert new values into the register buffers at the appropriate offset.
    when (startIdx === 0.U && startRecording) {
      val numExtraBits = Phy.DigitalBitsPerCycle.U - validHighStreak
      val dataOffset = validHighStreak + rxBitsReceivedOffset
      val prevMask = Wire(UInt(128.W))
      prevMask := ((1.U << validHighStreak) - 1.U) << rxBitsReceivedOffset
      val dataMask = Wire(UInt(128.W))
      dataMask := ((1.U << Phy.DigitalBitsPerCycle.U) - 1.U) << dataOffset
      val keepMask = Wire(UInt(128.W))
      keepMask := ~(prevMask | dataMask)
      for (lane <- 0 until numLanes) {
        val prev = Wire(UInt(128.W))
        prev := (prevDataBits(lane) << (rxBitsReceived % 64.U)) >> numExtraBits
        val data = Wire(UInt(128.W))
        data := io.phy.rxReceiveData(lane).bits << dataOffset
        val keep = Wire(UInt(128.W))
        keep := Cat(outputDataBuffer(lane)(rxBlock1), outputDataBuffer(lane)(rxBlock0))
        val newData = Wire(UInt(128.W))
        newData := (prev & prevMask) | (data & dataMask) | (keep & keepMask)
        outputDataBuffer(lane)(rxBlock0) := newData(63, 0)
        outputDataBuffer(lane)(rxBlock1) := newData(127, 64)
      }
      val data = Wire(UInt(128.W))
      data := io.phy.rxReceiveValid.bits << dataOffset
      val keep = Wire(UInt(128.W))
      keep := Cat(outputValidBuffer(rxBlock1), outputValidBuffer(rxBlock0))
      val newData = Wire(UInt(128.W))
      newData := prevMask | (data & dataMask) | (keep & keepMask)
      outputValidBuffer(rxBlock0) := newData(63, 0)
      outputValidBuffer(rxBlock1) := newData(127, 64)
      rxBitsReceived := rxBitsReceived +& Phy.DigitalBitsPerCycle.U +& validHighStreak
    } .otherwise {
      val dataMask = Wire(UInt(128.W))
      dataMask := ((1.U << (Phy.DigitalBitsPerCycle.U - startIdx)) - 1.U) << rxBitsReceivedOffset
      val keepMask = Wire(UInt(128.W))
      keepMask := ~dataMask
      when (recordingStarted || startRecording) {
        for (lane <- 0 until numLanes) {
          val data = Wire(UInt(128.W))
          data := (io.phy.rxReceiveData(lane).bits << rxBitsReceivedOffset) >> startIdx
          val keep = Wire(UInt(128.W))
          keep := Cat(outputDataBuffer(lane)(rxBlock1), outputDataBuffer(lane)(rxBlock0))
          val newData = Wire(UInt(128.W))
          newData := (data & dataMask) | (keep & keepMask)
          outputDataBuffer(lane)(rxBlock0) := newData(63, 0)
          outputDataBuffer(lane)(rxBlock1) := newData(127, 64)
        }
        val data = Wire(UInt(128.W))
        data := (io.phy.rxReceiveValid.bits << rxBitsReceivedOffset) >> startIdx
        val keep = Wire(UInt(128.W))
        keep := Cat(outputValidBuffer(rxBlock1), outputValidBuffer(rxBlock0))
        val newData = Wire(UInt(128.W))
        newData := (data & dataMask) | (keep & keepMask)
        outputValidBuffer(rxBlock0) := newData(63, 0)
        outputValidBuffer(rxBlock1) := newData(127, 64)
        rxBitsReceived := rxBitsReceived +& Phy.DigitalBitsPerCycle.U - startIdx
      }
    }
  }
}

class Esd extends BlackBox {
  val io = IO(new Bundle {
    val term = Input(Bool())
  })

  override val desiredName = "AA_ESD"
}

class UciephyTestTL(params: UciephyTestParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("uciephytest", Seq("ucbbar,uciephytest")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  val topIO = BundleBridgeSource(() => new UciephyTopIO(params.numLanes))
  override lazy val module = new UciephyTestImpl
  class UciephyTestImpl extends Impl {
    val io = IO(new Bundle {})
    withClockAndReset(clock, reset) {
      // TEST HARNESS
      val test = Module(new UciephyTest(params.bufferDepthPerLane, params.numLanes))

      // MMIO registers.
      val txTestMode = RegInit(0.U(1.W))
      val txValidFramingMode = RegInit(0.U(1.W))
      val txLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes)(0.U(Phy.SerdesRatio.W))))
      val txManualRepeat = RegInit(0.U(1.W))
      val txFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val txExecute = Wire(DecoupledIO(UInt(1.W)))
      val txBitsToSend = RegInit(0.U(32.W))
      val txDataLane = RegInit(0.U(log2Ceil(params.numLanes).W))
      val txDataOffset = RegInit(0.U((params.bufferDepthPerLane - 6).W))
      val txPermute = RegInit(VecInit((0 until Phy.SerdesRatio).map(_.U(log2Ceil(Phy.SerdesRatio).W))))
      val rxLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes)(0.U(Phy.SerdesRatio.W))))
      val rxValidStartThreshold = RegInit(4.U(log2Ceil(Phy.DigitalBitsPerCycle).W))
      val rxValidStopThreshold = RegInit(4.U(log2Ceil(Phy.DigitalBitsPerCycle).W))
      val rxFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val rxDataLane = RegInit(0.U(log2Ceil(params.numLanes).W))
      val rxDataOffset = RegInit(0.U((params.bufferDepthPerLane - 6).W))
      val rxPermute = RegInit(VecInit((0 until Phy.SerdesRatio).map(_.U(log2Ceil(Phy.SerdesRatio).W))))
      val driverPuCtl = RegInit(VecInit(Seq.fill(params.numLanes + 5)(0.U(48.W))))
      val driverPdCtl = RegInit(VecInit(Seq.fill(params.numLanes + 5)(0.U(48.W))))
      val driverEn = RegInit(VecInit(Seq.fill(params.numLanes + 5)(false.B)))
      val clockingMiscCtl = RegInit(VecInit(Seq.fill(params.numLanes + 2)(0.U(64.W))))
      val clockingEnCtl = RegInit(VecInit(Seq.fill(params.numLanes + 2)(0.U(64.W))))
      val clockingEnbCtl = RegInit(VecInit(Seq.fill(params.numLanes + 2)(~0.U(64.W))))
      val terminationCtl = RegInit(VecInit(Seq.fill(params.numLanes + 3)(~0.U(64.W))))
      val vrefCtl = RegInit(VecInit(Seq.fill(params.numLanes + 1)(0.U(64.W))))

      txFsmRst.ready := true.B
      txExecute.ready := true.B
      rxFsmRst.ready := true.B

      test.io.mmio.txTestMode := TxTestMode(txTestMode)
      test.io.mmio.txValidFramingMode := TxValidFramingMode(txTestMode)
      test.io.mmio.txLfsrSeed := txLfsrSeed
      test.io.mmio.txManualRepeat := txManualRepeat
      test.io.mmio.txFsmRst := txFsmRst.valid
      test.io.mmio.txExecute := txExecute.valid
      test.io.mmio.txBitsToSend := txBitsToSend
      test.io.mmio.txDataLane := txDataLane
      test.io.mmio.txDataOffset := txDataOffset
      test.io.mmio.txPermute := txPermute
      test.io.mmio.rxLfsrSeed := rxLfsrSeed
      test.io.mmio.rxValidStartThreshold := rxValidStartThreshold
      test.io.mmio.rxValidStopThreshold := rxValidStopThreshold
      test.io.mmio.rxFsmRst := rxFsmRst.valid
      test.io.mmio.rxDataLane := rxDataLane
      test.io.mmio.rxDataOffset := rxDataOffset
      test.io.mmio.rxPermute := rxPermute

      // PHY
      val phy = Module(new Phy(params.numLanes))
      phy.io.test <> test.io.phy
      topIO.out(0)._1 <> phy.io.top

      phy.io.driverPuCtl := driverPuCtl
      phy.io.driverPdCtl := driverPdCtl
      phy.io.driverEn := driverEn
      phy.io.clockingMiscCtl := clockingMiscCtl
      phy.io.clockingEnCtl := clockingEnCtl
      phy.io.clockingEnbCtl := clockingEnbCtl
      phy.io.terminationCtl := terminationCtl
      phy.io.vrefCtl := vrefCtl

      val toRegField = (r: UInt) => {
        RegField(r.getWidth, r, RegWriteFn((valid, data) => {
          when (valid) {
            r := data
          }
          true.B
        }), None)
      }

      val ESD_txclkn = Module(new Esd)
      val ESD_txclkp = Module(new Esd)
      val ESD_txdata8 = Module(new Esd)
      val ESD_txvld = Module(new Esd)
      val ESD_txdata12 = Module(new Esd)
      val ESD_rxckp = Module(new Esd)
      val ESD_rxckn = Module(new Esd)
      val ESD_rxdata8 = Module(new Esd)
      val ESD_rxdata12 = Module(new Esd)
      val ESD_rxvld = Module(new Esd)
      val ESD_pll_iref = Module(new Esd)
      val ESD_clk_ref_p = Module(new Esd)
      val ESD_clk_ref_n = Module(new Esd)
      ESD_txclkn.io.term := phy.io.top.txClkN.asBool
      ESD_txclkp.io.term := phy.io.top.txClkP.asBool
      ESD_txdata8.io.term := phy.io.top.txData(0)
      ESD_txvld.io.term := phy.io.top.txValid
      ESD_txdata12.io.term := phy.io.top.txData(1)
      ESD_rxckp.io.term := topIO.out(0)._1.rxClkP.asBool
      ESD_rxckn.io.term := topIO.out(0)._1.rxClkN.asBool
      ESD_rxdata8.io.term := topIO.out(0)._1.rxData(0)
      ESD_rxdata12.io.term := topIO.out(0)._1.rxData(1)
      ESD_rxvld.io.term := topIO.out(0)._1.rxValid
      ESD_pll_iref.io.term := topIO.out(0)._1.pllIref
      ESD_clk_ref_p.io.term := phy.io.top.refClkP.asBool
      ESD_clk_ref_n.io.term := phy.io.top.refClkN.asBool

      var mmioRegs = Seq(
        toRegField(txTestMode),
        toRegField(txValidFramingMode),
      ) ++ (0 until params.numLanes).map((i: Int) => {
        toRegField(txLfsrSeed(i))
      }) ++ Seq(
        toRegField(txManualRepeat),
        RegField.w(1, txFsmRst),
        RegField.w(1, txExecute),
        RegField.r(params.bufferDepthPerLane, test.io.mmio.txBitsSent),
        toRegField(txBitsToSend),
        toRegField(txDataLane),
        toRegField(txDataOffset),
        RegField.w(64, test.io.mmio.txDataChunkIn),
        RegField.r(64, test.io.mmio.txDataChunkOut),
      ) ++ (0 until Phy.SerdesRatio).map((i: Int) => {
          toRegField(txPermute(i))
      }) ++ Seq(
        RegField.r(2, test.io.mmio.txTestState.asUInt),
      ) ++ (0 until params.numLanes).map((i: Int) => {
          toRegField(rxLfsrSeed(i))
      }) ++ Seq(
        toRegField(rxValidStartThreshold),
        toRegField(rxValidStartThreshold),
        RegField.w(1, rxFsmRst),
        RegField.r(params.bufferDepthPerLane, test.io.mmio.rxBitsReceived),
        RegField.r(params.bufferDepthPerLane, test.io.mmio.rxBitErrors),
        RegField.r(32, test.io.mmio.rxSignature),
        toRegField(rxDataLane),
        toRegField(rxDataOffset),
        RegField.r(64, test.io.mmio.rxDataChunk),
        RegField.r(64, test.io.mmio.rxValidChunk),
      ) ++ (0 until Phy.SerdesRatio).map((i: Int) => {
          toRegField(rxPermute(i))
      }) ++ (0 until params.numLanes + 5).flatMap((i: Int) => {
          Seq(
            toRegField(driverPuCtl(i)),
            toRegField(driverPuCtl(i)),
            toRegField(driverEn(i)),
          )
      }) ++ (0 until params.numLanes + 2).flatMap((i: Int) => {
          Seq(
            toRegField(clockingMiscCtl(i)),
            toRegField(clockingEnCtl(i)),
            toRegField(clockingEnbCtl(i)),
          )
      }) ++ (0 until params.numLanes + 3).flatMap((i: Int) => {
          Seq(
            toRegField(terminationCtl(i)),
          )
      }) ++ (0 until params.numLanes + 1).flatMap((i: Int) => {
          Seq(
            toRegField(vrefCtl(i)),
          )
      })

      node.regmap(mmioRegs.zipWithIndex.map({ case (f, i) => i * 8 -> Seq(f) }): _*)
    }
  }
}


trait CanHavePeripheryUciephyTest { this: BaseSubsystem =>
  private val portName = "uciephytest"

  private val pbus = locateTLBusWrapper(PBUS)

  val uciephy = p(UciephyTestKey) match {
    case Some(params) => {
      val uciephy = LazyModule(new UciephyTestTL(params, pbus.beatBytes)(p))
      uciephy.clockNode := pbus.fixedClockNode
      pbus.coupleTo(portName) { uciephy.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }
      Some(uciephy)
    }
    case None => None
  }
}

class WithUciephyTest(params: UciephyTestParams) extends Config((site, here, up) => {
  case UciephyTestKey => Some(params)
})
