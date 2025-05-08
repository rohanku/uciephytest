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
import uciephytest.phy.{Phy, UciePllCtlIO, PhyToTestIO, TxLaneDigitalCtlIO, RxLaneDigitalCtlIO, RxLaneCtlIO, DriverControlIO, TxSkewControlIO, RxAfeIO}
import freechips.rocketchip.util.{AsyncQueueParams}
import testchipip.soc.{OBUS}
import edu.berkeley.cs.ucie.digital.tilelink._
import edu.berkeley.cs.ucie.digital.interfaces.{FdiParams, RdiParams, AfeParams}
import edu.berkeley.cs.ucie.digital.protocol.{ProtocolLayerParams}
import edu.berkeley.cs.ucie.digital.sideband.{SidebandParams}
import edu.berkeley.cs.ucie.digital.logphy.{LinkTrainingParams, TransmitPattern, RegisterRWIO, RegisterRW}

case class UciephyTestParams(
  address: BigInt = 0x4000,
  bufferDepthPerLane: Int = 11,
  numLanes: Int = 16,
  bitCounterWidth: Int = 64,
  protoParams: ProtocolLayerParams = ProtocolLayerParams(),
  tlParams: TileLinkParams = TileLinkParams(address = 0x0000, addressRange = ((2L << 32)-1), configAddress = 0x7000, inwardQueueDepth = 2, outwardQueueDepth = 2),
  fdiParams: FdiParams = FdiParams(width = 64, dllpWidth = 64, sbWidth = 32),
  rdiParams: RdiParams = RdiParams(width = 64, sbWidth = 32),
  sbParams: SidebandParams = SidebandParams(),
  linkTrainingParams: LinkTrainingParams = LinkTrainingParams(),
  afeParams: AfeParams = AfeParams(sbSerializerRatio = 1,
                                    sbWidth = 1,
                                    mbSerializerRatio = 32,
                                    mbLanes = 16,
                                    STANDALONE = false),
  laneAsyncQueueParams: AsyncQueueParams = AsyncQueueParams(),
  managerWhere: TLBusWrapperLocation = PBUS,
  sim: Boolean = false
)

case object UciephyTestKey extends Field[Option[Seq[UciephyTestParams]]](None)

object DataMode extends ChiselEnum {
  // Send/receive finite number of bits.
  val finite = Value(0.U(1.W))
  // Send/receive infinite amount of data.
  // In manual test mode, repeats the sent data bits.
  // In LFSR mode, continues sending LFSR data indefinitely.
  val infinite = Value(1.U(1.W))
}

object TestTarget extends ChiselEnum {
  // Receive from mainband once valid lane goes high.
  val mainband = Value(0.U(1.W))
  // Receive from loopback receiver as soon as first one is received.
  val loopback = Value(1.U(1.W))
}

// TX test modes.
object TxTestMode extends ChiselEnum {
  // Data to send is provided manually via `txDataOffset` and `txDataChunkIn`.
  val manual = Value(0.U(1.W))
  // Data to send is derived from an LFSR.
  val lfsr = Value(1.U(1.W))
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

class UciephyTestTLIO(numLanes: Int = 16) extends Bundle {
  val txData = Output(Vec(numLanes, Bool()))
  val txValid = Output(Bool())
  val txTrack = Output(Bool())
  val refClkP = Input(Clock())
  val refClkN = Input(Clock())
  val sbClk = Input(Clock())
  val txClkP = Output(Clock())
  val txClkN = Output(Clock())
  val bypassClkP = Input(Clock())
  val bypassClkN = Input(Clock())
  val rxData = Input(Vec(numLanes, Bool()))
  val rxValid = Input(Bool())
  val rxTrack = Input(Bool())
  val rxClkP = Input(Clock())
  val rxClkN = Input(Clock())
  val sbTxClk = Output(Clock())
  val sbTxData = Output(Bool())
  val sbRxClk = Input(Clock())
  val sbRxData = Input(Bool())
  val pllRdacVref = Input(Bool())
  val testPllRdacVref = Input(Bool())
  val debug = Output(new UciephyDebugIO())
}

class UciephyDebugIO() extends Bundle {
  // Test PLL differential clock.
  val testPllClkP = Bool()
  val testPllClkN = Bool()
  // Main PLL differential clock.
  val pllClkP = Bool()
  val pllClkN = Bool()
}

// bufferDepthPerLane: log2(# of bits stored per lane)
// numLanes: number of lanes
// bitCounterWidth: Width of counters for TX bits sent and RX bits received.
class UciephyTestMMIO(bufferDepthPerLane: Int = 10, numLanes: Int = 2, bitCounterWidth: Int = 64) extends Bundle {
  // GENERAL CONTROL
  // =====================
  // The test setup being targeted.
  val testTarget = Input(TestTarget())
  // TX CONTROL
  // =====================
  // The test mode of the TX.
  val txTestMode = Input(TxTestMode())
  // The data mode of the TX.
  val txDataMode = Input(DataMode())
  // Seed of the TX LFSR.
  val txLfsrSeed = Input(Vec(numLanes + 1, UInt((2 * Phy.SerdesRatio).W)))
  // Resets the TX FSM (i.e. resetting the number of bits sent to 0, reseeding the LFSR,
  // and stopping any in-progress transmissions).
  val txFsmRst = Input(Bool())
  // Starts a transmission starting from the beginning of the input buffer (`TxTestMode.manual`) or from
  // the current state of the LFSR (`TxTestMode.lsfr`). Does not do anything if a transmission is in progress.
  val txExecute = Input(Bool())
  // The number of packets of `Phy.SerdesRatio` bits sent since the last FSM reset.
  val txPacketsSent = Output(UInt(bitCounterWidth.W))
  // The number of 32-bit chunks per repeating period in TX transmission in manual mode.
  // Set to 0 to send the entire buffer. Numbers greater than the buffer length will send the entire buffer
  // in `TxTestMode.manual`.
  val txManualRepeatPeriod = Input(UInt((bufferDepthPerLane - 5 + 1).W))
  // The number of packets to send during transmission.
  val txPacketsToSend = Input(UInt(bitCounterWidth.W))
  // Enable clock P signal.
  val txClkPEn = Input(Bool())
  // Enable clock P signal.
  val txClkNEn = Input(Bool())
  // Enable track signal.
  val txTrackEn = Input(Bool())
  // Data chunk lane group in input buffer. Each lane group consists of 4 adjacent lanes (e.g. 0, 1, 2, 3).
  // Lane numLanes is valid, numLanes + 1 is track, numLanes + 2 is loopback.
  val txDataLaneGroup = Input(UInt((log2Ceil(numLanes + 3) - 2).max(1).W))
  // Data chunk offset in input buffer.
  val txDataOffset = Input(UInt((bufferDepthPerLane - 5).W))
  // 128-bit data chunk to write (32 bits per lane).
  val txDataChunkIn = Flipped(DecoupledIO(UInt(128.W)))
  // Data chunk at the given chunk offset for inspecting the data to be sent. Only available in idle/done mode.
  val txDataChunkOut = Output(UInt(128.W))
  // State of the TX test FSM.
  val txTestState = Output(TxTestState())

  // RX CONTROL
  // ====================
  // The data mode of the RX.
  val rxDataMode = Input(DataMode())
  // Seed of the RX LFSR used for detecting bit errors. Should be the same as the TX seed of the transmitting chiplet.
  val rxLfsrSeed = Input(Vec(numLanes + 1, UInt((2 * Phy.SerdesRatio).W)))
  // Resets the RX FSM (i.e. resetting the number of bits received and the offset within the output
  // buffer to 0).
  val rxFsmRst = Input(Bool())
  // The number of packets received since the last FSM reset. Only the first 2^bufferDepthPerLane bits received
  // per lane are stored in the output buffer.
  val rxPacketsReceived = Output(UInt(bitCounterWidth.W))
  // The number of packets to receive.
  val rxPacketsToReceive = Input(UInt(bitCounterWidth.W))
  // The number of bit errors per lane since the last FSM reset. Only applicable in `TxTestMode.lsfr`.
  // Extra lanes for valid errors (requires 1111000011110000...) and loopback.
  val rxBitErrors = Output(Vec(numLanes + 2, UInt(bitCounterWidth.W)))
  // Pause the `rxPacketsReceived` and `rxBitErrors` counters to read them atomically.
  val rxPauseCounters = Input(Bool())
  // A MISR derived from the packets received since the last FSM reset.
  val rxSignature = Output(UInt(32.W))
  // Data chunk lane in output buffer.
  val rxDataLane = Input(UInt(log2Ceil(numLanes + 3).W))
  // Data chunk offset in output buffer.
  val rxDataOffset = Input(UInt((bufferDepthPerLane - 5).W))
  // Data chunk at the given chunk offset for inspect the received data.
  val rxDataChunk = Output(UInt(32.W))
}

class UciephyTestIO(bufferDepthPerLane: Int = 10, numLanes: Int = 2, bitCounterWidth: Int = 64) extends Bundle {
  val mmio = new UciephyTestMMIO(bufferDepthPerLane, numLanes, bitCounterWidth)

  // PHY INTERFACE
  // ====================
  val phy = Flipped(new PhyToTestIO(numLanes))
}

class UciephyTest(bufferDepthPerLane: Int = 10, numLanes: Int = 2, bitCounterWidth: Int = 64) extends Module {
  val io = IO(new UciephyTestIO(bufferDepthPerLane, numLanes))

  // General computations
  val maxBitCount = VecInit(Seq.fill(bitCounterWidth)(true.B)).asUInt
  val maxSramPackets = 1.U << (bufferDepthPerLane - 5).U;

  // TX registers
  val txReset = io.mmio.txFsmRst || reset.asBool
  val txState = withReset(txReset) { RegInit(TxTestState.idle) }
  val txPacketsEnqueued = withReset(txReset) { RegInit(0.U(bitCounterWidth.W)) }
  val inputBufferAddrReg = withReset(txReset) { RegInit(0.U((bufferDepthPerLane - 5).W)) }
  val txLfsrs = (0 until numLanes + 1).map((i: Int) => {
    val lfsr = Module(
      new FibonacciLFSR(
        2 * Phy.SerdesRatio,
        taps = LFSR.tapsMaxPeriod.get(2 * Phy.SerdesRatio).get.head,
        step = Phy.SerdesRatio,
      ),
    )
    lfsr.io.seed.bits := io.mmio.txLfsrSeed(i).asTypeOf(lfsr.io.seed.bits)
    lfsr.io.seed.valid := txReset
    lfsr.io.increment := false.B
    lfsr
  })
  val loadedFirstChunk = withReset(txReset) { RegInit(false.B) }
  val txManualRepeatPeriod = Mux(io.mmio.txManualRepeatPeriod === 0.U || io.mmio.txManualRepeatPeriod > maxSramPackets, maxSramPackets, io.mmio.txManualRepeatPeriod)

  // RX registers
  val rxReset = io.mmio.rxFsmRst || reset.asBool
  val rxPacketsReceived = withReset(rxReset) { RegInit(0.U((64 - log2Ceil(Phy.SerdesRatio)).W)) }
  val rxReceiveOffset = withReset(rxReset) { RegInit(0.U(log2Ceil(Phy.SerdesRatio).W)) }
  /// numLanes data lanes, 1 valid lane, 1 loopback lane.
  val rxBitErrors = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes + 2)(0.U(64.W)))) }
  val rxPacketsReceivedOutput = withReset(rxReset) { RegInit(0.U(64.W)) }
  /// numLanes data lanes, 1 valid lane, 1 loopback lane.
  val rxErrorMask = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes + 2)(0.U(Phy.SerdesRatio.W)))) }
  val rxBitErrorsOutput = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes + 2)(0.U(64.W)))) }
  val rxLfsrs = (0 until numLanes + 1).map((i: Int) => {
    val lfsr = Module(
      new FibonacciLFSR(
        2 * Phy.SerdesRatio,
        taps = LFSR.tapsMaxPeriod.get(2 * Phy.SerdesRatio).get.head,
        step = Phy.SerdesRatio,
      ),
    )
    lfsr.io.seed.bits := io.mmio.rxLfsrSeed(i).asTypeOf(lfsr.io.seed.bits)
    lfsr.io.seed.valid := rxReset
    lfsr.io.increment := false.B
    lfsr
  })

  val rxSignature = withReset(rxReset) { RegInit(0.U(32.W)) }

  val numSrams = (numLanes + 2)/4 + 1
  val inputBuffer = (0 until numSrams).map(i => SyncReadMem(1 << (bufferDepthPerLane - 5), UInt(128.W)))
  val inputBufferAddr = Wire(UInt((bufferDepthPerLane - 5).W))
  inputBufferAddr := io.mmio.txDataOffset
  val inputRdPorts = (0 until numSrams).map(i => inputBuffer(i)(inputBufferAddr))
  val inputWrPorts = (0 until numSrams).map(i => inputBuffer(i)(io.mmio.txDataOffset))
  val outputBuffer = (0 until numSrams).map(i => SyncReadMem(1 << (bufferDepthPerLane - 5),  UInt(128.W)))
  val outputBufferAddr = Wire(UInt(log2Ceil(1 << (bufferDepthPerLane - 5)).W))
  outputBufferAddr := io.mmio.rxDataOffset
  val outputRdPorts = (0 until numSrams).map(i => outputBuffer(i)(io.mmio.rxDataOffset))
  val outputWrPorts = (0 until numSrams).map(i => outputBuffer(i)(outputBufferAddr))

  io.mmio.txPacketsSent := txPacketsEnqueued
  io.mmio.txDataChunkIn.ready := txState === TxTestState.idle
  io.mmio.txDataChunkOut := 0.U
  for (i <- 0 until numSrams) {
    when (i.U === io.mmio.txDataLaneGroup) {
      io.mmio.txDataChunkOut := inputRdPorts(i)
    }
  }
  io.mmio.txTestState := txState
  io.mmio.rxPacketsReceived := rxPacketsReceivedOutput
  io.mmio.rxBitErrors := rxBitErrorsOutput
  io.mmio.rxSignature := rxSignature
  io.mmio.rxDataChunk := 0.U
  for (i <- 0 until numSrams) {
    when (i.U === io.mmio.rxDataLane >> 2.U) {
      io.mmio.rxDataChunk := outputRdPorts(i).asTypeOf(Vec(4, UInt(32.W)))(io.mmio.rxDataLane % 4.U)
    }
  }

  for (lane <- 0 until numLanes) {
    io.phy.tx.bits.data(lane) := 0.U
  }
  io.phy.tx.bits.valid := 0.U
  io.phy.tx.bits.track := 0.U
  // Needs to always be true to send clock and track even when data isn't valid.
  io.phy.tx.valid := true.B

  // clkp = 101010...
  io.phy.tx.bits.clkp := Mux(io.mmio.txClkPEn, VecInit((0 until Phy.SerdesRatio/2).flatMap(_ => Seq(true.B, false.B))).asTypeOf(io.phy.tx.bits.clkp), VecInit(Seq.fill(Phy.SerdesRatio)(false.B)).asUInt)
  // clkn = 010101...
  io.phy.tx.bits.clkn := Mux(io.mmio.txClkNEn, VecInit((0 until Phy.SerdesRatio/2).flatMap(_ => Seq(false.B, true.B))).asTypeOf(io.phy.tx.bits.clkn), VecInit(Seq.fill(Phy.SerdesRatio)(false.B)).asUInt)

  io.phy.tx_loopback.bits := 0.U

  // Unlike `io.phy.tx.valid`, only true when data is valid.
  val tx_valid = Wire(Bool())
  tx_valid := false.B

  // TX logic
  switch(txState) {
    is(TxTestState.idle) {
      when (io.mmio.txDataChunkIn.valid) {
        for (i <- 0 until numSrams) {
          when (i.U === io.mmio.txDataLaneGroup) {
              inputWrPorts(i) := io.mmio.txDataChunkIn.bits
          }
        }
      }

      when (io.mmio.txExecute) {
        txState := TxTestState.run
      }
    }
    is(TxTestState.run) {
      switch (io.mmio.txTestMode) {
        is (TxTestMode.manual) {
          // Need to load first chunk ahead of time so that we can constantly send data.
          when (loadedFirstChunk) {
            // Increment address when packet is enqueued.
            when (io.phy.tx.ready) {
              inputBufferAddr := (inputBufferAddrReg + 1.U) % txManualRepeatPeriod
            } .otherwise {
              inputBufferAddr := inputBufferAddrReg % txManualRepeatPeriod
            }
            // Only send the next packet if we still need to send more bits.
            switch (io.mmio.txDataMode) {
              is(DataMode.finite) {
                tx_valid := txPacketsEnqueued < io.mmio.txPacketsToSend
              }
              is(DataMode.infinite) {
                tx_valid := true.B
              }
            }
          } .otherwise {
            inputBufferAddr := 0.U
            loadedFirstChunk := true.B
          }
        }
        is (TxTestMode.lfsr) {
          switch (io.mmio.txDataMode) {
            is(DataMode.finite) {
              tx_valid := txPacketsEnqueued < io.mmio.txPacketsToSend
            }
            is(DataMode.infinite) {
              tx_valid := true.B
            }
          }
        }
      }
      when (tx_valid) {
        switch(io.mmio.txTestMode) {
          is (TxTestMode.manual) {
            switch (io.mmio.testTarget) {
              is (TestTarget.mainband) {
                for (lane <- 0 until numLanes) {
                  io.phy.tx.bits.data(lane) := inputRdPorts(lane >> 2).asTypeOf(Vec(4, UInt(32.W)))(lane % 4)
                }
                io.phy.tx.bits.valid := inputRdPorts(numLanes >> 2).asTypeOf(Vec(4, UInt(32.W)))(numLanes % 4)
                io.phy.tx.bits.track := inputRdPorts((numLanes + 1) >> 2).asTypeOf(Vec(4, UInt(32.W)))((numLanes + 1) % 4)
              }
              is (TestTarget.loopback) {
                io.phy.tx_loopback.bits := inputRdPorts((numLanes + 2) >> 2).asTypeOf(Vec(4, UInt(32.W)))((numLanes + 2) % 4)
              }
            }
          }
          is (TxTestMode.lfsr) {
            switch (io.mmio.testTarget) {
              is (TestTarget.mainband) {
                for (lane <- 0 until numLanes) {
                  io.phy.tx.bits.data(lane) := Reverse(txLfsrs(lane).io.out.asUInt)(31, 0).asTypeOf(io.phy.tx.bits.data(lane))
                }
                io.phy.tx.bits.valid := VecInit((0 until Phy.SerdesRatio/8).flatMap(_ => Seq.fill(4)(true.B) ++ Seq.fill(4)(false.B))).asUInt
                // track = trackEn ? 101010... : 000000...
                io.phy.tx.bits.track := Mux(io.mmio.txTrackEn, io.phy.tx.bits.clkp, VecInit((0 until Phy.SerdesRatio/2).flatMap(_ => Seq(false.B, false.B))).asTypeOf(io.phy.tx.bits.track))
              }
              is (TestTarget.loopback) {
                io.phy.tx_loopback.bits := Reverse(txLfsrs(numLanes).io.out.asUInt)(31, 0).asTypeOf(io.phy.tx_loopback.bits)
              }
            }
          }
        }
      }

      when (tx_valid && io.phy.tx.ready) {
        txPacketsEnqueued := Mux(txPacketsEnqueued < VecInit(Seq.fill(txPacketsEnqueued.getWidth)(true.B)).asUInt, txPacketsEnqueued + 1.U, txPacketsEnqueued)
        inputBufferAddrReg := (inputBufferAddrReg + 1.U) % txManualRepeatPeriod
        when (io.mmio.txTestMode === TxTestMode.lfsr) {
          for (lane <- 0 until numLanes + 1) {
            txLfsrs(lane).io.increment := true.B
          }
        }
      }

      when (loadedFirstChunk && !tx_valid) {
        txState := TxTestState.done
      }
    }
    is(TxTestState.done) {
    }
  }
  io.phy.tx_loopback.valid := tx_valid

  // RX logic

  io.phy.rx.ready := true.B
  io.phy.rx_loopback.ready := true.B

  for (i <- 0 until numLanes + 2) {
    val newRxBitErrors = rxBitErrors(i) +& PopCount(rxErrorMask(i))
    rxBitErrors(i) := Mux(newRxBitErrors > maxBitCount, maxBitCount, newRxBitErrors)
  }
  when (io.mmio.rxPauseCounters) {
    rxPacketsReceivedOutput := rxPacketsReceivedOutput
    rxBitErrorsOutput := rxBitErrorsOutput
  } .otherwise {
    rxPacketsReceivedOutput := RegNext(rxPacketsReceived)
    rxBitErrorsOutput := rxBitErrors
  }

  // Dumb RX logic (starts recording as soon as valid goes high and never stops)
  val recordingStarted = withReset(rxReset) { RegInit(false.B) }
  val startRecording = Wire(Bool())
  val startIdx = Wire(UInt(log2Ceil(Phy.SerdesRatio).W))
  startRecording := false.B
  startIdx := 0.U

  // numLanes data lanes, 1 valid lane, 1 track lane, 1 loopback lane.
  val runningData = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes+3)(0.U(32.W)))) }

  for (i <- 0 until numLanes + 2) {
    rxErrorMask(i) := 0.U
  }

  // Check valid streak after each packet is dequeued.
  when (io.phy.rx.ready & io.phy.rx.valid) {

    // Find correct start index if recording hasn't started already.
    for (i <- Phy.SerdesRatio - 1 to 0 by -1) {
      val shouldStartRecording = Wire(Bool())
      shouldStartRecording := false.B
      switch (io.mmio.testTarget) {
        is (TestTarget.mainband) {
          shouldStartRecording := io.phy.rx.bits.valid(i)
        }
        is (TestTarget.loopback) {
          shouldStartRecording := io.phy.rx_loopback.bits(i)
        }
      }
      when(!recordingStarted && shouldStartRecording) {
        startRecording := true.B
        startIdx := i.U
        rxReceiveOffset := Phy.SerdesRatio.U - i.U
      }
    }

    recordingStarted := recordingStarted || startRecording


    when (!recordingStarted && !startRecording) {
      // Store latest data at the beginning of the `runningData` register.
      for (lane <- 0 until numLanes + 3) {
        if (lane < numLanes) {
          runningData(lane) := io.phy.rx.bits.data(lane)
        } else if (lane == numLanes) {
          runningData(lane) := io.phy.rx.bits.valid
        } else if (lane == numLanes + 1) {
          runningData(lane) := io.phy.rx.bits.track
        } else {
          runningData(lane) := io.phy.rx_loopback.bits
        }
      }
    } .otherwise {
      val fullPacketReceived = rxReceiveOffset +& Phy.SerdesRatio.U - startIdx >= 32.U
      val shouldProcessPacket = fullPacketReceived && (io.mmio.rxDataMode === DataMode.infinite || rxPacketsReceived < io.mmio.rxPacketsToReceive)
      val shouldWrite = rxPacketsReceived < maxSramPackets && fullPacketReceived
      val dataMask = Wire(UInt(64.W))
      dataMask := ((1.U << (Phy.SerdesRatio.U - startIdx)) - 1.U) << rxReceiveOffset
      val keepMask = Wire(UInt(64.W))
      keepMask := ~dataMask
      val toWrite = (0 until numSrams).map(i => {
        val wire = Wire(Vec(4, UInt(32.W)))
        for (i <- 0 until 4) {
          wire(i) := 0.U
        }
        wire
      })
      when (shouldProcessPacket) {
          rxPacketsReceived := Mux(rxPacketsReceived < VecInit(Seq.fill(rxPacketsReceived.getWidth)(true.B)).asUInt, rxPacketsReceived + 1.U, rxPacketsReceived)
      }
      for (lane <- 0 until numLanes + 2) {
        val rawData = if (lane < numLanes) {
          io.phy.rx.bits.data(lane)
        } else if (lane == numLanes) {
          io.phy.rx.bits.valid
        } else if (lane == numLanes) {
          io.phy.rx.bits.track
        } else {
          io.phy.rx_loopback.bits
        }
        val data = Wire(UInt(64.W))
        data := (rawData << rxReceiveOffset) >> startIdx
        val newData = Wire(UInt(64.W))
        newData := (data & dataMask) | (runningData(lane) & keepMask)
        runningData(lane) := newData(31, 0)
        when(fullPacketReceived) {
          runningData(lane) := newData >> 32.U
        }
        when(shouldWrite) {
          toWrite(lane >> 2)(lane % 4) := newData(31, 0)
        }

        when (shouldProcessPacket) {
          // Compare data against LFSR and increment LFSR.
          if (lane < numLanes) {
            rxLfsrs(lane).io.increment := true.B
            val lfsrData = newData(31, 0)
            rxErrorMask(lane) := newData(31, 0) ^ Reverse(rxLfsrs(lane).io.out.asUInt)(31, 0)
          }
          // Compare valid against intended waveform.
          if (lane == numLanes) {
            rxErrorMask(lane) := newData(31, 0) ^ "h0f0f_0f0f".U
          }
        }
      }
      when (shouldWrite) {
        outputBufferAddr := rxPacketsReceived
        for (i <- 0 until numSrams) {
          outputWrPorts(i) := toWrite(i).asTypeOf(outputWrPorts(i))
        }
      }
    }
  }
}


class UciephyTestTL(params: UciephyTestParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
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
  override lazy val desiredName = "UciephyTestTL"
  val device = new SimpleDevice("uciephytest", Seq("ucbbar,uciephytest"))
  val node = TLRegisterNode(Seq(AddressSet(params.address, 16384-1)), device, "reg/control", beatBytes=beatBytes)

  val topIO = BundleBridgeSource(() => new UciephyTestTLIO(params.numLanes))

  val uciTL = LazyModule(new UCITLFront(
        tlParams    = params.tlParams,
        protoParams = params.protoParams,
        fdiParams   = params.fdiParams,
        rdiParams   = params.rdiParams,
        sbParams    = params.sbParams,
        //myId        = myId,
        linkTrainingParams = params.linkTrainingParams,
        afeParams   = params.afeParams,
        laneAsyncQueueParams = params.laneAsyncQueueParams
      ))

  // // Diplomatic nodes to support Merged TL connections
  // val merged_tl_manager = TLMergedCreditedSource(TLMergedCreditedDelay(
  //                                                   CreditedDelay(4, 4),
  //                                                   CreditedDelay(4, 4)))

  // val merged_tl_client = TLMergedCreditedSink(TLMergedCreditedDelay(
  //                                                   CreditedDelay(4, 4),
  //                                                   CreditedDelay(4, 4)))

  // uciTL.managerNode <> merged_tl_manager
  // merged_tl_client <> uciTL.clientNode

  override lazy val module = new UciephyTestImpl
  class UciephyTestImpl extends Impl {
    val io = IO(new Bundle {})
    withClockAndReset(clock, reset) {
      // TEST HARNESS
      val test = Module(new UciephyTest(params.bufferDepthPerLane, params.numLanes, params.bitCounterWidth))

      // MMIO registers.
      val testTarget = RegInit(TestTarget.mainband)
      val txTestMode = RegInit(TxTestMode.manual)
      val txDataMode = RegInit(DataMode.finite)
      val txLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes + 1)(1.U(test.io.mmio.txLfsrSeed(0).getWidth.W))))
      val txFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val txExecute = Wire(DecoupledIO(UInt(1.W)))
      val txWriteChunk = Wire(DecoupledIO(UInt(1.W)))
      val txManualRepeatPeriod = RegInit(0.U(test.io.mmio.txManualRepeatPeriod.getWidth.W))
      val txPacketsToSend = RegInit(0.U(test.io.mmio.txPacketsToSend.getWidth.W))
      val txClkPEn = RegInit(false.B)
      val txClkNEn = RegInit(false.B)
      val txTrackEn = RegInit(false.B)
      val txDataLaneGroup = RegInit(0.U(test.io.mmio.txDataLaneGroup.getWidth.W))
      val txDataOffset = RegInit(0.U(test.io.mmio.txDataOffset.getWidth.W))
      val txDataChunkIn0 = RegInit(0.U(64.W))
      val txDataChunkIn1 = RegInit(0.U(64.W))
      val rxDataMode = RegInit(DataMode.infinite)
      val rxLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes + 1)(1.U(test.io.mmio.rxLfsrSeed(0).getWidth.W))))
      val rxFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val rxPacketsToReceive = RegInit(0.U(test.io.mmio.rxPacketsToReceive.getWidth.W))
      val rxPauseCounters = RegInit(0.U(1.W))
      val rxDataLane = RegInit(0.U(test.io.mmio.rxDataLane.getWidth.W))
      val rxDataOffset = RegInit(0.U(test.io.mmio.rxDataOffset.getWidth.W))

      val pllBypassEn = RegInit(false.B)
      val txctl = RegInit(VecInit(Seq.fill(params.numLanes + 5)({
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
      })))
      val rxctl = RegInit(VecInit(Seq.fill(params.numLanes + 5)({
        val w = Wire(new RxLaneDigitalCtlIO)
        w.zen := false.B
        w.zctl := 0.U
        w.vref_sel := 63.U
        w.afeBypassEn := false.B
        w.afeOpCycles := 16.U
        w.afeOverlapCycles := 2.U
        w.afeBypass.aEn := false.B
        w.afeBypass.aPc := true.B
        w.afeBypass.bEn := false.B
        w.afeBypass.bPc := true.B
        w.afeBypass.selA := false.B
        w.sample_negedge := false.B
        w.delay := 0.U
        w
        }
        )))
      val pllCtl = RegInit({
        val w = Wire(new UciePllCtlIO)
        w.dref_low := 30.U
        w.dref_high := 98.U
        w.dcoarse := 15.U
        w.d_kp := 50.U
        w.d_ki := 4.U
        w.d_clol := true.B
        w.d_ol_fcw := 0.U
        w.d_accumulator_reset := "h8000".U
        w.vco_reset := true.B
        w.digital_reset := true.B
        w
      })
      val testPllCtl = RegInit({
        val w = Wire(new UciePllCtlIO)
        w.dref_low := 30.U
        w.dref_high := 98.U
        w.dcoarse := 15.U
        w.d_kp := 50.U
        w.d_ki := 4.U
        w.d_clol := true.B
        w.d_ol_fcw := 0.U
        w.d_accumulator_reset := "h8000".U
        w.vco_reset := true.B
        w.digital_reset := true.B
        w
      })

      // UCIe logphy related
      val ucieStack = RegInit(false.B)
      val maxPatternCountWidth = log2Ceil(params.linkTrainingParams.maxPatternCount + 1)
      val pattern = RegInit(0.U(2.W))
      val patternUICount = RegInit(0.U(maxPatternCountWidth.W))
      val triggerNew = new RegisterRW(false.B, "triggerNew")
      val triggerExit = new RegisterRW(false.B, "triggerExit")
      val outputValid = RegInit(false.B)
      val errorCounts = RegInit(VecInit(Seq.fill(params.afeParams.mbLanes)(0.U(maxPatternCountWidth.W))))

      txFsmRst.ready := true.B
      txExecute.ready := true.B
      txWriteChunk.ready := true.B
      rxFsmRst.ready := true.B

      test.io.mmio.txDataChunkIn.bits := Cat(txDataChunkIn1, txDataChunkIn0)
      test.io.mmio.txDataChunkIn.valid := txWriteChunk.valid

      test.io.mmio.testTarget := testTarget
      test.io.mmio.txTestMode := txTestMode
      test.io.mmio.txDataMode := txDataMode
      test.io.mmio.txLfsrSeed := txLfsrSeed
      test.io.mmio.txFsmRst := txFsmRst.valid
      test.io.mmio.txExecute := txExecute.valid
      test.io.mmio.txManualRepeatPeriod := txManualRepeatPeriod
      test.io.mmio.txPacketsToSend := txPacketsToSend
      test.io.mmio.txClkPEn := txClkPEn
      test.io.mmio.txClkNEn := txClkPEn
      test.io.mmio.txTrackEn := txTrackEn
      test.io.mmio.txDataLaneGroup := txDataLaneGroup
      test.io.mmio.txDataOffset := txDataOffset
      test.io.mmio.rxDataMode := rxDataMode
      test.io.mmio.rxLfsrSeed := rxLfsrSeed
      test.io.mmio.rxFsmRst := rxFsmRst.valid
      test.io.mmio.rxPacketsToReceive := rxPacketsToReceive
      test.io.mmio.rxPauseCounters := rxPauseCounters
      test.io.mmio.rxDataLane := rxDataLane
      test.io.mmio.rxDataOffset := rxDataOffset

      // PHY
      val phy = Module(new Phy(params.numLanes, params.sim))
      when (ucieStack) {

        phy.io.test.tx <> uciTL.module.io.phyAfe.get.tx.map(f => {
          val x = Wire(chiselTypeOf(phy.io.test.tx.bits))
          x.data := f.data
          x.valid := f.valid.asTypeOf(x.valid)
          // clkp = 101010...
          x.clkp := VecInit((0 until Phy.SerdesRatio/2).flatMap(_ => Seq(true.B, false.B))).asTypeOf(x.clkp)
          // clkn = 010101...
          x.clkn := VecInit((0 until Phy.SerdesRatio/2).flatMap(_ => Seq(false.B, true.B))).asTypeOf(x.clkn)
          // track = 000000... (for now)
          x.track := VecInit((0 until Phy.SerdesRatio/2).flatMap(_ => Seq(false.B, false.B))).asTypeOf(x.track)
          x
        })

        phy.io.test.rx.map(f => {
          val x = Wire(chiselTypeOf(uciTL.module.io.phyAfe.get.rx.bits))
          x.data := f.data
          x.valid := f.valid.asTypeOf(x.valid)
          x
        }) <> uciTL.module.io.phyAfe.get.rx

        // TODO: delete txRst and rxRst from uciedigital

        phy.io.sideband.txClk := uciTL.module.io.txSbAfe.clk
        phy.io.sideband.txData := uciTL.module.io.txSbAfe.data
        uciTL.module.io.rxSbAfe.clk := phy.io.sideband.rxClk
        uciTL.module.io.rxSbAfe.data := phy.io.sideband.rxData

        test.io.phy.tx.nodeq()
        test.io.phy.rx.noenq()
        test.io.phy.tx_loopback.nodeq()
        test.io.phy.rx_loopback.noenq()
        phy.io.test.tx_loopback.noenq()
        phy.io.test.rx_loopback.nodeq()
      }.otherwise {
        phy.io.test <> test.io.phy
        uciTL.module.io.phyAfe.get.tx.nodeq()
        uciTL.module.io.phyAfe.get.rx.noenq()
        uciTL.module.io.rxSbAfe.clk := 0.U
        uciTL.module.io.rxSbAfe.data := 0.U.asTypeOf(uciTL.module.io.rxSbAfe.data)
        // Tie sideband to 0 for simple test
        phy.io.sideband.txClk := false.B
        phy.io.sideband.txData := false.B
      }

      uciTL.module.io.train.get.pattern := pattern.asTypeOf(TransmitPattern())
      uciTL.module.io.train.get.patternUICount := patternUICount
      triggerNew.connect(uciTL.module.io.train.get.triggerNew)
      triggerExit.connect(uciTL.module.io.train.get.triggerExit)
      outputValid := uciTL.module.io.train.get.outputValid
      errorCounts := uciTL.module.io.train.get.errorCounts

      topIO.out(0)._1 <> phy.io.top

      phy.io.pllBypassEn := pllBypassEn
      phy.io.txctl := txctl
      phy.io.pllCtl := pllCtl
      phy.io.testPllCtl := testPllCtl
      phy.io.rxctl := rxctl


      var mmioRegs = Seq(
        toRegFieldRw(testTarget, "testTarget"),
        toRegFieldRw(txTestMode, "txTestMode"),
        toRegFieldRw(txDataMode, "txDataMode"),
      ) ++ (0 until params.numLanes + 1).map((i: Int) => {
        toRegFieldRw(txLfsrSeed(i), s"txLfsrSeed_$i")
      }) ++ Seq(
        RegField.w(1, txFsmRst, RegFieldDesc("txFsmRst", "")),
        RegField.w(1, txExecute, RegFieldDesc("txExecute", "")),
        RegField.w(1, txWriteChunk, RegFieldDesc("txWriteChunk", "")),
        toRegFieldR(test.io.mmio.txPacketsSent, "txPacketsSent"),
        toRegFieldRw(txManualRepeatPeriod, "txManualRepeatPeriod"),
        toRegFieldRw(txPacketsToSend, "txPacketsToSend"),
        toRegFieldRw(txClkPEn, "txClkPEn"),
        toRegFieldRw(txClkNEn, "txClkNEn"),
        toRegFieldRw(txTrackEn, "txTrackEn"),
        toRegFieldRw(txDataLaneGroup, "txDataLaneGroup"),
        toRegFieldRw(txDataOffset, "txDataOffset"),
        toRegFieldRw(txDataChunkIn0, "txDataChunkIn0"),
        toRegFieldRw(txDataChunkIn1, "txDataChunkIn1"),
        toRegFieldR(test.io.mmio.txDataChunkOut(63, 0), "txDataChunkOut0"),
        toRegFieldR(test.io.mmio.txDataChunkOut(127, 64), "txDataChunkOut1"),
      ) ++ Seq(
        toRegFieldR(test.io.mmio.txTestState, "txTestState"),
        toRegFieldRw(rxDataMode, s"rxDataMode"),
      ) ++ (0 until params.numLanes + 1).map((i: Int) => {
          toRegFieldRw(rxLfsrSeed(i), s"rxLfsrSeed_$i")
      }) ++ (0 until params.numLanes + 2).map((i: Int) => {
        toRegFieldR(test.io.mmio.rxBitErrors(i), s"rxBitErrors_$i")
      }) ++ Seq(
        RegField.w(1, rxFsmRst, RegFieldDesc("rxFsmRst", "")),
        toRegFieldRw(rxPacketsToReceive, "rxPacketsToReceive"),
        toRegFieldRw(rxPauseCounters, "rxPauseCounters"),
        toRegFieldR(test.io.mmio.rxPacketsReceived, "rxPacketsReceived"),
        toRegFieldR(test.io.mmio.rxSignature, "rxSignature"),
        toRegFieldRw(rxDataLane, "rxDataLane"),
        toRegFieldRw(rxDataOffset, "rxDataOffset"),
        toRegFieldR(test.io.mmio.rxDataChunk, "rxDataChunk"),
        toRegFieldRw(pllCtl.dref_low, "pll_dref_low"),
        toRegFieldRw(pllCtl.dref_high, "pll_dref_high"),
        toRegFieldRw(pllCtl.dcoarse, "pll_dcoarse"),
        toRegFieldRw(pllCtl.d_kp, "pll_d_kp"),
        toRegFieldRw(pllCtl.d_ki, "pll_d_ki"),
        toRegFieldRw(pllCtl.d_clol, "pll_d_clol"),
        toRegFieldRw(pllCtl.d_ol_fcw, "pll_d_ol_fcw"),
        toRegFieldRw(pllCtl.d_accumulator_reset, "pll_d_accumulator_reset"),
        toRegFieldRw(pllCtl.vco_reset, "pll_vco_reset"),
        toRegFieldRw(pllCtl.digital_reset, "pll_digital_reset"),
        toRegFieldRw(testPllCtl.dref_low, "test_pll_dref_low"),
        toRegFieldRw(testPllCtl.dref_high, "test_pll_dref_high"),
        toRegFieldRw(testPllCtl.dcoarse, "test_pll_dcoarse"),
        toRegFieldRw(testPllCtl.d_kp, "test_pll_d_kp"),
        toRegFieldRw(testPllCtl.d_ki, "test_pll_d_ki"),
        toRegFieldRw(testPllCtl.d_clol, "test_pll_d_clol"),
        toRegFieldRw(testPllCtl.d_ol_fcw, "test_pll_d_ol_fcw"),
        toRegFieldRw(testPllCtl.d_accumulator_reset, "test_pll_d_accumulator_reset"),
        toRegFieldRw(testPllCtl.vco_reset, "test_pll_vco_reset"),
        toRegFieldRw(testPllCtl.digital_reset, "test_pll_digital_reset"),
        toRegFieldR(phy.io.pllOutput, "pllOutput"),
        toRegFieldR(phy.io.testPllOutput, "testPllOutput"),
        toRegFieldRw(pllBypassEn, "pllBypassEn")
      ) ++ (0 until params.numLanes + 5).flatMap((i: Int) => {
          Seq(
            toRegFieldRw(txctl(i).dll_reset, s"dll_reset_$i"),
            toRegFieldRw(txctl(i).driver, s"txctl_${i}_driver"),
            toRegFieldRw(txctl(i).skew, s"txctl_${i}_skew"),
            ) ++ (0 until 32).map((j: Int) => 
            toRegFieldRw(txctl(i).shuffler(j), s"txctl_${i}_shuffler_$j"),
            ) ++ Seq(
            toRegFieldRw(txctl(i).sample_negedge, s"txctl_${i}_sample_negedge"),
            toRegFieldRw(txctl(i).delay, s"txctl_${i}_delay"),
            toRegFieldR(phy.io.dllCode(i), s"dllCode_$i"),
          )
      }) ++ (0 until params.numLanes + 5).flatMap((i: Int) => {
          Seq(
            toRegFieldRw(rxctl(i).zen, s"zen_$i"),
            toRegFieldRw(rxctl(i).zctl, s"zctl_$i"),
            toRegFieldRw(rxctl(i).vref_sel, s"vref_sel_$i"),
            toRegFieldRw(rxctl(i).afeBypassEn, s"afeBypassEn_$i"),
            toRegFieldRw(rxctl(i).afeBypass, s"afeBypass_$i"),
            toRegFieldRw(rxctl(i).afeOpCycles, s"afeOpCycles_$i"),
            toRegFieldRw(rxctl(i).afeOverlapCycles, s"afeOverlapCycles_$i"),
            toRegFieldRw(rxctl(i).sample_negedge, s"sample_negedge_$i"),
            toRegFieldRw(rxctl(i).delay, s"rx_delay_$i"),
          )
      }) ++ Seq(
        RegField.w(1, ucieStack, RegFieldDesc("ucieStack", "")),
        RegField.r(1, outputValid, RegFieldDesc("outputValid", ""))
      ) ++ (0 until params.numLanes).map((i: Int) => {
        RegField.r(maxPatternCountWidth, errorCounts(i), RegFieldDesc(s"errorCounts_$i", ""))
      }) ++ Seq(
        RegField.w(2, pattern, RegFieldDesc("pattern", "")),
        RegField.w(32, patternUICount, RegFieldDesc("patternUICount", "")),
        RegField(1,
          RegReadFn(triggerNew.reg.asUInt),
          RegWriteFn((wen, data) => {
            when(wen) {
              triggerNew.reg := data.asBool
            }
            true.B
          }),
          RegFieldDesc("triggerNew", "training triggered")
        ),
        RegField(1,
          RegReadFn(triggerExit.reg.asUInt),
          RegWriteFn((wen, data) => {
            when(wen) {
              triggerExit.reg := data.asBool
            }
            true.B
          }),
          RegFieldDesc("triggerExit", "exit triggered")
        ),
      )

      node.regmap(mmioRegs.zipWithIndex.map({ case (f, i) => {
      i * 8 -> Seq(f)} }): _*)
    }
  }
}


trait CanHavePeripheryUciephyTest { this: BaseSubsystem =>
  private val portName = "uciephytest"

  private val pbus = locateTLBusWrapper(PBUS)
  private val sbus = locateTLBusWrapper(SBUS)

  val uciephy = p(UciephyTestKey) match {
    case Some(params) => {
      val uciephy = params.map(x => LazyModule(new UciephyTestTL(x, sbus.beatBytes)(p)))

      lazy val uciephy_tlbus = params.map(x => locateTLBusWrapper(x.managerWhere))

      for ((((ucie, ucie_params), tlbus), n) <- uciephy.zip(params).zip(uciephy_tlbus).zipWithIndex){
        ucie.clockNode := sbus.fixedClockNode
        ucie.uciTL.clockNode := sbus.fixedClockNode
        sbus.coupleTo(s"uciephytest{$n}") { ucie.node := TLBuffer() := TLFragmenter(sbus.beatBytes, sbus.blockBytes) := TLBuffer() := _ }
        pbus.coupleTo(s"ucie_tl_man_port{$n}") {
            ucie.uciTL.managerNode := TLWidthWidget(pbus.beatBytes) := TLBuffer() := TLSourceShrinker(ucie_params.tlParams.sourceIDWidth) := TLFragmenter(pbus.beatBytes, p(CacheBlockBytes)) := TLBuffer() := _
        } //manager node because SBUS is making request?
        sbus.coupleFrom(s"ucie_tl_cl_port{$n}") { _ := TLBuffer() := TLWidthWidget(sbus.beatBytes) := TLBuffer() := ucie.uciTL.clientNode }
        sbus.coupleTo(s"ucie_tl_ctrl_port{$n}") { ucie.uciTL.regNode.node := TLWidthWidget(sbus.beatBytes) := TLFragmenter(sbus.beatBytes, sbus.blockBytes) := TLBuffer() := _ }
      }
      Some(uciephy)
    }
    case None => None
  }
}

class WithUciephyTest(params: Seq[UciephyTestParams]) extends Config((site, here, up) => {
  case UciephyTestKey => Some(params)
})

class WithUciephyTestSim extends Config((site, here, up) => {
  case UciephyTestKey => up(UciephyTestKey, site).map(u => u.map(_.copy(sim = true)))
})
