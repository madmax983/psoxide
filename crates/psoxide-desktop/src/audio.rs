//! Host audio output for the desktop frontend.
//!
//! Bridges the `psoxide-core` SPU sample queue to the host sound device via
//! `rodio`. The core produces interleaved 44.1 kHz stereo `i16` samples; this
//! module drains them once per frame and appends them to a `rodio::Sink`.
//!
//! Latency model: samples are buffered by the sink with light backpressure —
//! if more than [`MAX_QUEUED_BUFFERS`] per-frame chunks are already queued
//! (~2 frames) the incoming chunk is dropped to keep latency bounded. There is
//! no resampling: the host is expected to consume at roughly 60 fps, which is
//! how fast the core produces ~735 sample pairs per frame. If no audio device
//! is available (e.g. a headless machine) audio initialisation fails
//! gracefully and playback is silently skipped — the emulator still runs.

use rodio::buffer::SamplesBuffer;
use rodio::{OutputStream, OutputStreamHandle, Sink};

/// SPU output sample rate in Hz.
const SAMPLE_RATE: u32 = 44_100;
/// Number of output channels (interleaved stereo).
const CHANNELS: u16 = 2;
/// Maximum number of per-frame chunks queued in the sink before backpressure
/// drops the incoming chunk (roughly two frames of latency).
const MAX_QUEUED_BUFFERS: usize = 2;

/// Host audio output backed by a `rodio` sink.
///
/// The `OutputStream` is retained (never dropped while playing) because
/// dropping it stops the stream.
pub struct AudioOutput {
    sink: Sink,
    _stream: OutputStream,
    _handle: OutputStreamHandle,
}

impl AudioOutput {
    /// Attempts to open the default host audio device.
    ///
    /// Returns `None` (after logging) when no device is available or the sink
    /// cannot be created, so the caller can continue running silently.
    #[must_use]
    pub fn try_new() -> Option<Self> {
        let (stream, handle) = match OutputStream::try_default() {
            Ok(pair) => pair,
            Err(err) => {
                eprintln!("audio: no output device ({err}); continuing silently");
                return None;
            }
        };
        let sink = match Sink::try_new(&handle) {
            Ok(sink) => sink,
            Err(err) => {
                eprintln!("audio: failed to create sink ({err}); continuing silently");
                return None;
            }
        };
        Some(Self {
            sink,
            _stream: stream,
            _handle: handle,
        })
    }

    /// Queues a chunk of interleaved-stereo samples for playback.
    ///
    /// Applies backpressure: if the sink already holds [`MAX_QUEUED_BUFFERS`]
    /// chunks the samples are dropped to bound latency. An empty chunk is
    /// ignored.
    pub fn queue(&self, samples: Vec<i16>) {
        if samples.is_empty() || self.sink.len() >= MAX_QUEUED_BUFFERS {
            return;
        }
        self.sink
            .append(SamplesBuffer::new(CHANNELS, SAMPLE_RATE, samples));
    }
}
