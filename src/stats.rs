use core::fmt::Display;

use portable_atomic::AtomicUsize;

pub struct UartStats {
    pub(crate) bytes_read: AtomicUsize,
    bytes_written: AtomicUsize,
    err_read: AtomicUsize,
    err_write: AtomicUsize,
    written_not_equal_read: AtomicUsize,
    err_rx_fifo_overflowed: AtomicUsize,
}

impl UartStats {
    fn new() -> Self {
        UartStats {
            bytes_read: 0.into(),
            bytes_written: 0.into(),
            err_read: 0.into(),
            err_write: 0.into(),
            written_not_equal_read: 0.into(),
            err_rx_fifo_overflowed: 0.into(),
        }
    }
}

pub struct TcpStats {
    bytes_read: AtomicUsize,
    bytes_written: AtomicUsize,
    pub(crate) connections: AtomicUsize,
    err_read: AtomicUsize,
    err_write: AtomicUsize,
    written_not_equal_read: AtomicUsize,
}

impl TcpStats {
    fn new() -> Self {
        TcpStats {
            bytes_read: 0.into(),
            bytes_written: 0.into(),
            err_read: 0.into(),
            err_write: 0.into(),
            connections: 0.into(),
            written_not_equal_read: 0.into(),
        }
    }
}

pub struct Stats {
    pub(crate) uart: UartStats,
    pub(crate) tcp: TcpStats,
}

impl Stats {
    pub fn new() -> Self {
        Stats {
            uart: UartStats::new(),
            tcp: TcpStats::new(),
        }
    }
}

impl Display for Stats {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!(
"TCP
    bytes_read    : {}
    bytes_written : {}
    connections   : {}
    err_read      : {}
    err_write     : {}
    written!=read : {}
UART   
    bytes_read    : {}
    bytes_written : {}
    err_read      : {}
    err_write     : {}
    written!=read : {}",
        load_atom(&self.tcp.bytes_read),
        load_atom(&self.tcp.bytes_written),
        load_atom(&self.tcp.connections),
        load_atom(&self.tcp.err_read),
        load_atom(&self.tcp.err_write),
        load_atom(&self.tcp.written_not_equal_read),
        load_atom(&self.uart.bytes_read),
        load_atom(&self.uart.bytes_written),
        load_atom(&self.uart.err_read),
        load_atom(&self.uart.err_write),
        load_atom(&self.uart.written_not_equal_read)
        ))
    }
}

fn load_atom(x: &portable_atomic::AtomicUsize) -> usize {
    x.load(core::sync::atomic::Ordering::Relaxed)
}
