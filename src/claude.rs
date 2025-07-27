#![no_std]

use core::convert::Infallible;

use embassy_net::tcp;
use esp_hal::uart;
use esp_println::println;

pub(crate) trait ErrorHandler {
    fn handle_read_error(&self, context: &str) -> ErrorAction;
    fn handle_write_error(&self, context: &str) -> ErrorAction;
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum ErrorAction {
    Continue,
    Break,
    Retry,
}

// Implement for UART errors
impl ErrorHandler for uart::RxError {
    fn handle_read_error(&self, context: &str) -> ErrorAction {
        match self {
            uart::RxError::FifoOverflowed => {
                println!("E: UartRx {} {}", self, context);
                ErrorAction::Continue
            },
            uart::RxError::FrameFormatViolated => {
                println!("E: UartRx {} {}", self, context);
                ErrorAction::Continue
            },
            uart::RxError::GlitchOccurred => {
                println!("E: UartRx {} {}", self, context);
                ErrorAction::Continue
            },
            uart::RxError::ParityMismatch => {
                println!("E: UartRx {} {}", self, context);
                ErrorAction::Continue
            },
            _ => {
                println!("E: UartRx {} {}", self, context);
                ErrorAction::Continue
            }
        }
    }
    
    fn handle_write_error(&self, context: &str) -> ErrorAction {
        ErrorAction::Continue
    }
}

impl ErrorHandler for uart::TxError {
    fn handle_read_error(&self, context: &str) -> ErrorAction {
        ErrorAction::Continue
    }

    fn handle_write_error(&self, context: &str) -> ErrorAction {
        println!("E: UartTx {} {}", self, context);
        ErrorAction::Continue
    }
}

// Implement for TCP errors
impl ErrorHandler for tcp::Error {
    fn handle_read_error(&self, context: &str) -> ErrorAction {
        println!("E: TcpRx {:?} {}", self, context);
        match self {
            tcp::Error::ConnectionReset => {
                ErrorAction::Break
            },
            _ => {
                ErrorAction::Continue
            }
        }
    }
    
    fn handle_write_error(&self, context: &str) -> ErrorAction {
        println!("E: TcpTx {:?} {}", self, context);
        ErrorAction::Continue
    }
}

impl ErrorHandler for Infallible {
    fn handle_read_error(&self, context: &str) -> ErrorAction {
        ErrorAction::Continue
    }

    fn handle_write_error(&self, context: &str) -> ErrorAction {
        ErrorAction::Continue
    }
}

// Your single, reusable function
pub(crate) async fn read_write<R, W>(
    rx: &mut R, 
    tx: &mut W, 
    break_on_zero_read: bool, 
    context: &str
) 
where 
    R: embedded_io_async::Read,
    W: embedded_io_async::Write,
    R::Error: ErrorHandler,
    W::Error: ErrorHandler,
{
    let mut buf = [0; 1024];
    
    loop {
        match rx.read(&mut buf).await {
            Err(e) => {
                match e.handle_read_error(context) {
                    ErrorAction::Continue => {
                        continue;
                    },
                    ErrorAction::Break => break,
                    ErrorAction::Retry => continue
                }
            }
            Ok(0) => {
                if break_on_zero_read {
                    break;
                }
            },
            Ok(bytes_read) => {
                match tx.write_all(&buf[0..bytes_read]).await {
                    Err(e) => {
                        match e.handle_write_error(context) {
                            ErrorAction::Continue => {},
                            ErrorAction::Break => break,
                            ErrorAction::Retry => {
                                // For write errors, you might want to retry the write
                                // or just continue to next read
                            }
                        }
                    },
                    Ok(()) => {}
                }
            }
        }
    }
}