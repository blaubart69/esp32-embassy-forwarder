#![no_std]

use embassy_net::tcp;
use esp_hal::uart;

trait ErrorHandler {
    fn handle_read_error(&self, context: &str) -> ErrorAction;
    fn handle_write_error(&self, context: &str) -> ErrorAction;
}

#[derive(Debug, Clone, Copy)]
enum ErrorAction {
    Continue,
    Break,
    Retry,
}

// Implement for UART errors
impl ErrorHandler for uart::RxError {
    fn handle_read_error(&self, context: &str) -> ErrorAction {
        match self {
            uart::RxError::FifoOverflowed => {
                // Use defmt or RTT for logging in embedded
                ErrorAction::Continue
            },
            uart::RxError::FrameFormatViolated => {
                ErrorAction::Continue
            },
            uart::RxError::GlitchOccurred => {
                ErrorAction::Continue
            },
            uart::RxError::ParityMismatch => {
                ErrorAction::Continue
            },
            _ => {
                ErrorAction::Continue
            }
        }
    }
    
    fn handle_write_error(&self, context: &str) -> ErrorAction {
        ErrorAction::Continue
    }
}

// Implement for TCP errors
impl ErrorHandler for tcp::Error: {
    fn handle_read_error(&self, context: &str) -> ErrorAction {
        match self {
            tcp::TcpReader:: => {
                defmt::error!("TCP connection reset in {}", context);
                ErrorAction::Break
            },
            tcp::TcpError::TimedOut => {
                defmt::warn!("TCP timeout in {}", context);
                ErrorAction::Retry
            },
            tcp::TcpError::ConnectionAborted => {
                defmt::error!("TCP connection aborted in {}", context);
                ErrorAction::Break
            },
            _ => {
                defmt::error!("Other TCP read error in {}: {:?}", context, self);
                ErrorAction::Continue
            }
        }
    }
    
    fn handle_write_error(&self, context: &str) -> ErrorAction {
        match self {
            tcp::TcpError::ConnectionReset | tcp::TcpError::ConnectionAborted => {
                defmt::error!("TCP write connection error in {}: {:?}", context, self);
                ErrorAction::Break
            },
            tcp::TcpError::TimedOut => {
                defmt::warn!("TCP write timeout in {}: {:?}", context, self);
                ErrorAction::Retry
            },
            _ => {
                defmt::error!("TCP write error in {}: {:?}", context, self);
                ErrorAction::Continue
            }
        }
    }
}

// Your single, reusable function
async fn read_write<R, W>(
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
    let mut retry_count = 0;
    const MAX_RETRIES: u8 = 3;
    
    loop {
        match rx.read(&mut buf).await {
            Err(e) => {
                match e.handle_read_error(context) {
                    ErrorAction::Continue => {
                        retry_count = 0; // Reset retry count on successful handling
                        continue;
                    },
                    ErrorAction::Break => break,
                    ErrorAction::Retry => {
                        retry_count += 1;
                        if retry_count >= MAX_RETRIES {
                            defmt::error!("Max retries exceeded in {}", context);
                            break;
                        }
                        // Maybe add a small delay here
                        continue;
                    }
                }
            }
            Ok(0) => {
                if break_on_zero_read {
                    break;
                }
            },
            Ok(bytes_read) => {
                retry_count = 0; // Reset on successful read
                
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