use embassy_net::{tcp::TcpSocket, IpListenEndpoint, Stack};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use esp_hal::{uart::Uart, Async};
use esp_println::println;
use log::info;

use crate::stats::Stats;

const PIPESIZE : usize = 2048;
type MyPipe = Pipe<NoopRawMutex, PIPESIZE>;

pub async fn accept_connection(stack: Stack<'_>, mut uart: Uart<'_, Async>, mut stats: &mut Stats) {
    let mut socket_rx_buffer = [0; 1024];
    let mut socket_tx_buffer = [0; 1024];

    let mut pipe1: MyPipe = Pipe::new();
    let (mut pipe_to_uart_rx, mut pipe_to_uart_tx) = pipe1.split();

    let mut pipe2: MyPipe = Pipe::new();
    let (mut pipe_to_tcp_rx, mut pipe_to_tcp_tx) = pipe2.split();

    let _ = uart.flush();
    let (mut uart_rx, mut uart_tx) = uart.split();

    let tcp_future = async {
        loop {
            // TODO: not sure IF it's needed... or how I should do it here
            //ensure_network(&stack).await;
            let mut sock = TcpSocket::new(stack, &mut socket_rx_buffer, &mut socket_tx_buffer);
            println!("I: waiting for connection");
            if let Err(err) = sock
                .accept(IpListenEndpoint {
                    addr: None,
                    port: 8080,
                })
                .await
            {
                println!("E: accept: {:?}", err);
            } else {
                let remote_ep = sock.remote_endpoint();
                println!("I: connection from {:?}", remote_ep);
                inc_atom(&stats.tcp.connections);
                let (mut tcp_rx, mut tcp_tx) = sock.split();
            
                let _ = embassy_futures::select::select(
                    read_write(&mut tcp_rx, &mut pipe_to_uart_tx, true, "TcpRx->Pipe"),
                    read_write(&mut pipe_to_tcp_rx, &mut tcp_tx, false, "Pipe->TcpTx"),
                ).await;

                info!("connection closed");
            }
        }
    };

    // Read + write from UART
    let uart_future = embassy_futures::join::join(
        read_write(&mut uart_rx,         &mut pipe_to_tcp_tx, false, "UartRx->Pipe"),
        read_write(&mut pipe_to_uart_rx, &mut uart_tx, false, "Pipe->UartTx"),
    );

    embassy_futures::join::join(tcp_future, uart_future).await;
    println!("THIS IS THE END, SHOULD NEVER HAPPEN. (will happen for sure)")
}

async fn read_write(rx : &mut impl embedded_io_async::Read, tx : &mut impl embedded_io_async::Write, break_on_zero_read : bool, context : &str) {
    let mut buf = [0; 1024];
    loop {
        match rx.read(&mut buf).await {
            Err( e) => {
                //println!("E: read() {} {:?}", context, e);
            }
            Ok(0) => {
                if break_on_zero_read {
                    break;
                }
            },
            Ok(bytes_read) => {
                match tx.write_all(&buf[0..bytes_read]).await {
                    Err(e) => println!("E: write_all {}: {:?}", context,e),
                    Ok(()) => {}
                }
            }
        }
    }
}





fn inc_atom(x: &portable_atomic::AtomicUsize) {
    x.add(1, core::sync::atomic::Ordering::Relaxed);
}

fn add_atom(x: &portable_atomic::AtomicUsize, val: usize) {
    x.add(val, core::sync::atomic::Ordering::Relaxed);
}
