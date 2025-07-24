#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(type_alias_impl_trait)]

//extern crate alloc;

use core::net::{Ipv4Addr, SocketAddrV4};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pipe::Pipe;
use embedded_io::Error;
use log::{info};
use portable_atomic::AtomicUsize;

use embassy_executor::Spawner;
use embassy_net::tcp::{TcpReader, TcpSocket, TcpWriter};
use embassy_net::{DhcpConfig, IpListenEndpoint, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{self, Parity, RxConfig, RxError, StopBits, Uart, UartRx, UartTx};
use esp_hal::Async;
use esp_println::println;
use esp_wifi::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState,
};
use esp_wifi::EspWifiController;
use static_cell::StaticCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

struct UartStats {
    bytes_read: AtomicUsize,
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

struct TcpStats {
    bytes_read: AtomicUsize,
    bytes_written: AtomicUsize,
    connections: AtomicUsize,
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

struct Stats {
    uart: UartStats,
    tcp: TcpStats,
}

impl Stats {
    fn new() -> Self {
        Stats {
            uart: UartStats::new(),
            tcp: TcpStats::new(),
        }
    }
}

fn load_atom(x: &portable_atomic::AtomicUsize) -> usize {
    x.load(core::sync::atomic::Ordering::Relaxed)
}

fn print_stats(s: &Stats) {
    println!(
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
        load_atom(&s.tcp.bytes_read),
        load_atom(&s.tcp.bytes_written),
        load_atom(&s.tcp.connections),
        load_atom(&s.tcp.err_read),
        load_atom(&s.tcp.err_write),
        load_atom(&s.tcp.written_not_equal_read),
        load_atom(&s.uart.bytes_read),
        load_atom(&s.uart.bytes_written),
        load_atom(&s.uart.err_read),
        load_atom(&s.uart.err_write),
        load_atom(&s.uart.written_not_equal_read)
    );
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    esp_println::logger::init_logger(log::LevelFilter::Info);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);

    let timer1 = TimerGroup::new(peripherals.TIMG0);
    static WIFI_CONTROLLER_CELL: StaticCell<EspWifiController> = StaticCell::new();
    let esp_wifi_ctrl = WIFI_CONTROLLER_CELL
        .init(esp_wifi::init(timer1.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap());

    let (controller, interfaces) = esp_wifi::wifi::new(esp_wifi_ctrl, peripherals.WIFI).unwrap();
    let wifi_interface = interfaces.sta;
    
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    //let tls_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    let net_config = embassy_net::Config::dhcpv4(DhcpConfig::default());

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(StackResources::<3>, StackResources::<3>::new()),
        net_seed,
    );

    let mut uart = {
        let config = esp_hal::uart::Config::default()
            .with_baudrate(115200)
            .with_data_bits(esp_hal::uart::DataBits::_8)
            .with_parity(Parity::None)
            .with_stop_bits(StopBits::_1)
            .with_rx(esp_hal::uart::RxConfig::default().with_fifo_full_threshold(64));

        let uart0 = esp_hal::uart::Uart::new(peripherals.UART0, config)
            .unwrap()
            .with_tx(peripherals.GPIO21)
            .with_rx(peripherals.GPIO20)
            .into_async();

        // EOT (CTRL-D)
        //const AT_CMD: u8 = 0x04;
        //uart0.set_at_cmd(esp_hal::uart::AtCmdConfig::default().with_cmd_char(AT_CMD));

        println!("UART0 configuration: {:?}", config);

        uart0
    };

    spawner.spawn(connection_task(controller)).ok();
    spawner.spawn(net_task(runner)).ok();

    let mut stats = Stats::new();
    accept_connection(stack, uart, &mut stats).await;
}

const PIPESIZE : usize = 2048;
type MyPipe = Pipe<NoopRawMutex, PIPESIZE>;

async fn accept_connection(stack: Stack<'_>, mut uart: Uart<'_, Async>, mut stats: &mut Stats) {
    let mut socket_rx_buffer = [0; 1024];
    let mut socket_tx_buffer = [0; 1024];

    // Pipe setup
    let mut pipe1: MyPipe = Pipe::new();
    let (mut pipe_to_uart_rx, mut pipe_to_uart_tx) = pipe1.split();

    // komisch mit de Doppepunkt um den Typen herum
    //let (mut tcp_pipe_reader1, mut tcp_pipe_writer1) = Pipe::<NoopRawMutex, 20>::new().split();

    let mut pipe2: MyPipe = Pipe::new();
    let (mut pipe_to_tcp_rx, mut pipe_to_tcp_tx) = pipe2.split();

    let _ = uart.flush();
    // split()
    let (mut uart_rx, mut uart_tx) = uart.split();

    let tcp_future = async {
        loop {
            ensure_network(&stack).await;
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

async fn ensure_network(stack: &Stack<'_>) {
    loop {
        if !stack.is_config_up() {
            println!("Waiting for network stack to get configured...");
            Timer::after(Duration::from_millis(1000)).await
        } else if let Some(config) = stack.config_v4() {
            println!(
                "Got IP: {}, gateway: {:?}, DNS: {:?}",
                config.address, config.gateway, config.dns_servers
            );
            break;
        } else {
            println!("X: error getting network config after is_config_up(). bad.");
        }
    }
}

#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }

        //const SSID: &str = "OpenWrt";
        let SSID: &'static str = env!("ESP32_SSID");
        let PASSWORD: &'static str = env!("ESP32_PASS");

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start_async().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect to SSID {} ...", SSID);

        match controller.connect_async().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

/*
    * don't know why this isn't working

let listen_address = SocketAddrV4::new(Ipv4Addr::new(0, 0, 0, 0), 8080);
let listen_ep = IpListenEndpoint::from(listen_address);
println!("I: listening on: {}", listen_ep);
let x = sock.accept(listen_ep).await;
*/

/* macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}
 */
/*
async fn uart_read(
    uart_rx: &mut UartRx<'_, Async>,
    tcp_pipe_writer: &mut MyPipeWriter<'_>,
) -> ! {
    let mut buf = [0; 64];
    loop {
        match uart_rx.read_async(&mut buf).await {
            Err(rxe) => println!("E: uart_read {:?}", rxe),
            Ok(0) => println!("W: uart_read 0 bytes"),
            Ok(bytes_read) => {
                let data = &buf[..bytes_read];
                //trace!("UART IN: {}", bytes_read);
                //(*uart_pipe_writer).write(data).await;
                tcp_pipe_writer.write(data).await;
            }
        }
    }
}

async fn uart_write(
    uart_tx: &mut UartTx<'_, Async>,
    tcp_pipe_reader: &mut MyPipeReader<'_>,
) -> ! {
    let mut buf = [0; 64];
    loop {
        let bytes_read = tcp_pipe_reader.read(&mut buf).await;
        let data = &buf[0..bytes_read];
        match uart_tx.write_async(&data).await {
            Err(e) => println!("E: uart_write {:?}", e),
            Ok(0) => println!("E: uart_write 0 bytes"),
            Ok(written) => {
                //println!("uartTx written {}", written);
            }
        }
    }
}


async fn tcp_write(
    tcp_tx: &mut TcpWriter<'_>,
    uart_pipe_reader: &mut MyPipeReader<'_>,
) -> Result<(), embassy_net::tcp::Error> {
    let mut buf = [0; 64];
    loop {
        let bytes_read = uart_pipe_reader.read(&mut buf).await;
        let mut data = &buf[..bytes_read];
        //trace!("TCP OUT: {}", bytes_read);

        while !data.is_empty() {
            match tcp_tx.write(data).await {
                Ok(0) => panic!("write() returned Ok(0)"),
                Ok(bytes_written) => data = &data[bytes_written..],
                Err(e) => {
                    println!("E: tcp_tx.write {:?}", e);
                    return Err(e);
                }
            }
        }
    }
}

async fn tcp_read(
    tcp_rx: &mut TcpReader<'_>,
    uart_pipe_writer: &mut MyPipeWriter<'_>,
) {
    let mut buf = [0; 64];
    loop {
        match tcp_rx.read(&mut buf).await {
            Err(e) => println!("E: tcp_read {:?}", e),
            Ok(0) => break,
            Ok(bytes_read) => {
                let data = &buf[..bytes_read];
                //trace!("TCP IN: {}", bytes_read);
                //(*uart_pipe_writer).write(data).await;
                uart_pipe_writer.write(data).await;
            }
        }
    }
    println!("EXIT tcp_read");
}
*/
