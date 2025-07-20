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
use embedded_io_async::Write;
use portable_atomic::AtomicUsize;

use embassy_executor::Spawner;
use embassy_futures::select::Either;
use embassy_net::tcp::{TcpSocket, TcpWriter};
use embassy_net::{DhcpConfig, IpListenEndpoint, Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Parity, RxConfig, RxError, StopBits, SwFlowControl, Uart, UartRx};
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

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

struct UartStats {
    bytes_read : AtomicUsize,
    bytes_written : AtomicUsize,
    err_read : AtomicUsize,
    err_write : AtomicUsize,
    written_not_equal_read : AtomicUsize,
    err_rx_fifo_overflowed : AtomicUsize
}

struct TcpStats {
    bytes_read : AtomicUsize,
    bytes_written : AtomicUsize,
    connections : AtomicUsize,
    err_read : AtomicUsize,
    err_write : AtomicUsize,
    written_not_equal_read : AtomicUsize
}

struct Stats {
    uart : UartStats,
    tcp : TcpStats
}

fn load_atom(x : &portable_atomic::AtomicUsize) -> usize {
    x.load(core::sync::atomic::Ordering::Relaxed)
}

fn print_stats(s : &Stats) {
    println!("TCP
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
    load_atom(&s.tcp.bytes_read)
    ,load_atom(&s.tcp.bytes_written)
    ,load_atom(&s.tcp.connections)
    ,load_atom(&s.tcp.err_read)
    ,load_atom(&s.tcp.err_write)
    ,load_atom(&s.tcp.written_not_equal_read)

    ,load_atom(&s.uart.bytes_read)
    ,load_atom(&s.uart.bytes_written)
    ,load_atom(&s.uart.err_read)
    ,load_atom(&s.uart.err_write)
    ,load_atom(&s.uart.written_not_equal_read)
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
        singleton!(StackResources::<3>::new()),
        net_seed,
    );

    let mut uart = {
        let config = esp_hal::uart::Config::default()
            .with_baudrate(115200)
            .with_data_bits(esp_hal::uart::DataBits::_8)
            .with_parity(Parity::None)
            .with_stop_bits(StopBits::_1)
            .with_rx(
                esp_hal::uart::RxConfig::default().with_fifo_full_threshold(64),
            );

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

    let mut socket_rx_buffer = [0; 1024];
    let mut socket_tx_buffer = [0; 1024];

    let mut stats = Stats {
        uart : UartStats { bytes_read: 0.into(), bytes_written: 0.into(), err_read: 0.into(), err_write: 0.into(), written_not_equal_read: 0.into(), err_rx_fifo_overflowed : 0.into() }
        , tcp: TcpStats { bytes_read: 0.into(), bytes_written: 0.into(), err_read: 0.into(), err_write: 0.into(), connections: 0.into(), written_not_equal_read : 0.into() } };

    loop {
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

        let mut sock = TcpSocket::new(stack, &mut socket_rx_buffer, &mut socket_tx_buffer);
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

            handle_conexión(&mut sock,&mut uart, &mut stats).await;
        }
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}

fn inc_atom(x : &portable_atomic::AtomicUsize) {
    x.add(1, core::sync::atomic::Ordering::Relaxed);
}

fn add_atom(x : &portable_atomic::AtomicUsize, val : usize) {
    x.add(val, core::sync::atomic::Ordering::Relaxed);
}

async fn handle_conexión(sock : &mut TcpSocket<'_>, mut uart : &mut Uart<'_, Async>, stats : &mut Stats) {
    let mut sock_buf = [0; 512];
    let mut uart_buf = [0; 512];

    inc_atom(&stats.tcp.connections);
    let _ = uart.flush();

    loop {
        let reads_in_flight = embassy_futures::select::select(
            sock.read(&mut sock_buf),
            embedded_io_async::Read::read(&mut uart, &mut uart_buf)
            //uart.read_async(&mut uart_buf),
        ).await;

        match reads_in_flight {
            Either::First(sock_result) => match sock_result {
                Err(e) => {
                    inc_atom(&stats.tcp.err_read);
                    //println!("E: sock.read(): {:?}", e);
                    break;
                }
                Ok(0) => {
                    println!("I: connection closed by remote");
                    print_stats(&stats);
                    break;
                }
                Ok(bytes_read) => {
                    add_atom(&stats.tcp.bytes_read, bytes_read);
                    match embedded_io_async::Write::write(&mut uart, &sock_buf[0..bytes_read]).await {
                        Err(e) => {
                            //println!("E: uart write IoError {:?}", e);
                            inc_atom(&stats.uart.err_write);
                        },
                        Ok(written) => {
                            if written != bytes_read {
                                //println!("W: uart written != socket bytes_read:  {} != {}",written, bytes_read);
                                inc_atom(&stats.uart.written_not_equal_read);
                            }
                            else {
                                //println!("I: sock->uart: {} bytes", bytes_read);
                                add_atom(&stats.uart.bytes_written, written);
                            }
                        }
                    }
                }
            },
            Either::Second(uart_result) => match uart_result {
                Err( esp_hal::uart::IoError::Rx(rx_err) ) => {
                    match rx_err {
                        RxError::FifoOverflowed => {
                            inc_atom(&stats.uart.err_rx_fifo_overflowed);
                        },
                        _ => {
                            inc_atom(&stats.uart.err_read);
                        }
                    }
                },
                Err(io_err) => {
                    println!("W: uart read IoError: {:?}", io_err);
                    inc_atom(&stats.uart.err_read);
                }
                Ok(0) => {
                    println!("W: uart.read() returned 0 bytes read");
                },
                Ok(bytes_read) => {
                    add_atom(&stats.uart.bytes_read, bytes_read);
                    match sock.write(&uart_buf[0..bytes_read]).await {
                        Err(e) => {
                            //println!("E: socket.write TxError {:?}", e);
                            inc_atom(&stats.tcp.err_write);
                        },
                        Ok(written) => {
                            if written != bytes_read {
                                //println!("W: socket written != uart bytes_read:  {} != {}",written, bytes_read);
                                inc_atom(&stats.tcp.written_not_equal_read);
                            }
                            else {
                                //println!("I: uart->sock: {} bytes, ", bytes_read);
                                add_atom(&stats.tcp.bytes_written, written);
                            }
                        }
                    }
                }
            },
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

        const SSID: &str = "BabyStube";
        const PASSWORD: &str = "PaulaNadja1977";

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
