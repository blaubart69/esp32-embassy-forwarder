#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(type_alias_impl_trait)]

//extern crate alloc;

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Parity, StopBits};
use esp_println::println;
use esp_wifi::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState,
};
use esp_wifi::EspWifiController;
use static_cell::StaticCell;

use espfwd::forward;

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

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    esp_println::logger::init_logger(log::LevelFilter::Info);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    //let mut led = Output::new(peripherals.GPIO0, Level::High, OutputConfig::default());

    let _ = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let _ = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    // let _ = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default()); --> broken pipe

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

    let uart = {
        let config = esp_hal::uart::Config::default()
            .with_baudrate(460800)
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

    let mut stats = espfwd::stats::Stats::new();
    forward::accept_connection(stack, uart, &mut stats).await;
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
        let ssid: &'static str = env!("ESP32_SSID");
        let password: &'static str = env!("ESP32_PASS");

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: ssid.try_into().unwrap(),
                password: password.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start_async().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect to SSID {} ...", ssid);

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
