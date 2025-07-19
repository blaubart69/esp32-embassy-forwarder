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

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{DhcpConfig, IpListenEndpoint, Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_wifi::wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState};
use esp_wifi::EspWifiController;
use static_cell::StaticCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    esp_println::logger::init_logger(log::LevelFilter::Info);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);

    let timer1 = TimerGroup::new(peripherals.TIMG0);

    static WIFI_CONTROLLER_CELL : StaticCell<EspWifiController> = StaticCell::new();
    let esp_wifi_ctrl = WIFI_CONTROLLER_CELL.init(
        esp_wifi::init(
            timer1.timer0,
            rng.clone(),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );
    /*
    let esp_wifi_ctrl = singleton!(
        esp_wifi::init(
            timer1.timer0,
            rng.clone(),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );*/


    let (controller, interfaces) = esp_wifi::wifi::new(esp_wifi_ctrl, peripherals.WIFI).unwrap();
    let wifi_interface = interfaces.sta;

    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    //let tls_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    let net_config = embassy_net::Config::dhcpv4(DhcpConfig::default());
    
    // Init network stack
    let (stack , runner) = embassy_net::new(
        wifi_interface,
        net_config,
        singleton!(StackResources::<3>::new()),
        net_seed,
    );

    spawner.spawn(connection_task(controller)).ok();
    spawner.spawn(net_task(runner)).ok();
    
    
    loop {
        loop {
            if ! stack.is_config_up() {
                println!("Waiting for network stack to get configured...");
                Timer::after(Duration::from_millis(1000)).await
            }
            else if let Some(config) = stack.config_v4() {
                println!("Got IP: {}, gateway: {:?}, DNS: {:?}", config.address, config.gateway, config.dns_servers);
                break;
            }
            else {
                println!("X: error getting network config after is_config_up(). bad.");
            }
        }
        
        let mut rx_buffer = [0; 128];
        let mut tx_buffer = [0; 128];

        let mut sock = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        if let Err(err) = sock.accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            }
        ).await {
            println!("E: accept: {:?}", err);
        }
        else {
            println!("I: connection from {:?}", sock.remote_endpoint());
            loop {
                let mut buf = [0; 128];
                match sock.read(&mut buf).await {
                    Err(e) => {
                        println!("E: {:?}",e);
                        break;
                    },
                    Ok(0) => {
                        println!("I: connection closed by remote");
                        break;
                    }
                    Ok(bytes_read) => {
                        println!("I: read {} bytes", bytes_read);
                    }
                }
            }
        }
        
    }



    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}
/*
#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}
*/
#[embassy_executor::task]
async fn net_task(mut runner : Runner<'static, WifiDevice<'static>> ) {
    runner.run().await
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

        const SSID: &str = "hacke";
        const PASSWORD: &str = "hacke1234";

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




/* 
    * don't know why this isn't working

let listen_address = SocketAddrV4::new(Ipv4Addr::new(0, 0, 0, 0), 8080);
let listen_ep = IpListenEndpoint::from(listen_address);
println!("I: listening on: {}", listen_ep);
let x = sock.accept(listen_ep).await;
*/
