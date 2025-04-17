use bstr::ByteSlice;
use esp32_nimble::{
    utilities::BleUuid, uuid128, BLEAdvertisementData, BLEDevice, NimbleProperties,
};
use esp_idf_svc::hal::{
    delay, gpio, ledc, peripheral::Peripheral, peripherals::Peripherals, units::Hertz,
};
use log::{info, warn};
use std::sync::mpsc::{sync_channel, SyncSender};

fn init_ble(sender: SyncSender<Vec<u8>>) -> Result<(), esp32_nimble::BLEError> {
    // BLE GATT UUIDs
    static BLE_SERVICE_UUID: BleUuid = uuid128!("87cde903-dd98-4bda-b3ac-ee6e1718f373");
    static BLE_CMD_UUID: BleUuid = uuid128!("047c2b6b-97b5-4b0c-adba-bbea3f7fb2e2");

    let ble_device = BLEDevice::take();
    let ble_server = ble_device.get_server(); // not need start manually
    let ble_advertising = ble_device.get_advertising();

    // BLE Server Configuration
    ble_server.advertise_on_disconnect(true);
    ble_server.on_connect(|server, desc| {
        info!("Connected to device: {}", desc.address());
        let _ = server.update_conn_params(desc.conn_handle(), 24, 48, 0, 60);
    });
    ble_server.on_disconnect(|desc, _| info!("Disconnected from device: {}", desc.address()));

    // BLE Service Configuration
    let ble_service = ble_server.create_service(BLE_SERVICE_UUID);
    let writable_characteristic_cmd = ble_service.lock().create_characteristic(
        BLE_CMD_UUID,
        NimbleProperties::READ | NimbleProperties::WRITE | NimbleProperties::NOTIFY,
    );
    writable_characteristic_cmd.lock().on_write(move |cmd| {
        let data = cmd.recv_data();
        info!("Received command: {}", data.as_bstr());
        if let Err(_) = sender.try_send(data.to_vec()) {
            warn!("is busy");
        }
    });

    // BLE Start Advertising
    ble_advertising.lock().set_data(
        BLEAdvertisementData::new()
            .name("BLE Lock")
            .add_service_uuid(BLE_SERVICE_UUID),
    )?;
    ble_advertising.lock().min_interval(1280); // 800ms
    ble_advertising.lock().max_interval(1600); // 1000ms
    ble_advertising.lock().scan_response(false);
    ble_advertising.lock().start()
}

struct SG90<'a> {
    ledc: ledc::LedcDriver<'a>,
    max_duty: u32,
}

impl<'a> SG90<'a> {
    fn new<C, T>(
        channel: impl Peripheral<P = C> + 'a,
        timer: impl Peripheral<P = T> + 'a,
        pin: impl Peripheral<P = impl gpio::OutputPin> + 'a,
    ) -> Result<Self, esp_idf_svc::sys::EspError>
    where
        C: ledc::LedcChannel<SpeedMode = <T as ledc::LedcTimer>::SpeedMode>,
        T: ledc::LedcTimer + 'a,
    {
        let ledc = ledc::LedcDriver::new(
            channel,
            ledc::LedcTimerDriver::new(
                timer,
                &ledc::config::TimerConfig::new()
                    .frequency(Hertz(50))
                    .resolution(ledc::Resolution::Bits14),
            )?,
            pin,
        )?;
        let max_duty = ledc.get_max_duty();
        Ok(Self { ledc, max_duty })
    }

    fn set_degree(&mut self, degree: u8) -> Result<(), esp_idf_svc::sys::EspError> {
        let duty = (degree as f32 / 1800.0) + 0.025;
        let duty = duty * self.max_duty as f32;
        self.ledc.set_duty(duty as u32)?;
        Ok(())
    }
}

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let (sender, receiver) = sync_channel(1);
    init_ble(sender)?;

    let peripherals = Peripherals::take()?;
    let mut st90 = SG90::new(
        peripherals.ledc.channel0,
        peripherals.ledc.timer0,
        peripherals.pins.gpio9,
    )?;

    st90.set_degree(0)?;
    delay::FreeRtos::delay_ms(1000);

    loop {
        let data = receiver.recv()?;
        if let Ok(degree) = data.to_str_lossy().parse::<u8>() {
            if degree > 180 {
                warn!("Invalid degree: {}", degree);
                continue;
            }
            info!("Set degree: {}", degree);

            st90.set_degree(degree)?;
            delay::FreeRtos::delay_ms(1000);

            st90.set_degree(0)?;
            delay::FreeRtos::delay_ms(1000);
        } else {
            warn!("Invalid command: {}", data.as_bstr());
        }
    }
}
