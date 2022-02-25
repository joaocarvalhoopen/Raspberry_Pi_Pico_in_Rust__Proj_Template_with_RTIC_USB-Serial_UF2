#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

mod fmt;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

    use crate::fmt::Wrapper;
    use core::fmt::Write;
    use embedded_hal::digital::v2::OutputPin;
    use embedded_time::duration::Extensions;

    use rp_pico::hal;
    use rp_pico::pac;
    use rp_pico::XOSC_CRYSTAL_FREQ;

    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};
    // USB Communications Class Device support
    use usbd_serial::SerialPort;

    // Blinck time 5 seconds
    const SCAN_TIME_US: u32 = 5000000; //  200000; // 5000000;  // 1000000; // 200000;

    // IMPORTANT: The USB-Serial with RTIC github project example that I'm following.
    //            I tried to use the Pico board examples of USB-Serial (without interrupts
    //            and with interrupts with success, but when using with RTIC I could not make
    //            it work when merged with the RTIC example.) So I asked some questions
    //            in the in Matrix chat and received links to examples of there github
    //            project where it was working, then a used and adapted some parts there
    //            in this project template.
    //            This were the kind folks that helped me in the Matrix chat, the 3 projects
    //            that they suggest me to study are good examples of programs made with RTIC
    //            and USB and should be studied.
    //
    // Paul Daniel Faria
    // https://github.com/Nashenas88/dactyl-manuform-kb2040-rs/blob/main/src/main.rs#L80
    //
    // see also:
    // korken89
    // https://github.com/korken89/pico-probe/tree/master/src
    //
    // see also:
    // Mathias
    // https://github.com/mgottschlag/rp2040-usb-sound-card/blob/b8078b57361c1b08755e5ab5f9992c56457ec18b/src/main.rs#L188
    //
    //
    // Global Static variable, has to be written inside unsafe blocks.
    // A reference can be obtained with as_ref() method.

    pub struct Counter {
        counter: u32,
        enable: bool,
    }

    impl Counter {
        fn new() -> Self {
            Counter {
                counter: 0_u32,
                enable: true,
            }
        }

        fn get(&self) -> u32 {
            self.counter
        }

        fn reset(&mut self) {
            self.counter = 0_u32;
        }

        fn increment(&mut self) {
            self.counter += 1_u32;
        }

        fn enable(&mut self, state: bool) {
            self.enable = state;
        }
    }

    #[shared]
    struct Shared {
        timer: hal::Timer,
        alarm: hal::timer::Alarm0,
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
        led_blink_enable: bool,

        serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_dev: usb_device::device::UsbDevice<'static, hal::usb::UsbBus>,

        counter: Counter,
    }

    #[local]
    struct Local {}

    #[init(local = [usb_bus: Option<usb_device::bus::UsbBusAllocator<hal::usb::UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        //*******
        // Initialization of the system clock.

        let mut resets = c.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(c.device.WATCHDOG);

        // Configure the clocks - The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        //*******
        // Initialization of the USB and Serial and USB Device ID.

        // USB
        //
        // Set up the USB driver
        // The bus that is used to manage the device and class below.
        let usb_bus: &'static _ =
            c.local
                .usb_bus
                .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
                    c.device.USBCTRL_REGS,
                    c.device.USBCTRL_DPRAM,
                    clocks.usb_clock,
                    true,
                    &mut resets,
                )));

        // Set up the USB Communications Class Device driver.
        let serial = SerialPort::new(usb_bus);

        // Create a USB device with a fake VID and PID
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        //*******
        // Initialization of the LED GPIO and the timer.

        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt(&mut timer);

        // Enable led_blink.
        let led_blink_enable = true;

        // Reset the counter
        let counter = Counter::new();

        //********
        // Return the Shared variables struct, the Local variables struct and the XPTO Monitonics
        //    (Note: Read again the RTIC book in the section of Monotonics timers)
        (
            Shared {
                timer,
                alarm,
                led,
                led_blink_enable,

                serial,
                usb_dev,

                counter,
            },
            Local {},
            init::Monotonics(),
        )
    }

    /// Task that blinks the rp-pico onboard LED and that send a message "LED ON!" and "LED OFF!" do USB-Serial.
    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [timer, alarm, led, led_blink_enable,  serial, counter],
        local = [tog: bool = true],
    )]
    fn timer_irq(mut cx: timer_irq::Context) {
        let mut buf = [0u8; 64];

        let led = cx.shared.led;
        let led_blink_enable = cx.shared.led_blink_enable;
        let counter = cx.shared.counter;

        let tog = cx.local.tog;

        // Blinks the LED ON / OFF.
        (led, led_blink_enable, counter).lock(|led_a, led_blink_enable_a, counter_a| {
            let led_state_str: &str;
            if *led_blink_enable_a {
                if *tog {
                    led_a.set_high().unwrap();
                    led_state_str = "ON ";
                } else {
                    led_a.set_low().unwrap();
                    led_state_str = "OFF";
                }
                let _ = writeln!(
                    Wrapper::new(&mut buf),
                    "LED {}!   counter = {}",
                    led_state_str,
                    counter_a.get()
                );
            }
            if counter_a.enable {
                counter_a.increment();
            }

            if *led_blink_enable_a {
                *tog = !*tog;
            }
        });

        // Clears the timer interrupt and Set's the new delta_time in the future.
        let mut timer = cx.shared.timer;
        let mut alarm = cx.shared.alarm;
        (alarm).lock(|a| {
            (timer).lock(|timer_a| {
                a.clear_interrupt(timer_a);
                let _ = a.schedule(SCAN_TIME_US.microseconds());
            });
        });

        // Write the message "blabla! 2" do USB-Serial.
        cx.shared.serial.lock(|s| {
            write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }, false);
        });

        /*
        // Write the message "blabla! 2" do USB-Serial.
        c.shared.serial.lock(|s| {
            let mut buf = [0u8; 64];
            let _ = writeln!(Wrapper::new(&mut buf), "blabla! {}", 2); /*"{:?}"*/
            write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }, false);
        });
        */
    }

    /// Usb interrupt handler. Runs every time the host requests new data.
    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [led, led_blink_enable, serial, usb_dev, counter])]
    fn usb_rx(cx: usb_rx::Context) {
        let led = cx.shared.led;
        let led_blink_enable = cx.shared.led_blink_enable;

        let usb_dev = cx.shared.usb_dev;
        let serial = cx.shared.serial;
        let counter = cx.shared.counter;

        (led, led_blink_enable, usb_dev, serial, counter).lock(
            |led_a, led_blink_enable_a, usb_dev_a, serial_a, counter_a| {
                // Check for new data
                if usb_dev_a.poll(&mut [serial_a]) {
                    let mut buf = [0u8; 64];
                    match serial_a.read(&mut buf) {
                        Err(_e) => {
                            // Do nothing
                            // let _ = serial_a.write(b"Error.");
                            // let _ = serial_a.flush();
                        }
                        Ok(0) => {
                            // Do nothing
                            let _ = serial_a.write(b"Didn't received data.");
                            let _ = serial_a.flush();
                        }
                        Ok(_count) => {
                            match_usb_serial_buf(
                                &buf,
                                led_a,
                                led_blink_enable_a,
                                serial_a,
                                counter_a,
                            );

                            /*
                            // Code to echo the characters in Upper Case.

                            // Convert to upper case
                            buf.iter_mut().take(count).for_each(|b| {
                                b.make_ascii_uppercase();
                            });
                            if count > 0 {
                                let _ = serial_a.write(b"Received data! ");
                                let _ = serial_a.flush();
                            }

                            // Send back to the host
                            let mut wr_ptr = &buf[..count];
                            while !wr_ptr.is_empty() {
                                match serial_a.write(wr_ptr) {
                                    Ok(len) => {
                                        wr_ptr = &wr_ptr[len..];
                                        // if wr_ptr.len() == 0 {
                                        // let _ = serial_a.write(b"");
                                        let _ = serial_a.flush();
                                        // }
                                    }
                                    // On error, just drop unwritten data.
                                    // One possible error is Err(WouldBlock), meaning the USB
                                    // write buffer is full.
                                    Err(_) => break,
                                };
                            }

                            */
                        }
                    }
                }
            },
        );
    }

    // Task with least priority that only runs when nothing else is running.
    #[idle(local = [x: u32 = 0])]
    fn idle(_cx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        // let _x: &'static mut u32 = cx.local.x;

        //hprintln!("idle").unwrap();

        loop {
            cortex_m::asm::nop();
        }
    }

    /* New Tasks */

    /// This function come from the github with USB-Serial example (see link above).
    ///
    /// Helper function to ensure all data is written across the serial interface.
    fn write_serial(serial: &mut SerialPort<'static, hal::usb::UsbBus>, buf: &str, block: bool) {
        let write_ptr = buf.as_bytes();

        // Because the buffer is of constant size and initialized to zero (0) we here
        // add a test to determine the size that's really occupied by the str that we
        // wan't to send. From index zero to first byte that is as the zero byte value.
        let mut index = 0;
        while index < write_ptr.len() && write_ptr[index] != 0 {
            index += 1;
        }
        let mut write_ptr = &write_ptr[0..index];

        while !write_ptr.is_empty() {
            match serial.write(write_ptr) {
                Ok(len) => write_ptr = &write_ptr[len..],
                // Meaning the USB write buffer is full
                Err(UsbError::WouldBlock) => {
                    if !block {
                        break;
                    }
                }
                // On error, just drop unwritten data.
                Err(_) => break,
            }
        }
        // let _ = serial.write("\n".as_bytes());
        let _ = serial.flush();
    }

    fn match_usb_serial_buf(
        buf: &[u8; 64],
        led: &mut hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
        led_blink_enable: &mut bool,
        serial: &mut SerialPort<'static, hal::usb::UsbBus>,
        counter: &mut Counter,
    ) {
        let _buf_len = buf.len();
        match buf[0] {
            // Print Menu
            b'M' | b'm' => {
                write_serial(serial, "M - Print Menu\n", false);
                print_menu(serial);
            }
            // 0 - Reset counter
            b'0' => {
                write_serial(serial, "M - Print Menu\n", false);
                counter.reset();
            }
            // 1 - Increment counter
            b'1' => {
                write_serial(serial, "1 - Increment counter\n", false);
                counter.increment();
            }
            // 2 - Start continues counter
            b'2' => {
                write_serial(serial, "2 - Start continues counter\n", false);
                counter.enable(true);
            }
            // 3 - Stop continues counter
            b'3' => {
                write_serial(serial, "3 - Stop continues counter\n", false);
                counter.enable(false);
            }
            // 4 - Get switch and LED state
            b'4' => {
                write_serial(serial, "4 - Get switch and LED state\n", false);

                // GPIO 25 onboard LED, we are going to read the bit 8 of the gpio_status register.
                //  OUTFROMPERI - output signal from selected peripheral, before register
                //                override is applied.
                // See pag 272 of the Pico Datasets:
                // https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#_gpio_functions

                let led_status_reg =
                    unsafe { (*pac::IO_BANK0::ptr()).gpio[25].gpio_status.read().bits() };

                // Reserved bit.
                // let sio_pin_value = unsafe { (*pac::SIO::ptr()).gpio_out.read().bits() };

                let (led_bool, led_status) = if ((led_status_reg & 1 << 8) >> 8) == 1_u32 {
                    (true, "ON")
                } else {
                    (false, "OFF")
                };

                let mut buf = [0u8; 64];
                let _ = writeln!(
                    Wrapper::new(&mut buf),
                    "LED Status {:b}, {}   LED {}",
                    led_status_reg,
                    led_bool,
                    led_status
                );
                write_serial(
                    serial,
                    unsafe { core::str::from_utf8_unchecked(&buf) },
                    false,
                );

                // unsafe { (*pac::TIMER::ptr()).timerawh.read().bits() };
            }
            // 5 - Set LED on
            b'5' => {
                write_serial(serial, "5 - Set LED on\n", false);
                *led_blink_enable = false;
                let _ = led.set_high();
            }
            // 6 - Set LED off
            b'6' => {
                write_serial(serial, "6 - Set LED off\n", false);
                *led_blink_enable = false;
                let _ = led.set_low();
            }
            // 7 - Set LED blink enable
            b'7' => {
                write_serial(serial, "7 - Set LED blink enable\n", false);
                *led_blink_enable = true;
            }
            b'8' => {
                write_serial(serial, "8 - Display data rate\n", false);

                let data_rate = serial.line_coding().data_rate();
                let mut buf = [0u8; 64];
                let _ = writeln!(Wrapper::new(&mut buf), "Data rate: {} bit/s", data_rate);
                write_serial(
                    serial,
                    unsafe { core::str::from_utf8_unchecked(&buf) },
                    false,
                );
            }
            _ => {
                write_serial(
                    serial,
                    unsafe { core::str::from_utf8_unchecked(buf) },
                    false,
                );
                write_serial(serial, "Invalid option!\n", false);
            }
        }
    }

    fn print_menu(serial: &mut SerialPort<'static, hal::usb::UsbBus>) {
        let mut _buf = [0u8; 273];

        // Create the Menu.
        let menu_str = "*****************
*  Menu:
*
*  M / m - Print menu
*    0   - Reset counter
*    1   - Increment counter
*    2   - Start continues counter
*    3   - Stop continues counter
*    4   - Get switch and LED state
*    5   - Set LED on
*    6   - Set LED off
*    7   - Set LED blink enable
*    8   - Display data rate
*****************
Enter option: ";

        write_serial(serial, menu_str, true);

        // Send out the data to USB-Serial.
        // let _ = serial.write(menu_str);

        // let _ = writeln!(Wrapper::new(&mut buf), &menu_str);
        // write_serial(serial, unsafe { core::str::from_utf8_unchecked(menu_str) });
    }
}
