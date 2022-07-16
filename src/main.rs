#![no_std]
#![no_main]

use panic_halt as _;

fn print(uart: &d1_pac::UART0, message: &[u8]) {
    for byte in message.iter() {
        uart.thr().write(|w| unsafe { w.thr().bits(*byte) });
        while uart.usr.read().tfnf().bit_is_clear() {}
    }
}

#[riscv_rt::entry]
fn main() -> ! {
    let p = d1_pac::Peripherals::take().unwrap();
    let ccu = &p.CCU;
    let gpio = &p.GPIO;
    let uart0 = &p.UART0;

    // Enable UART0 clock.
    ccu.uart_bgr.write(|w| w.uart0_gating().pass().uart0_rst().deassert());

    // Set LED to output.
    gpio.pc_cfg0.write(|w| w.pc1_select().output());

    // Set PB8 and PB9 to function 6, UART0, internal pullup.
    gpio.pb_cfg1.write(|w| w.pb8_select().uart0_tx().pb9_select().uart0_rx());
    gpio.pb_pull0.write(|w| w.pc8_pull().pull_up().pc9_pull().pull_up());

    // Configure UART0 for 115200 8n1.
    // By default APB1 is 24MHz, use divisor 13 for 115200.
    uart0.mcr.write(|w| unsafe { w.bits(0) });
    uart0.fcr().write(|w| w.fifoe().set_bit());
    uart0.halt.write(|w| w.halt_tx().enabled());
    uart0.lcr.write(|w| w.dlab().divisor_latch());
    uart0.dll().write(|w| unsafe { w.dll().bits(13)});
    uart0.dlh().write(|w| unsafe { w.dlh().bits(0) });
    uart0.lcr.write(|w| w.dlab().rx_buffer().dls().eight());
    uart0.halt.write(|w| w.halt_tx().disabled());

    // Blink LED
    loop { unsafe {
        gpio.pc_dat.write(|w| w.bits(2));
        riscv::asm::delay(100_000_000);
        gpio.pc_dat.write(|w| w.bits(0));
        riscv::asm::delay(100_000_000);
        print(&uart0, b"Hello, world!\r\n");
    }}
}
