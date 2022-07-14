#![no_std]
#![no_main]

use panic_halt as _;

#[riscv_rt::entry]
fn main() -> ! {
    let p = d1_pac::Peripherals::take().unwrap();
    let gpio = &p.GPIO;
    gpio.pc_cfg0.write(|w| w.pc1_select().output());
    loop { unsafe {
        gpio.pc_dat.write(|w| w.bits(2));
        riscv::asm::delay(1_000_000_000);
        gpio.pc_dat.write(|w| w.bits(0));
        riscv::asm::delay(1_000_000_000);
    }}
}
