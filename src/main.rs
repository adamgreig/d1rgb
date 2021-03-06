#![no_std]
#![no_main]

use panic_halt as _;

mod de;

// Not sure if alignment is required or not, but force the issue.
#[repr(C, align(256))]
struct FrameBuffer([u8; 3*480*272]);
static mut FB: FrameBuffer = FrameBuffer(*include_bytes!("ferris.data"));

struct Uart(d1_pac::UART0);
static mut PRINTER: Option<Uart> = None;
impl core::fmt::Write for Uart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.as_bytes() {
            self.0.thr().write(|w| unsafe { w.thr().bits(*byte) });
            while self.0.usr.read().tfnf().bit_is_clear() {}
        }
        Ok(())
    }
}
pub fn _print(args: core::fmt::Arguments) {
    use core::fmt::Write;
    unsafe {
        PRINTER.as_mut().unwrap().write_fmt(args).ok();
    }
}
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        $crate::_print(core::format_args!($($arg)*));
    }
}
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        $crate::_print(core::format_args!($($arg)*));
        $crate::print!("\r\n");
    }
}

#[riscv_rt::entry]
fn main() -> ! {
    let p = d1_pac::Peripherals::take().unwrap();

    // Enable UART0 clock.
    let ccu = &p.CCU;
    ccu.uart_bgr.write(|w| w.uart0_gating().pass().uart0_rst().deassert());

    // Enable LEDC clock.
    ccu.ledc_clk.write(|w| w.clk_gating().on().clk_src_sel().hosc());
    ccu.ledc_bgr.write(|w| w.gating().pass().rst().deassert());

    // Enable PLL_VIDEO0. Fin=24M N=27 M=2 Fout(4X)=324M
    // Aiming to exceed the 297MHz requirement for DE while being
    // a multiply of 9MHz to generate 9M pixel clock.
    // Though maybe DE needs to be exactly 297, or less than 297...
    ccu.pll_video0_ctrl.write(|w| unsafe {
        w.pll_output_div2().clear_bit()
         .pll_input_div2().set_bit()
         .pll_n().bits(27 - 1)
         .pll_output_gate().clear_bit()
         .lock_enable().clear_bit()
         .pll_ldo_en().set_bit()
         .pll_en().set_bit()
    });
    ccu.pll_video0_ctrl.modify(|_, w| w.lock_enable().set_bit());
    while ccu.pll_video0_ctrl.read().lock().is_unlocked() {}
    unsafe { riscv::asm::delay(20_000) };
    ccu.pll_video0_ctrl.modify(|_, w| w.pll_output_gate().set_bit());

    // Set TCON_LCD to PLL_VIDEO0(4X) and enable.
    ccu.tconlcd_clk.write(|w| unsafe {
        w.clk_gating().on()
         .clk_src_sel().pll_video0_4x()
         .factor_n().n1()
         .factor_m().bits(0)
    });
    ccu.tconlcd_bgr.write(|w| w.rst().deassert().gating().pass());

    // Enable Display Engine clock with VIDEO0(4X)=324MHz and bring out of reset.
    // FACTOR_M=0 for M=1 for DE_CLK=CLK_SRC/1.
    ccu.de_clk.write(|w| w.clk_gating().on().clk_src_sel().pll_video0_4x());
    ccu.de_bgr.write(|w| w.rst().deassert().gating().pass());

    // Set PC0 to LEDC_DO, PC1 LED to output.
    let gpio = &p.GPIO;
    gpio.pc_cfg0.write(|w| w.pc0_select().ledc_do().pc1_select().output());

    // Set PB8 and PB9 to function 6, UART0, internal pullup.
    gpio.pb_cfg1.write(|w| w.pb8_select().uart0_tx().pb9_select().uart0_rx());
    gpio.pb_pull0.write(|w| w.pc8_pull().pull_up().pc9_pull().pull_up());

    // Set PD0-21 to LCD functions, and PD22 (LCD backlight PWM) to output.
    gpio.pd_cfg0.write(|w|
        w.pd0_select().lcd0_d2()
         .pd1_select().lcd0_d3()
         .pd2_select().lcd0_d4()
         .pd3_select().lcd0_d5()
         .pd4_select().lcd0_d6()
         .pd5_select().lcd0_d7()
         .pd6_select().lcd0_d10()
         .pd7_select().lcd0_d11());
    gpio.pd_cfg1.write(|w|
        w.pd8_select().lcd0_d12()
         .pd9_select().lcd0_d13()
         .pd10_select().lcd0_d14()
         .pd11_select().lcd0_d15()
         .pd12_select().lcd0_d18()
         .pd13_select().lcd0_d19()
         .pd14_select().lcd0_d20()
         .pd15_select().lcd0_d21());
    gpio.pd_cfg2.write(|w|
        w.pd16_select().lcd0_d22()
         .pd17_select().lcd0_d23()
         .pd18_select().lcd0_clk()
         .pd19_select().lcd0_de()
         .pd20_select().lcd0_hsync()
         .pd21_select().lcd0_vsync()
         .pd22_select().output());

    // Configure UART0 for 115200 8n1.
    // By default APB1 is 24MHz, use divisor 13 for 115200.
    let uart0 = p.UART0;
    uart0.mcr.write(|w| unsafe { w.bits(0) });
    uart0.fcr().write(|w| w.fifoe().set_bit());
    uart0.halt.write(|w| w.halt_tx().enabled());
    uart0.lcr.write(|w| w.dlab().divisor_latch());
    uart0.dll().write(|w| unsafe { w.dll().bits(13)});
    uart0.dlh().write(|w| unsafe { w.dlh().bits(0) });
    uart0.lcr.write(|w| w.dlab().rx_buffer().dls().eight());
    uart0.halt.write(|w| w.halt_tx().disabled());
    unsafe { PRINTER = Some(Uart(uart0)) };

    // Turn on LCD backlight (PWM can come later).
    gpio.pd_dat.write(|w| unsafe { w.bits(1<<22) });

    // Configure TCON for RGB operation:
    let lcd0 = &p.TCON_LCD0;
    // TCON_EN / LCD_EN
    lcd0.lcd_gctl_reg.write(|w| unsafe { w.bits(1 << 31) });
    // Configure TCON_LCD in RGB666 mode for our 480x272 LCD.
    lcd0.lcd_ctl_reg.write(|w| unsafe { w.bits(
        // LCD_CTL_REG.LCD_EN=0 to disable
        (0 << 31)
        // LCD_CTL_REG.LCD_IF=0 for HV(Sync+DE)
        | (0 << 24)
        // Delay is VT-VD=292-272=20 (ie BP+FP?) (max 30)
        | (20 << 4)
        // LCD_CTL_REG.LCD_SRC_SEL=000 for Display Engine source.
        // Try 0b001 for color check or 0b111 for grid check.
        | (0b000 << 0)
    )});
    lcd0.lcd_hv_if_reg.write(|w| unsafe { w.bits(
        // LCD_HV_IF_REG.HV_MODE=0 for 24bit/cycle parallel mode.
        0 << 28
    )});
    lcd0.lcd_dclk_reg.write(|w| unsafe { w.bits(
        // LCD_DCLK_REG.LCD_DCLK_EN=0001 for dclk_en=1, others=0
        (0b0001 << 28)
        // Linux just sets bit 31, i.e. 0b1000, which is "reserved" in D1 docs.
        //(1 << 31)
        // LCD_DCLK_REG.LCD_DCLK_DIV=36 for /36 to obtain 9MHz DCLK from 324MHz.
        | (36 << 0)
    )});
    lcd0.lcd_basic0_reg.write(|w| unsafe { w.bits(
        // LCD_BASIC0_REG.WIDTH_X=479 for 480px wide panel
        (479 << 16)
        // LCD_BASIC0_REG.HEIGHT_Y=271 for 272px tall panel
        | (271 << 0)
    )});
    lcd0.lcd_basic1_reg.write(|w| unsafe { w.bits(
        // LCD_BASIC1_REG.HT=530 for 531 horizontal clocks total
        (530 << 16)
        // LCD_BASIC1_REG.HBP=42 for 43 Thbp
        | (42 << 0)
    )});
    lcd0.lcd_basic2_reg.write(|w| unsafe { w.bits(
        // LCD_BASIC2_REG.VT=584 for 292 vertical rows total
        (584 << 16)
        // LCD_BASIC2_REG.VBP=11 for 12 Tvbp
        | (11 << 0)
    )});
    lcd0.lcd_basic3_reg.write(|w| unsafe { w.bits(
        // LCD_BASIC3_REG.HSPW=3 for 4 DCLK wide HSYNC pulse
        (3 << 16)
        // LCD_BASIC3_REG.VSPW=3 for 4 row long VSYNC pulse
        | (3 << 0)
    )});
    lcd0.lcd_io_tri_reg.write(|w| unsafe { w.bits(
        // Enable IO0-3, D2..7, D10..15, D18..23 inclusive.
        0x0003_0303
    )});
    // LCD_EN
    lcd0.lcd_ctl_reg.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 31)) });

    // Initialise display engine
    unsafe {
        println!("Framebuffer addr is {:08X}", FB.0.as_ptr() as u32);
        println!("First bytes are: {:02X} {:02X} {:02X}",
                 FB.0[0], FB.0[1], FB.0[2]);
    }
    unsafe { de::init(&FB.0) };

    // Configure LEDC
    let ledc = &p.LEDC;
    ledc.led_t01_timing_ctrl.write(|w| unsafe {
        w.t1h_time().bits(0x14)
         .t1l_time().bits(0x06)
         .t0h_time().bits(0x07)
         .t0l_time().bits(0x13)
    });
    ledc.ledc_data_finish_cnt.write(|w| unsafe {
        w.led_wait_data_time().bits(0x1D4C)
    });
    ledc.led_reset_timing_ctrl.write(|w| unsafe {
        w.tr_time().bits(0x1D4C)
         .led_num().bits(0)
    });
    ledc.ledc_wait_time0_ctrl.write(|w| unsafe {
        w.wait_tim0_en().set_bit()
         .total_wait_time0().bits(0xFF)
    });
    ledc.ledc_dma_ctrl.write(|w| w.ledc_dma_en().clear_bit());
    ledc.ledc_int_ctrl.write(|w| w.global_int_en().clear_bit());
    ledc.ledc_ctrl.write(|w| unsafe {
        w.total_data_length().bits(1)
         .led_rgb_mode().grb()
         .led_msb_top().msb()
         .led_msb_g().msb()
         .led_msb_r().msb()
         .led_msb_b().msb()
    });
    set_led(ledc, 0x0000_00FF);

    // Blink LED
    loop { unsafe {
        gpio.pc_dat.write(|w| w.bits(2));
        set_led(ledc, 0x0000FFFF);
        riscv::asm::delay(100_000_000);

        gpio.pc_dat.write(|w| w.bits(0));
        set_led(ledc, 0x00FF00FF);
        riscv::asm::delay(100_000_000);
    }}
}

fn set_led(ledc: &d1_pac::LEDC, rgb: u32) {
    ledc.ledc_ctrl.modify(|_, w| w.ledc_soft_reset().set_bit());
    ledc.ledc_data.write(|w| unsafe { w.bits(rgb) });
    ledc.ledc_ctrl.modify(|_, w| w.ledc_en().enable().reset_led_en().set_bit());
    while ledc.ledc_ctrl.read().reset_led_en().bit_is_set() {}
}
