// display.rs

use embassy_stm32::{
    Peri,
    gpio::{AnyPin, Level, Output, Speed},
};
use embassy_time::{Duration, Timer};

pub struct Max7219<'d> {
    din: Output<'d>,
    cs: Output<'d>,
    clk: Output<'d>,
}

impl<'d> Max7219<'d> {
    pub fn new(
        din: Peri<'static, AnyPin>,
        cs: Peri<'static, AnyPin>,
        clk: Peri<'static, AnyPin>,
    ) -> Self {
        let din = Output::new(din, Level::Low, Speed::Low);
        let cs = Output::new(cs, Level::High, Speed::Low);
        let clk = Output::new(clk, Level::Low, Speed::Low);

        let mut display = Self { din, cs, clk };
        display.init();
        display
    }

    fn send_byte(&mut self, data: u8) {
        for i in (0..8).rev() {
            self.clk.set_low();

            if (data & (1 << i)) != 0 {
                self.din.set_high();
            } else {
                self.din.set_low();
            }
            cortex_m::asm::delay(100);

            self.clk.set_high();
            cortex_m::asm::delay(100);
        }
    }

    pub fn send_command(&mut self, address: u8, data: u8) {
        self.cs.set_low();
        self.send_byte(address);
        self.send_byte(data);
        self.cs.set_high();
        self.clk.set_low();
    }

    pub fn init(&mut self) {
        self.send_command(0x0F, 0x00);
        self.send_command(0x0C, 0x01); // 进入正常工作模式
        self.send_command(0x0B, 0x07); // 扫描范围：扫描所有 8 行
        self.send_command(0x09, 0x00); // 不解码
        self.send_command(0x0A, 0x00); // 亮度

        self.clear(); // 清空屏幕
    }

    pub fn clear(&mut self) {
        for i in 1..=8 {
            self.send_command(i, 0x00); // 把每一行都设为0
        }
    }

    // 刷新屏幕
    pub fn flush(&mut self, buffer: &[u8; 8]) {
        for (i, byte) in buffer.iter().enumerate() {
            self.send_command((i as u8) + 1, *byte);
        }
    }
}
