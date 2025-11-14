#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::i2s::{Config, Format, I2S};
use embassy_stm32::time::Hertz;
use embassy_time::{Timer, Duration};
use core::f32::consts::PI;
use defmt::*;
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = {
        use embassy_stm32::rcc::*;

        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL192,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV4),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;

        // reference your chip's manual for proper clock settings; this config
        // is recommended for a 32 bit frame at 48 kHz sample rate
        config.rcc.plli2s = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL384,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV5),
        });
        config.enable_debug_during_sleep = true;

        config
    };

    let p = embassy_stm32::init(config);

    // 创建多个不同频率的wavetable
    let frequencies = [100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0];
    let mut wavetables = [[0u16; 1200]; 8];
    
    // 预生成所有频率的wavetable
    for (idx, &freq) in frequencies.iter().enumerate() {
        for (i, frame) in wavetables[idx].chunks_mut(2).enumerate() {
            let t = i as f32 / 48000.0; // 时间
            let sample = (2.0 * PI * freq * t).sin();
            let sample_quantized = (sample * 32767.0) as i16;
            frame[0] = sample_quantized as u16; // 左声道 - 正弦波
            frame[1] = sample_quantized as u16; // 右声道 - 正弦波
        }
        info!("Generated wavetable for {} Hz", freq);
    }

    // i2s configuration
    let mut dma_buffer = [0u16; 2400];

    let mut i2s_config = Config::default();
    i2s_config.format = Format::Data16Channel32;
    i2s_config.master_clock = false;
    let mut i2s = I2S::new_txonly_nomck(
        p.SPI3,
        p.PB3,  // sd
        p.PA15, // ws
        p.PB3,  // ck
        p.DMA1_CH7,
        &mut dma_buffer,
        i2s_config,
    );
    i2s.start();
    
    info!("I2S started, beginning frequency sweep...");

    let mut current_freq_index = 0;
    let mut frame_counter = 0;
    
    loop {
        // 选择当前频率的wavetable
        let current_wavetable = &wavetables[current_freq_index];
        let current_freq = frequencies[current_freq_index];
        
        // 每传输一定次数后切换频率
        frame_counter += 1;
        if frame_counter >= 100 { // 每100帧切换一次频率
            current_freq_index = (current_freq_index + 1) % frequencies.len();
            frame_counter = 0;
            info!("Switching to frequency: {} Hz", frequencies[current_freq_index]);
        }
        
        // 传输当前wavetable
        match i2s.write(current_wavetable).await {
            Ok(_) => {
                // 成功传输
            }
            Err(e) => {
                error!("I2S write error: {:?}", e);
                // 简单的错误恢复
                i2s.stop();
                Timer::after_millis(10).await;
                i2s.start();
                Timer::after_millis(10).await;
            }
        }
        
    }
}