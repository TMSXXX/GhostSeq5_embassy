#![no_std]
#![no_main]

// --------------------------------------------------------------------------
// 1. (添加) 声明我们的新模块
// --------------------------------------------------------------------------
mod synth;
use micromath::F32Ext;
use core::fmt::Write;
use defmt::*;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    Peri, adc::{Adc, SampleTime}, exti::ExtiInput, gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed}, i2c::I2c, i2s, interrupt::{self, InterruptExt, Priority}, time::Hertz
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use heapless::String;
use once_cell::sync::OnceCell;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use core::f32::consts::PI;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_graphics::{
    mono_font::MonoTextStyle,
    primitives::{Line, PrimitiveStyle},
};
use embedded_graphics::{mono_font::ascii::FONT_6X10, text::Baseline};
use embedded_graphics::{pixelcolor::BinaryColor, primitives::Rectangle};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

// --------------------------------------------------------------------------
// 2. (清理) 移除所有 synth 相关的 struct 和 const
//    (FmParams, Envelope, NOTE_FREQUENCIES)
// --------------------------------------------------------------------------

type OledDisplay = Ssd1306<
    I2CInterface<I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::Master>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
>;
type OledText = String<16>;

// ... (DMA consts) ...
const AUDIO_DMA_BUF_SIZE: usize = 2400;
static DMA_BUF_CELL: StaticCell<[u16; AUDIO_DMA_BUF_SIZE]> = StaticCell::new();
const HALF_DMA_LEN: usize = AUDIO_DMA_BUF_SIZE / 2; // 1200
const SAMPLES_PER_BUFFER: usize = HALF_DMA_LEN / 2; // 600
static AUDIO_BUFFERS: StaticCell<[[u16; HALF_DMA_LEN]; 2]> = StaticCell::new();

const HAAS_DELAY_MS: usize = 20; // 20ms 延迟
const HAAS_DELAY_SIZE: usize = (48000 * HAAS_DELAY_MS) / 1000; // 48000Hz * 0.020s = 960 个采样
static HAAS_DELAY_LINE: StaticCell<[i16; HAAS_DELAY_SIZE]> = StaticCell::new();

// ... (SINE_TABLE const 和 static) ...
const WAVE_TABLE_SIZE: usize = 1024;
static SINE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static SQUARE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static SAWTOOTH_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static TRIANGLE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();

// ... (get_sine_table() 函数) ...
fn get_sine_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SINE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            let phase = (i as f32 / WAVE_TABLE_SIZE as f32) * (2.0 * PI);
            table[i] = phase.sin();
        }
        info!("Sine table (1024 samples) generated.");
        table
    })
}
fn get_square_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SQUARE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            // 计算当前相位（0到2π）
            let phase = (i as f32 / WAVE_TABLE_SIZE as f32) * (2.0 * PI);
            // 50占空比：相位在0~π时为1.0，π~2π时为-1.0
            table[i] = if phase < PI { 1.0 } else { -1.0 };
        }
        info!("Square table (1024 samples) generated.");
        table
    })
}
fn get_sawtooth_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SAWTOOTH_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            // 计算归一化位置（0.0 ~ 1.0）
            let normalized = i as f32 / WAVE_TABLE_SIZE as f32;
            // 映射到[-1.0, 1.0]：0.0 → -1.0，1.0 → 1.0
            table[i] = 2.0 * normalized - 1.0;
        }
        info!("Sawtooth table (1024 samples) generated.");
        table
    })
}
fn get_triangle_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    TRIANGLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        // 半周期长度（前半段上升，后半段下降）
        let half = WAVE_TABLE_SIZE / 2;

        for i in 0..WAVE_TABLE_SIZE {
            if i < half {
                // 前半周期：从-1.0线性上升到1.0
                // 归一化位置（0.0~1.0）→ 映射到[-1.0, 1.0]
                let normalized = i as f32 / half as f32;
                table[i] = 2.0 * normalized - 1.0;
            } else {
                // 后半周期：从1.0线性下降到-1.0
                let normalized = (i - half) as f32 / half as f32;
                table[i] = 1.0 - 2.0 * normalized;
            }
        }

        info!("Triangle table (1024 samples) generated.");
        table
    })
}

// ... (Enums) ...
#[derive(Debug, Clone, Copy)]
enum Keyboard {
    KeyPress(u8),
    KeyRelease(u8),
}

#[derive(Debug, Clone, Copy)]
enum AudioCommand {
    Play(f32),
    Stop,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Waveform {
    Sine,
    Triangle,
    Sawtooth,
    Square,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaveParams {
    pub carrier_wave: Waveform,
    pub mod_wave: Waveform,
}

// --------------------------------------------------------------------------
// 3. (清理) 移除 synth 相关的通道
//    (FmParams, FM_PARAM_CHANNEL)
//    它们现在通过 synth:: 访问
// --------------------------------------------------------------------------
static KEYBOARD_CHANNEL: Channel<CriticalSectionRawMutex, Keyboard, 8> = Channel::new();
static FREQUENCY_CHANNEL: Channel<CriticalSectionRawMutex, f32, 2> = Channel::new();
static OLED_TEXT_CHANNEL: Channel<CriticalSectionRawMutex, OledText, 2> = Channel::new();
static AUDIO_CHANNEL: Channel<CriticalSectionRawMutex, AudioCommand, 4> = Channel::new();
static AMP_CHANNEL: Channel<CriticalSectionRawMutex, f32, 2> = Channel::new();
static WAVE_PARAMS_CHANNEL: Channel<CriticalSectionRawMutex, WaveParams, 2> = Channel::new();

// ... (Executor 和中断定义) ...
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

#[embassy_stm32::interrupt]
unsafe fn TIM2() {
    EXECUTOR_HIGH.on_interrupt();
}
#[embassy_stm32::interrupt]
unsafe fn TIM3() {
    EXECUTOR_MED.on_interrupt();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    enable_fpu();
    let config = {
        // ... (config 结构体完全不变) ...
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
    info!("System starting...");

    Timer::after_millis(100).await;

    // 初始化I2C和OLED
    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.frequency = Hertz(400_000);

    let i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_config);
    info!("I2C initialized.");

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    info!("OLED display created.");

    // 配置键盘
    let keys: [[Peri<'static, AnyPin>; 4]; 2] = [
        [p.PA3.into(), p.PA2.into(), p.PA1.into(), p.PA0.into()],
        [p.PA7.into(), p.PA6.into(), p.PA5.into(), p.PA4.into()],
    ];
    info!("Keyboard pins configured.");

    // 配置电位器
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES15);

    // 使用官方示例的I2S配置
    let mut i2s_config = i2s::Config::default();
    i2s_config.format = i2s::Format::Data16Channel32;
    i2s_config.master_clock = false;
    i2s_config.frequency = Hertz(48000);

    info!("I2S config set.");

    // 配置编码器
    let enc_a = ExtiInput::new(p.PA9, p.EXTI9, Pull::Up);
    let enc_b = ExtiInput::new(p.PA8, p.EXTI8, Pull::Up);
    let enc_sw = ExtiInput::new(p.PA10, p.EXTI10, Pull::Up);

    // I2S初始化
    let i2s = i2s::I2S::new_txonly_nomck(
        p.SPI3,     // SPI3
        p.PB5,      // sd
        p.PA15,     // ws
        p.PB3,      // ck
        p.DMA1_CH7, // txdma
        DMA_BUF_CELL.init([0u16; AUDIO_DMA_BUF_SIZE]),
        i2s_config,
    );
    info!("I2S configured.");

    // 初始化OLED
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();

    get_sine_table();
    get_sawtooth_table();
    get_square_table();
    get_triangle_table();

    interrupt::TIM2.set_priority(Priority::P3);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::TIM2);

    interrupt::TIM3.set_priority(Priority::P7);
    let spawner_med = EXECUTOR_MED.start(interrupt::TIM3);
    

    // LOW (P15)
    spawner.spawn(oled_task(display)).unwrap();
    spawner.spawn(synth::adc_task(adc, p.PB0)).unwrap();

    // MEDIUM (P7):
    spawner_med.spawn(synth::control_task(keys)).unwrap(); // <-- 调用新模块的任务
    spawner_med.spawn(synth::encoder_task(enc_a, enc_b, enc_sw)).unwrap();

    // HIGH (P3)
    spawner_high.spawn(audio_task(i2s)).unwrap();

    info!("All tasks started, system ready!");
}

fn enable_fpu() {
    unsafe {
        let scb = cortex_m::peripheral::SCB::ptr();
        (*scb).cpacr.modify(|r| r | (0b1111 << 20));
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

// --------------------------------------------------------------------------
// 5. (清理) 移除 keyboard_task 和 synth_task
//    它们现在在 synth.rs 中
// --------------------------------------------------------------------------
// [DELETE] #[embassy_executor::task] async fn keyboard_task(...)
// [DELETE] #[embassy_executor::task] async fn synth_task(...)

// ... (oled_task 保持不变) ...
#[embassy_executor::task]
async fn oled_task(mut display: OledDisplay) {
    info!("OLED task started!");

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let clear_style = PrimitiveStyle::with_fill(BinaryColor::Off);

    // 状态变量
    let mut frequency: f32 = 0.0;
    let mut key_text: String<16> = String::new();
    core::write!(key_text, "Init...").unwrap();

    // 标志位
    let mut freq_dirty = true;
    let mut text_dirty = true;

    // 初始显示
    display.clear(BinaryColor::Off).unwrap();
    // (修复：确保初始文本也被绘制)
    Rectangle::new(Point::new(0, 34), Size::new(128, 12)) // (y=34, h=12)
        .into_styled(clear_style)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline(&key_text, Point::new(0, 34), text_style, Baseline::Top) // (y=34)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();
    text_dirty = false; // 我们已经画过了

    loop {
        // --- 1. 非阻塞地排空所有挂起的消息 ---
        if let Ok(new_frequency) = FREQUENCY_CHANNEL.try_receive() {
            frequency = new_frequency;
            freq_dirty = true;
        }
        if let Ok(new_text) = OLED_TEXT_CHANNEL.try_receive() {
            key_text = new_text;
            text_dirty = true;
        }

        // --- 2. 仅在需要时重绘 ---

        if freq_dirty {
            let mut freq_text: String<16> = String::new();
            core::write!(freq_text, "Freq: {} Hz", frequency as i32).unwrap();

            Rectangle::new(Point::new(0, 10), Size::new(128, 12))
                .into_styled(clear_style)
                .draw(&mut display)
                .unwrap();
            Text::with_baseline(&freq_text, Point::new(0, 10), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
        }

        if text_dirty {
            Rectangle::new(Point::new(0, 34), Size::new(128, 12))
                .into_styled(clear_style)
                .draw(&mut display)
                .unwrap();
            Text::with_baseline(&key_text, Point::new(0, 34), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
        }

        // --- 3. 仅在有新内容时才阻塞刷新 ---
        if freq_dirty || text_dirty {
            display.flush().unwrap();
        }

        freq_dirty = false;
        text_dirty = false;

        // --- 4. Await (释放 CPU) ---
        Timer::after_millis(10).await;
    }
}

// ... (audio_task 保持不变) ...
#[embassy_executor::task()]
async fn audio_task(mut i2s: i2s::I2S<'static, u16>) {
    info!("Audio task starting...");

    // (P3 本地参数 不变)
    let mut frequency = 100.0f32;
    let mut is_on = false;
    let mut carrier_phase: f32 = 0.0;
    let mut modulator_phase: f32 = 0.0;
    let mut params = synth::FmParams { index: 1.5, ratio: 2.0 }; 
    let mut amplitude: f32 = 0.0; 
    let mut wave_params = WaveParams {
        carrier_wave: Waveform::Triangle,
        mod_wave: Waveform::Square,
    };

    let audio_buffers = AUDIO_BUFFERS.init([[0u16; HALF_DMA_LEN]; 2]);
    let mut current_buffer_idx = 0;

    // (Haas 初始化 不变)
    let haas_delay_line = HAAS_DELAY_LINE.init([0i16; HAAS_DELAY_SIZE]);
    let mut haas_write_ptr: usize = 0;
    let mut haas_active: bool = false; 

    const TABLE_MASK: usize = WAVE_TABLE_SIZE - 1; 
    const TABLE_SIZE_F32: f32 = WAVE_TABLE_SIZE as f32;
    const TWO_PI: f32 = 2.0 * PI;
    const TWO_PI_INV: f32 = 0.15915494; 

    // -----------------------------------------------------------------
    // 修复: 把“波形表”的获取（Get）移到循环 *之外*
    // -----------------------------------------------------------------
    let sine_table = SINE_TABLE.get().unwrap();
    let sawtooth_table = SAWTOOTH_TABLE.get().unwrap();
    let square_table = SQUARE_TABLE.get().unwrap();
    let triangle_table = TRIANGLE_TABLE.get().unwrap();

    let mut fill_buffer =
        |buffer: &mut [u16; HALF_DMA_LEN], freq: f32, p: &synth::FmParams, 
         wp: &WaveParams, // <-- 传入波形参数
         amp: f32, 
         cp: &mut f32, mp: &mut f32, on: bool,
         hdl: &mut [i16; HAAS_DELAY_SIZE], 
         hwp: &mut usize, 
         haas_on: bool
        | {
            
            // (我们现在是“借用”这些表，不再调用 get_..._table())
            
            let carrier_freq = freq;
            let modulator_freq = carrier_freq * p.ratio; 
            let carrier_phase_increment = (TWO_PI * carrier_freq) / 48000.0;
            let modulator_phase_increment = (TWO_PI * modulator_freq) / 48000.0;
            
            for i in 0..SAMPLES_PER_BUFFER {
                
                // 1. 计算单声道 (Mono) 样本
                let sample_f32 = if on {
                    // (调制波 B)
                    let mod_phase_rads = *mp;
                    let mod_index_f32 = (mod_phase_rads * TWO_PI_INV) * TABLE_SIZE_F32; 
                    let mod_idx0 = (mod_index_f32 as i32) as usize & TABLE_MASK;
                    
                    let mod_val = //triangle_table[mod_idx0];
                    match wp.mod_wave {
                        Waveform::Sine => sine_table[mod_idx0],
                        Waveform::Triangle => triangle_table[mod_idx0],
                        Waveform::Sawtooth => sawtooth_table[mod_idx0],
                        Waveform::Square => square_table[mod_idx0],
                    };
                    
                    let phase_offset = mod_val * p.index; 
                    let mut carrier_phase_rads = *cp + phase_offset;
                    while carrier_phase_rads < 0.0 { 
                        carrier_phase_rads += TWO_PI;
                    }

                    // (载波 A)
                    let carrier_index_f32 = (carrier_phase_rads * TWO_PI_INV) * TABLE_SIZE_F32;
                    let carrier_index_trunc_int = carrier_index_f32 as i32;
                    let carrier_index_trunc = carrier_index_trunc_int as f32;
                    let carrier_index_frac = carrier_index_f32 - carrier_index_trunc;
                    let carrier_idx0 = carrier_index_trunc_int as usize; 
                    let carrier_idx0_wrapped = carrier_idx0 & TABLE_MASK;
                    let carrier_idx1_wrapped = (carrier_idx0 + 1) & TABLE_MASK;
                    
                    let (val0, val1) = //(sine_table[carrier_idx0_wrapped], sine_table[carrier_idx1_wrapped]);
                    match wp.carrier_wave {
                        Waveform::Sine => (sine_table[carrier_idx0_wrapped], sine_table[carrier_idx1_wrapped]),
                        Waveform::Triangle => (triangle_table[carrier_idx0_wrapped], triangle_table[carrier_idx1_wrapped]),
                        Waveform::Sawtooth => (sawtooth_table[carrier_idx0_wrapped], sawtooth_table[carrier_idx1_wrapped]),
                        Waveform::Square => (square_table[carrier_idx0_wrapped], square_table[carrier_idx1_wrapped]),
                    };
                    (val1 - val0).mul_add(carrier_index_frac, val0)
                } else {
                    0.0
                };

                let mono_sample_i16 = (sample_f32 * amp * 32767.0) as i16;

                // (Haas 效应 不变)
                let read_ptr = *hwp;
                let delayed_sample_i16 = hdl[read_ptr];
                hdl[read_ptr] = mono_sample_i16;
                *hwp += 1;
                if *hwp >= HAAS_DELAY_SIZE { *hwp = 0; }
                
                if haas_on {
                    buffer[i * 2] = mono_sample_i16 as u16; 
                    buffer[i * 2 + 1] = delayed_sample_i16 as u16;
                } else {
                    buffer[i * 2] = mono_sample_i16 as u16;
                    buffer[i * 2 + 1] = mono_sample_i16 as u16;
                }

                // (推进相位 不变)
                *cp += carrier_phase_increment;
                *mp += modulator_phase_increment;
                if *cp > TWO_PI { *cp -= TWO_PI; }
                if *mp > TWO_PI { *mp -= TWO_PI; }
            }
            
            if !on { *cp = 0.0; *mp = 0.0; }
        };

    // (预填充)
    fill_buffer(
        &mut audio_buffers[0],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
    );
    fill_buffer(
        &mut audio_buffers[1],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
    );

    i2s.start();
    info!("I2S started!");
    
    let mut write_future = i2s.write(&audio_buffers[current_buffer_idx]);

    loop {
        match write_future.await {
            Ok(_) => {}
            Err(e) => {
                error!("I2S write error: {:?}", e); 
                audio_buffers[0].fill(0); 
                audio_buffers[1].fill(0); 
                carrier_phase = 0.0; modulator_phase = 0.0; 
                current_buffer_idx = 0; 
                i2s.clear();
                haas_delay_line.fill(0);
                haas_write_ptr = 0; 
                //is_on = false;
                write_future = i2s.write(&audio_buffers[current_buffer_idx]);
                continue; 
            }
        }
        
        // (缓冲区拆分逻辑 不变)
        let (buf0_slice, buf1_slice) = audio_buffers.split_at_mut(1);
        let buf0 = &mut buf0_slice[0];
        let buf1 = &mut buf1_slice[0]; 
        current_buffer_idx ^= 1;
        let (buf_to_write, buf_to_fill);
        if current_buffer_idx == 0 {
            buf_to_write = buf0; buf_to_fill = buf1;
        } else {
            buf_to_write = buf1; buf_to_fill = buf0;
        }
        
        // (P3 消息接收逻辑 不变)
        if let Ok(new_params) = synth::FM_PARAM_CHANNEL.try_receive() {
            params = new_params; 
        }
        if let Ok(new_amp) = AMP_CHANNEL.try_receive() {
            amplitude = new_amp;
        }
        if let Ok(haas_on) = synth::HAAS_STATE_CHANNEL.try_receive() {
            haas_active = haas_on;
        }
        if let Ok(new_waves) = WAVE_PARAMS_CHANNEL.try_receive() {
            wave_params = new_waves;
        }
        while let Ok(command) = AUDIO_CHANNEL.try_receive() {
            match command {
                AudioCommand::Play(freq) => {
                    frequency = freq;
                    is_on = true;
                }
                AudioCommand::Stop => {
                    is_on = false;
                }
            }
        }
        
        if !is_on {
            buf_to_write.fill(0); 
        }
        
        write_future = i2s.write(buf_to_write);
        
        // (填充缓冲区)
        fill_buffer(
            buf_to_fill,
            frequency,
            &params, 
            &wave_params, 
            amplitude, 
            &mut carrier_phase,
            &mut modulator_phase,
            is_on,
            haas_delay_line, 
            &mut haas_write_ptr, 
            haas_active
        );
    }
}